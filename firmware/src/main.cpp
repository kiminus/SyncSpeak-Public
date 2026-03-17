/**
 * INMP441 I2S Microphone → WiFi TCP Audio Stream with ACK buffer
 * Board: NodeMCU ESP-32S
 * Framework: Arduino (PlatformIO)
 *
 * Architecture: Producer / Consumer with ACK-pending buffer
 *   mic_capture_task  (Core 1, pri 5) — fills ack_buf slots, enqueues indices
 *   net_send_task     (Core 0, pri 4) — dequeues, sends, manages ACK buffer
 *
 * Buffer model:
 *   conn_queue  : small index queue between tasks (uint8_t slot indices)
 *   ack_buf[]   : main buffer (~160 KB); slots live here until server ACKs.
 *                 New chunks are dropped if B_MAX_BYTES is exhausted.
 *                 Slots are expired after T_MAX_MS without an ACK.
 *
 * Wire protocol (raw TCP):
 *   ESP32 → Server : [uint32_t seq_id LE] [CHUNK_BYTES int16_t PCM LE]
 *   Server → ESP32 : [uint32_t seq_id LE]  (ACK — frees the slot)
 *
 * Wiring:
 *   INMP441 VDD  → 3.3V   INMP441 GND  → GND
 *   INMP441 SD   → GPIO32  INMP441 SCK  → GPIO14
 *   INMP441 WS   → GPIO15  INMP441 L/R  → GND
 */

#include <Arduino.h>
#include <WiFi.h>
#include <driver/i2s.h>
#include "constants.h"

/* ═══════════════════════════════════════════════════════════════════
 *  TUNABLE PARAMETERS
 * ═══════════════════════════════════════════════════════════════════ */
#define SAMPLE_RATE        16000          /* Hz                               */
#define T_P_MS             200            /* chunk period ms  →  mic fills one
                                           * chunk every T_P_MS               */
#define CHUNK_SAMPLES      (SAMPLE_RATE * T_P_MS / 1000)   /* 3200 samples   */
#define CHUNK_BYTES        (CHUNK_SAMPLES * 2)              /* 6400 bytes     */

#define MAX_BUFFER_CHUNKS  15             /* 15 × 6400 B ≈ 96 KB heap alloc;
                                           * static BSS cannot hold 160 KB —
                                           * dram0_0_seg is shared with WiFi  */
#define B_MAX_BYTES        (MAX_BUFFER_CHUNKS * CHUNK_BYTES)
#define T_MAX_MS           5000           /* drop slot if no ACK within this  */

#define CONN_QUEUE_DEPTH   4              /* mic→net handoff queue (indices)  */

/* ── I2S / Mic Pins ─────────────────────────────────────────────── */
#define I2S_SD_PIN         32
#define I2S_SCK_PIN        14
#define I2S_WS_PIN         15
#define I2S_PORT           I2S_NUM_0
#define DMA_BUF_COUNT      8
#define DMA_BUF_LEN        1024
#define READ_BUF_SAMPLES   512            /* int32_t words per i2s_read call  */

/* ── Filter ─────────────────────────────────────────────────────── */
/* Single-pole IIR high-pass:  y[n] = (1-α)(y[n-1] + x[n] - x[n-1])
 * HP_ALPHA = fc/(fc+fs): 0.005 ≈ 80 Hz | 0.01 ≈ 160 Hz | 0.02 ≈ 320 Hz   */
#define HP_ALPHA           0.01f
#define NOISE_GATE_THRESH  100            /* int16 units; 0 = disabled        */

/* ═══════════════════════════════════════════════════════════════════
 *  ACK BUFFER
 *
 *  Each entry is one chunk of audio.  Lifecycle:
 *    FREE  →  CLAIMED (mic writes)  →  IN_QUEUE (conn_queue)
 *          →  PENDING_ACK (sent to server)  →  FREE (ACK received or timeout)
 * ═══════════════════════════════════════════════════════════════════ */
typedef struct {
    int16_t    data[CHUNK_SAMPLES]; /* 6400 bytes of PCM                     */
    uint32_t   seq_id;
    TickType_t sent_at;             /* tick when transmitted; for T_MAX check */
    bool       in_use;              /* slot is claimed (mic or net)           */
    bool       awaiting_ack;        /* true once sent to server               */
} BufEntry;

static BufEntry         *ack_buf = nullptr;           /* heap-allocated ~96 KB */
static SemaphoreHandle_t buf_mutex;
static volatile uint32_t next_seq_id = 0;   /* written only by mic task     */

/* conn_queue holds uint8_t slot indices — mic producer, net consumer */
static QueueHandle_t conn_queue;

/* ── Slot helpers ────────────────────────────────────────────────── */

/* Claim a free slot; returns index or -1 if buffer full.
 * Only called from mic_capture_task, so next_seq_id needs no mutex. */
static int claim_slot()
{
    xSemaphoreTake(buf_mutex, portMAX_DELAY);
    int slot = -1;
    for (int i = 0; i < MAX_BUFFER_CHUNKS; i++) {
        if (!ack_buf[i].in_use) {
            ack_buf[i].in_use       = true;
            ack_buf[i].awaiting_ack = false;
            ack_buf[i].seq_id       = next_seq_id++;
            slot = i;
            break;
        }
    }
    xSemaphoreGive(buf_mutex);
    return slot;
}

static void free_slot(int i)
{
    xSemaphoreTake(buf_mutex, portMAX_DELAY);
    ack_buf[i].in_use = false;
    xSemaphoreGive(buf_mutex);
}

/* On disconnect: release all sent-but-unACKed slots so the mic can reuse them. */
static void free_pending_slots()
{
    xSemaphoreTake(buf_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_BUFFER_CHUNKS; i++) {
        if (ack_buf[i].in_use && ack_buf[i].awaiting_ack)
            ack_buf[i].in_use = false;
    }
    xSemaphoreGive(buf_mutex);
}

/* ═══════════════════════════════════════════════════════════════════
 *  HIGH-PASS FILTER  y[n] = (1-α)(y[n-1] + x[n] - x[n-1])
 *  Only called from mic_capture_task — no lock needed.
 * ═══════════════════════════════════════════════════════════════════ */
static float hp_prev_in  = 0.0f;
static float hp_prev_out = 0.0f;

static inline int16_t highpass_filter(int16_t input)
{
    float in_f  = (float)input;
    float out_f = (1.0f - HP_ALPHA) * (hp_prev_out + in_f - hp_prev_in);
    hp_prev_in  = in_f;
    hp_prev_out = out_f;
    if (out_f >  32767.0f) out_f =  32767.0f;
    if (out_f < -32768.0f) out_f = -32768.0f;
    int16_t result = (int16_t)out_f;
    if (NOISE_GATE_THRESH > 0 && abs(result) < NOISE_GATE_THRESH) result = 0;
    return result;
}

/* ═══════════════════════════════════════════════════════════════════
 *  I2S INIT
 * ═══════════════════════════════════════════════════════════════════ */
static void i2s_init()
{
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    cfg.sample_rate          = SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT; /* INMP441 needs 64 BCK/sample */
    cfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = DMA_BUF_COUNT;
    cfg.dma_buf_len          = DMA_BUF_LEN;
    cfg.use_apll             = true;
    cfg.tx_desc_auto_clear   = false;
    cfg.fixed_mclk           = 0;

    i2s_pin_config_t pins = {};
    pins.bck_io_num   = I2S_SCK_PIN;
    pins.ws_io_num    = I2S_WS_PIN;
    pins.data_out_num = I2S_PIN_NO_CHANGE;
    pins.data_in_num  = I2S_SD_PIN;

    i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
    i2s_set_pin(I2S_PORT, &pins);
    i2s_start(I2S_PORT);
    Serial.printf("I2S mic ready (SR=%d Hz, HP~%d Hz)\n",
                  SAMPLE_RATE, (int)(HP_ALPHA * SAMPLE_RATE));
}

/* ═══════════════════════════════════════════════════════════════════
 *  PRODUCER — mic_capture_task  (Core 1, priority 5)
 *
 *  Claims an ack_buf slot, fills it sample-by-sample from the I2S DMA.
 *  When CHUNK_SAMPLES are collected, the slot index is pushed to
 *  conn_queue for the net task.
 *
 *  If ack_buf is full (B_MAX_BYTES reached): samples are discarded
 *  (I2S DMA keeps draining) until a slot becomes free via ACK/timeout.
 *  If conn_queue is full (net task stuck): the filled slot is freed and
 *  the chunk is dropped.
 * ═══════════════════════════════════════════════════════════════════ */
static void mic_capture_task(void *arg)
{
    static int32_t raw[READ_BUF_SAMPLES];

    /* Warm up: settle the IIR filter before recording real audio */
    {
        size_t dummy;
        for (int i = 0; i < 8; i++) {
            i2s_read(I2S_PORT, raw, sizeof(raw), &dummy, pdMS_TO_TICKS(50));
            int n = (int)(dummy / sizeof(int32_t));
            for (int j = 0; j < n; j++)
                highpass_filter((int16_t)(raw[j] >> 16));
        }
        Serial.println("mic: filter settled");
    }

    int write_slot = claim_slot();   /* -1 if buffer already full at boot */
    int buf_count  = 0;

    while (1) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_read(I2S_PORT, raw, sizeof(raw),
                                 &bytes_read, pdMS_TO_TICKS(50));
        if (err != ESP_OK) continue;

        int count = (int)(bytes_read / sizeof(int32_t));

        /* No free slot yet — try to claim one; drop this DMA block if still full */
        if (write_slot < 0) {
            write_slot = claim_slot();
            if (write_slot < 0) continue;
            buf_count = 0;
        }

        /* Fill slot — top 16 bits of 32-bit I2S word are the INMP441's 16 MSBs */
        for (int i = 0; i < count && buf_count < CHUNK_SAMPLES; i++) {
            ack_buf[write_slot].data[buf_count++] =
                highpass_filter((int16_t)(raw[i] >> 16));
        }

        if (buf_count < CHUNK_SAMPLES) continue;   /* chunk not full yet */

        /* Hand off to net task */
        uint8_t slot_u8 = (uint8_t)write_slot;
        if (xQueueSend(conn_queue, &slot_u8, 0) != pdTRUE) {
            /* conn_queue full: net is stuck; drop this chunk */
            free_slot(write_slot);
            Serial.println("mic: conn_queue full — dropped chunk");
        }

        write_slot = claim_slot();   /* -1 = buffer full, will discard next */
        buf_count  = 0;
    }
}

/* Shared TCP client + connection state.
 * net_send_task owns connect/disconnect; net_recv_task only reads.
 * TCP is full-duplex so concurrent send/recv on the same socket is safe. */
static WiFiClient     net_client;
static volatile bool  net_alive = false;

/* ═══════════════════════════════════════════════════════════════════
 *  net_send_task  (Core 0, priority 4)
 *
 *  Owns WiFi reconnection and chunk transmission.
 *  Sets net_alive=true after connect, false on any send failure.
 *  Does NOT touch ACKs — that is net_recv_task's job.
 * ═══════════════════════════════════════════════════════════════════ */
static void net_send_task(void *arg)
{
    while (1) {
        while (WiFi.status() != WL_CONNECTED) {
            Serial.println("net: waiting for WiFi...");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        net_client.setNoDelay(true);
        if (!net_client.connect(SERVER_HOST, SERVER_PORT)) {
            Serial.printf("net: connect to %s:%d failed\n", SERVER_HOST, SERVER_PORT);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        Serial.printf("net: connected to %s:%d\n", SERVER_HOST, SERVER_PORT);
        net_alive = true;

        while (net_alive) {
            uint8_t slot;
            if (xQueueReceive(conn_queue, &slot, pdMS_TO_TICKS(T_P_MS)) != pdTRUE)
                continue;

            BufEntry *e = &ack_buf[slot];
            bool ok =
                net_client.write((const uint8_t *)&e->seq_id, sizeof(e->seq_id)) == sizeof(e->seq_id) &&
                net_client.write((const uint8_t *)e->data,    CHUNK_BYTES)        == CHUNK_BYTES;

            if (ok) {
                xSemaphoreTake(buf_mutex, portMAX_DELAY);
                e->awaiting_ack = true;
                e->sent_at      = xTaskGetTickCount();
                xSemaphoreGive(buf_mutex);
            } else {
                free_slot(slot);
                net_alive = false;
                Serial.println("net: send failed — reconnecting");
            }
        }

        net_client.stop();
        free_pending_slots();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  net_recv_task  (Core 0, priority 3)
 *
 *  Independently polls the socket for incoming ACKs.  Because it runs
 *  concurrently with net_send_task on the same socket (full-duplex TCP),
 *  it timestamps each ACK the instant it arrives — no loop quantization.
 *
 *  Also owns the T_MAX_MS expiry scan (runs every T_P_MS).
 * ═══════════════════════════════════════════════════════════════════ */
static void net_recv_task(void *arg)
{
    TickType_t last_expire = xTaskGetTickCount();

    while (1) {
        /* Wait for a connection to be established */
        if (!net_alive) {
            vTaskDelay(pdMS_TO_TICKS(50));
            last_expire = xTaskGetTickCount();
            continue;
        }

        /* ── Drain all available ACKs ─────────────────────── */
        while (net_alive && net_client.available() >= (int)sizeof(uint32_t)) {
            uint32_t ack_id = 0;
            net_client.readBytes((char *)&ack_id, sizeof(ack_id));
            TickType_t now = xTaskGetTickCount();

            xSemaphoreTake(buf_mutex, portMAX_DELAY);
            for (int i = 0; i < MAX_BUFFER_CHUNKS; i++) {
                if (ack_buf[i].in_use && ack_buf[i].awaiting_ack &&
                    ack_buf[i].seq_id == ack_id) {
                    uint32_t rtt_ms = (now - ack_buf[i].sent_at) * portTICK_PERIOD_MS;
                    ack_buf[i].in_use = false;

                    int used = 0;
                    for (int j = 0; j < MAX_BUFFER_CHUNKS; j++)
                        if (ack_buf[j].in_use) used++;
                    used += (int)uxQueueMessagesWaiting(conn_queue);

                    Serial.printf("ACK seq=%-5u  t=%lums  RTT=%ums  buf=%dKB\n",
                                  ack_id, millis(), rtt_ms,
                                  (used * CHUNK_BYTES) / 1024);
                    break;
                }
            }
            xSemaphoreGive(buf_mutex);
        }

        /* ── Expire timed-out slots every T_P_MS ─────────── */
        TickType_t now = xTaskGetTickCount();
        if ((now - last_expire) >= pdMS_TO_TICKS(T_P_MS)) {
            xSemaphoreTake(buf_mutex, portMAX_DELAY);
            for (int i = 0; i < MAX_BUFFER_CHUNKS; i++) {
                if (ack_buf[i].in_use && ack_buf[i].awaiting_ack &&
                    (now - ack_buf[i].sent_at) >= pdMS_TO_TICKS(T_MAX_MS)) {
                    ack_buf[i].in_use = false;
                    Serial.printf("net: seq %u timed out\n", ack_buf[i].seq_id);
                }
            }
            xSemaphoreGive(buf_mutex);
            last_expire = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1));   /* 1ms poll — accurate RTT timestamps */
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  Arduino entry points
 * ═══════════════════════════════════════════════════════════════════ */
void setup()
{
    Serial.begin(115200);
    Serial.printf("=== INMP441 WiFi Audio Pusher  B_max=%luKB  T_max=%dms ===\n",
                  B_MAX_BYTES / 1024, T_MAX_MS);

    ack_buf = (BufEntry *)malloc(MAX_BUFFER_CHUNKS * sizeof(BufEntry));
    if (!ack_buf) {
        Serial.printf("FATAL: cannot allocate %u B for ack_buf (free heap: %u)\n",
                      MAX_BUFFER_CHUNKS * sizeof(BufEntry), esp_get_free_heap_size());
        while (1) vTaskDelay(1000);
    }
    memset(ack_buf, 0, MAX_BUFFER_CHUNKS * sizeof(BufEntry));
    Serial.printf("ack_buf: %u chunks × %u B = %u B  (free heap after: %u)\n",
                  MAX_BUFFER_CHUNKS, sizeof(BufEntry),
                  MAX_BUFFER_CHUNKS * sizeof(BufEntry), esp_get_free_heap_size());
    buf_mutex  = xSemaphoreCreateMutex();
    conn_queue = xQueueCreate(CONN_QUEUE_DEPTH, sizeof(uint8_t));

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("Connecting to '%s'...", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

    i2s_init();

    xTaskCreatePinnedToCore(mic_capture_task, "mic_cap",  4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(net_send_task,    "net_send", 8192, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(net_recv_task,    "net_recv", 4096, NULL, 3, NULL, 0);
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
