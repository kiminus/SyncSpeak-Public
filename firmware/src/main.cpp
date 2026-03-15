/**
 * INMP441 I2S Microphone → WiFi HTTP POST Audio Stream
 * Board: NodeMCU ESP-32S
 * Framework: Arduino (PlatformIO)
 *
 * Architecture: Producer / Consumer via FreeRTOS queues
 *   mic_capture_task  (Core 1, pri 5) — reads I2S, filters, enqueues chunks
 *   net_send_task     (Core 0, pri 4) — dequeues chunks, POSTs over HTTP
 *
 * A fixed pool of QUEUE_DEPTH static buffers is shared between the two tasks.
 * The producer rotates through them with a simple index; no free-list needed.
 * Recording never stalls: if the net falls behind, the oldest queued chunk
 * is evicted (it points to the same slot write_idx is about to reuse anyway).
 *
 * Wiring:
 *   INMP441 VDD  → 3.3V   INMP441 GND  → GND
 *   INMP441 SD   → GPIO32  INMP441 SCK  → GPIO14
 *   INMP441 WS   → GPIO15  INMP441 L/R  → GND
 */

#include <Arduino.h>
#include <WiFi.h>
#include <driver/i2s.h>

/* ═══════════════════════════════════════════════════════════════════
 *  CONFIGURATION
 * ═══════════════════════════════════════════════════════════════════ */
#include "constants.h"   /* extern declarations for secrets (defined in secrets.cpp) */

#define SERVER_PATH        "/audio"

#define SEND_INTERVAL_MS   200
#define SAMPLE_RATE        16000
/* Samples per chunk: 200ms worth */
#define CHUNK_SAMPLES      (SAMPLE_RATE * SEND_INTERVAL_MS / 1000)   /* 3200 */

/* ── Queue / buffer pool ─────────────────────────────────────────── */
/* Number of audio buffers in the shared pool.
 * Each is CHUNK_SAMPLES * 2 bytes = 6400 bytes → pool = 25600 bytes total. */
#define QUEUE_DEPTH        4

/* ── I2S / Mic Pins ─────────────────────────────────────────────── */
#define I2S_SD_PIN         32
#define I2S_SCK_PIN        14
#define I2S_WS_PIN         15
#define I2S_PORT           I2S_NUM_0
#define DMA_BUF_COUNT      8
#define DMA_BUF_LEN        1024
#define READ_BUF_SAMPLES   512

/* ── Filter Settings ────────────────────────────────────────────── */
/* Single-pole IIR high-pass filter.
 *   HP_ALPHA = fc / (fc + fs)   →   0.01 ≈ 160 Hz cutoff @ 16 kHz
 *   0.005 = ~80 Hz  |  0.01 = ~160 Hz  |  0.02 = ~320 Hz          */
#define HP_ALPHA           0.01f

/* Noise gate: zero out samples below this amplitude. 0 = disabled. */
#define NOISE_GATE_THRESH  100

/* ═══════════════════════════════════════════════════════════════════
 *  Shared queue infrastructure
 *
 *  audio_queue holds int16_t* pointers into chunk_pool.
 *  The producer rotates write_idx; the consumer just reads and sends.
 *  No free-list, no struct — the circular index IS the free-list.
 * ═══════════════════════════════════════════════════════════════════ */
static int16_t       chunk_pool[QUEUE_DEPTH][CHUNK_SAMPLES];
static QueueHandle_t audio_queue;

/* ═══════════════════════════════════════════════════════════════════
 *  High-pass filter  y[n] = (1-α)(y[n-1] + x[n] - x[n-1])
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
    if (NOISE_GATE_THRESH > 0 && abs(result) < NOISE_GATE_THRESH)
        result = 0;
    return result;
}

/* ═══════════════════════════════════════════════════════════════════
 *  I2S init
 * ═══════════════════════════════════════════════════════════════════ */
static void i2s_init()
{
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    cfg.sample_rate          = SAMPLE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = DMA_BUF_COUNT;
    cfg.dma_buf_len          = DMA_BUF_LEN;
    cfg.use_apll             = true;   /* APLL → jitter-free clock → less noise */
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
 *  PRODUCER — mic_capture_task
 *  Core 1, priority 5 (real-time)
 *
 *  Fills chunk_pool[write_idx] sample-by-sample.  When the buffer is
 *  full, it is pushed onto audio_queue.  If the queue is already full
 *  (net is lagging), the oldest pointer is evicted — it points to
 *  chunk_pool[(write_idx+1) % QUEUE_DEPTH], which is exactly the slot
 *  write_idx will overwrite next, so no separate free-list is needed.
 * ═══════════════════════════════════════════════════════════════════ */
static void mic_capture_task(void *arg)
{
    static int32_t raw[READ_BUF_SAMPLES];

    /* Warm up: settle the IIR filter before queueing real audio */
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

    int write_idx = 0;
    int buf_count = 0;

    while (1) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_read(I2S_PORT, raw, sizeof(raw),
                                 &bytes_read, pdMS_TO_TICKS(50));
        if (err != ESP_OK) continue;

        int count = (int)(bytes_read / sizeof(int32_t));

        for (int i = 0; i < count && buf_count < CHUNK_SAMPLES; i++) {
            chunk_pool[write_idx][buf_count++] = highpass_filter((int16_t)(raw[i] >> 16));
        }

        if (buf_count < CHUNK_SAMPLES) continue;   /* buffer not full yet */

        /* Buffer is full — enqueue the pointer */
        int16_t *ptr = chunk_pool[write_idx];

        if (uxQueueSpacesAvailable(audio_queue) == 0) {
            /* Net is lagging: evict the oldest entry (we are about to
             * overwrite that same slot on the next write_idx wrap anyway) */
            int16_t *dropped;
            xQueueReceive(audio_queue, &dropped, 0);
            Serial.println("mic: net lag — dropped oldest chunk");
        }
        xQueueSend(audio_queue, &ptr, 0);

        write_idx = (write_idx + 1) % QUEUE_DEPTH;
        buf_count = 0;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  CONSUMER — net_send_task
 *  Core 0, priority 4
 *
 *  Dequeues audio chunks and POSTs them to the server over a
 *  persistent HTTP chunked-encoding connection.  Reconnects
 *  automatically on error.
 * ═══════════════════════════════════════════════════════════════════ */
static void net_send_task(void *arg)
{
    while (1) {
        /* ── Wait for WiFi ─────────────────────────────────── */
        while (WiFi.status() != WL_CONNECTED) {
            Serial.println("net: waiting for WiFi...");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        /* ── Connect ───────────────────────────────────────── */
        WiFiClient client;
        client.setNoDelay(true);

        if (!client.connect(SERVER_HOST, SERVER_PORT)) {
            Serial.printf("net: connect to %s:%d failed\n", SERVER_HOST, SERVER_PORT);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        /* ── HTTP headers ──────────────────────────────────── */
        client.printf(
            "POST %s HTTP/1.1\r\n"
            "Host: %s:%d\r\n"
            "Content-Type: application/octet-stream\r\n"
            "X-Sample-Rate: %d\r\n"
            "X-Bits-Per-Sample: 16\r\n"
            "X-Channels: 1\r\n"
            "Transfer-Encoding: chunked\r\n"
            "Connection: keep-alive\r\n"
            "\r\n",
            SERVER_PATH, SERVER_HOST, SERVER_PORT, SAMPLE_RATE);

        Serial.printf("net: streaming to http://%s:%d%s\n",
                      SERVER_HOST, SERVER_PORT, SERVER_PATH);

        /* ── Send loop ─────────────────────────────────────── */
        while (client.connected()) {
            int16_t *ptr = nullptr;
            if (xQueueReceive(audio_queue, &ptr, pdMS_TO_TICKS(200)) != pdTRUE)
                continue;   /* timeout — just re-check connection */

            int byte_len = CHUNK_SAMPLES * (int)sizeof(int16_t);

            char hdr[16];
            snprintf(hdr, sizeof(hdr), "%x\r\n", byte_len);
            bool ok = client.print(hdr) &&
                      client.write((const uint8_t *)ptr, byte_len) == (size_t)byte_len &&
                      client.print("\r\n");

            if (!ok) {
                Serial.println("net: server disconnected — reconnecting");
                break;
            }
        }

        client.print("0\r\n\r\n");
        client.stop();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  Arduino entry points
 * ═══════════════════════════════════════════════════════════════════ */
void setup()
{
    Serial.begin(115200);
    Serial.println("=== INMP441 WiFi Audio Pusher ===");

    audio_queue = xQueueCreate(QUEUE_DEPTH, sizeof(int16_t *));

    /* ── WiFi ──────────────────────────────────────────────── */
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("Connecting to '%s'...", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

    /* ── I2S ───────────────────────────────────────────────── */
    i2s_init();

    /* ── Tasks ─────────────────────────────────────────────── */
    /*  mic_capture_task: Core 1, priority 5 — real-time audio  */
    xTaskCreatePinnedToCore(mic_capture_task, "mic_cap",  4096, NULL, 5, NULL, 1);
    /*  net_send_task:    Core 0, priority 4 — WiFi lives here  */
    xTaskCreatePinnedToCore(net_send_task,    "net_send", 8192, NULL, 4, NULL, 0);
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
