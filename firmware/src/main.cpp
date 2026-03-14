/**
 * INMP441 I2S Microphone → WiFi HTTP POST Audio Stream
 * Board: NodeMCU ESP-32S
 * Framework: Arduino (PlatformIO)
 *
 * POSTs chunked raw 16-bit signed PCM to SERVER_HOST:SERVER_PORT/audio.
 * Flushes when the audio buffer is full OR every SEND_INTERVAL_MS.
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
 *  CONFIGURATION — EDIT THESE
 * ═══════════════════════════════════════════════════════════════════ */
#define WIFI_SSID          "UCScamDeigo"
#define WIFI_PASS          "mainwater348"

#define SERVER_HOST        "192.168.0.210"
#define SERVER_PORT        8000
#define SERVER_PATH        "/audio"

/* Flush audio when buffer reaches this many samples OR after this many ms */
#define SEND_INTERVAL_MS   200
/* Buffer holds 2× the interval worth of samples so we never drop */
#define SAMPLE_RATE        16000
#define SEND_BUF_SAMPLES   (SAMPLE_RATE * SEND_INTERVAL_MS / 1000 * 2)  /* 6400 */

/* ── I2S / Mic Pins ─────────────────────────────────────────────── */
#define I2S_SD_PIN         32
#define I2S_SCK_PIN        14
#define I2S_WS_PIN         15
#define I2S_PORT           I2S_NUM_0
#define DMA_BUF_COUNT      8
#define DMA_BUF_LEN        1024
#define READ_BUF_SAMPLES   512

/* ═══════════════════════════════════════════════════════════════════
 *  I2S Microphone
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
    cfg.use_apll             = false;
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
    Serial.printf("I2S mic initialized (SR=%d Hz)\n", SAMPLE_RATE);
}

/* ═══════════════════════════════════════════════════════════════════
 *  Audio push task
 *  - opens ONE TCP connection via WiFiClient
 *  - sends HTTP POST headers with Transfer-Encoding: chunked
 *  - accumulates PCM samples, flushes when buffer full OR timer fires
 *  - reconnects automatically on error
 * ═══════════════════════════════════════════════════════════════════ */
static void audio_push_task(void *arg)
{
    static int32_t raw[READ_BUF_SAMPLES];
    static int16_t send_buf[SEND_BUF_SAMPLES];
    int buf_count = 0;

    while (1) {
        /* ── Wait for WiFi ─────────────────────────────────── */
        while (WiFi.status() != WL_CONNECTED) {
            Serial.println("Waiting for WiFi...");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        /* ── Connect ───────────────────────────────────────── */
        WiFiClient client;
        client.setNoDelay(true);

        if (!client.connect(SERVER_HOST, SERVER_PORT)) {
            Serial.printf("connect() to %s:%d failed\n", SERVER_HOST, SERVER_PORT);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        /* ── Send HTTP headers ─────────────────────────────── */
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

        Serial.printf("Connected — streaming to http://%s:%d%s\n",
                      SERVER_HOST, SERVER_PORT, SERVER_PATH);

        buf_count = 0;
        TickType_t last_flush = xTaskGetTickCount();
        bool error = false;

        /* ── Streaming loop ────────────────────────────────── */
        static int debug_tick = 0;
        static int dropout_count = 0;

        while (!error && client.connected()) {
            size_t bytes_read = 0;
            esp_err_t err = i2s_read(I2S_PORT, raw, sizeof(raw),
                                     &bytes_read, pdMS_TO_TICKS(50));
            if (err != ESP_OK) continue;   /* timeout is fine, check interval */

            int count = (int)(bytes_read / sizeof(int32_t));

            /* DEBUG: log stats every ~1 second to monitor mic health */
            if (++debug_tick >= (SAMPLE_RATE / READ_BUF_SAMPLES)) {
                int32_t mn = raw[0], mx = raw[0];
                int zeros = 0;
                for (int i = 0; i < count; i++) {
                    if (raw[i] < mn) mn = raw[i];
                    if (raw[i] > mx) mx = raw[i];
                    if (raw[i] == 0) zeros++;
                }
                if (mn == 0 && mx == 0) {
                    dropout_count++;
                    Serial.printf("ALL ZEROS — mic dropout #%d (check wiring!)\n", dropout_count);
                }
                Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
                debug_tick = 0;
            }

            for (int i = 0; i < count && buf_count < SEND_BUF_SAMPLES; i++) {
                /* INMP441: 24-bit audio left-justified in 32-bit frame.
                 * >> 16 takes the top 16 bits cleanly. */
                send_buf[buf_count++] = (int16_t)(raw[i] >> 16);
            }

            TickType_t now    = xTaskGetTickCount();
            bool buf_full     = buf_count >= SEND_BUF_SAMPLES;
            bool interval_hit = (now - last_flush) >= pdMS_TO_TICKS(SEND_INTERVAL_MS);

            if ((buf_full || interval_hit) && buf_count > 0) {
                int byte_len = buf_count * (int)sizeof(int16_t);

                /* Send one HTTP chunked-encoding chunk */
                char hdr[16];
                snprintf(hdr, sizeof(hdr), "%x\r\n", byte_len);
                bool ok = client.print(hdr) &&
                          client.write((const uint8_t *)send_buf, byte_len) == (size_t)byte_len &&
                          client.print("\r\n");

                if (!ok) {
                    Serial.println("Server disconnected — reconnecting...");
                    error = true;
                } else {
                    buf_count  = 0;
                    last_flush = xTaskGetTickCount();
                }
            }
        }

        /* Terminate chunked stream cleanly */
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
    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("Connecting to WiFi '%s'...", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

    i2s_init();

    xTaskCreatePinnedToCore(audio_push_task, "audio_push", 8192, NULL, 5, NULL, 1);
}

void loop()
{
    /* Everything runs in audio_push_task */
    vTaskDelay(pdMS_TO_TICKS(1000));
}
