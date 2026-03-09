/**
 * INMP441 I2S Microphone → WiFi HTTP POST Audio Stream
 * Board: NodeMCU ESP-32S
 * Framework: ESP-IDF (PlatformIO)
 *
 * POSTs chunked raw 16-bit signed PCM to SERVER_HOST:SERVER_PORT/audio.
 * Flushes when the audio buffer is full OR every SEND_INTERVAL_MS.
 *
 * Wiring:
 *   INMP441 VDD  → 3.3V   INMP441 GND  → GND
 *   INMP441 SD   → GPIO32  INMP441 SCK  → GPIO14
 *   INMP441 WS   → GPIO15  INMP441 L/R  → GND
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* ═══════════════════════════════════════════════════════════════════
 *  CONFIGURATION — EDIT THESE
 * ═══════════════════════════════════════════════════════════════════ */
#define WIFI_SSID          "UCScamDeigo"
#define WIFI_PASS          "mainwater348"

#define SERVER_HOST        "192.168.0.125"
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

static const char *TAG = "MIC_STREAM";

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0

/* ═══════════════════════════════════════════════════════════════════
 *  WiFi
 * ═══════════════════════════════════════════════════════════════════ */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected — reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid,     WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password,  WIFI_PASS, sizeof(wifi_config.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi '%s'...", WIFI_SSID);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ═══════════════════════════════════════════════════════════════════
 *  I2S Microphone
 * ═══════════════════════════════════════════════════════════════════ */
static void i2s_init(void)
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

    ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pins));
    ESP_ERROR_CHECK(i2s_start(I2S_PORT));
    ESP_LOGI(TAG, "I2S mic initialized (SR=%d Hz)", SAMPLE_RATE);
}

/* ── helpers ───────────────────────────────────────────────────── */
static bool sock_send_all(int sock, const void *data, int len)
{
    const char *p = (const char *)data;
    while (len > 0) {
        int n = send(sock, p, len, 0);
        if (n <= 0) return false;
        p += n;
        len -= n;
    }
    return true;
}

/* Send one HTTP chunked-encoding chunk */
static bool send_chunk(int sock, const void *data, int byte_len)
{
    char hdr[16];
    int hdr_len = snprintf(hdr, sizeof(hdr), "%x\r\n", byte_len);
    return sock_send_all(sock, hdr, hdr_len) &&
           sock_send_all(sock, data, byte_len) &&
           sock_send_all(sock, "\r\n", 2);
}

/* ═══════════════════════════════════════════════════════════════════
 *  Audio push task
 *  - opens ONE TCP connection
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
        /* ── Connect ───────────────────────────────────────── */
        struct sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(SERVER_PORT);
        inet_pton(AF_INET, SERVER_HOST, &addr.sin_addr);

        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock < 0) {
            ESP_LOGE(TAG, "socket() failed");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        /* 10-second send/recv timeouts */
        struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        /* Disable Nagle — we control chunking ourselves */
        int nodelay = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            ESP_LOGE(TAG, "connect() to %s:%d failed", SERVER_HOST, SERVER_PORT);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        /* ── Send HTTP headers ─────────────────────────────── */
        char headers[512];
        int hlen = snprintf(headers, sizeof(headers),
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

        if (!sock_send_all(sock, headers, hlen)) {
            ESP_LOGE(TAG, "Failed to send headers");
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        ESP_LOGI(TAG, "Connected — streaming to http://%s:%d%s",
                 SERVER_HOST, SERVER_PORT, SERVER_PATH);

        buf_count = 0;
        TickType_t last_flush = xTaskGetTickCount();
        bool error = false;

        /* ── Streaming loop ────────────────────────────────── */
        while (!error) {
            size_t bytes_read = 0;
            esp_err_t err = i2s_read(I2S_PORT, raw, sizeof(raw),
                                     &bytes_read, pdMS_TO_TICKS(50));
            if (err != ESP_OK) continue;   /* timeout is fine, check interval */

            int count = (int)(bytes_read / sizeof(int32_t));

            /* DEBUG: log stats every ~1 second to monitor mic health */
            static int debug_tick = 0;
            static int dropout_count = 0;
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
                    ESP_LOGW(TAG, "ALL ZEROS — mic dropout #%d (check wiring!)", dropout_count);
                } else {
                    ESP_LOGI(TAG, "raw range: [%ld .. %ld]  zeros=%d/%d  dropouts=%d",
                             (long)mn, (long)mx, zeros, count, dropout_count);
                }
                debug_tick = 0;
            }

            for (int i = 0; i < count && buf_count < SEND_BUF_SAMPLES; i++) {
                /* INMP441: 24-bit audio left-justified in 32-bit frame.
                 * Bits 31..8 contain audio. >> 16 takes the top 16 bits cleanly.
                 * (>> 8 was WRONG — crammed 24 bits into int16 → overflow/zeros) */
                send_buf[buf_count++] = (int16_t)(raw[i] >> 16);
            }

            TickType_t now      = xTaskGetTickCount();
            bool buf_full       = buf_count >= SEND_BUF_SAMPLES;
            bool interval_hit   = (now - last_flush) >= pdMS_TO_TICKS(SEND_INTERVAL_MS);

            if ((buf_full || interval_hit) && buf_count > 0) {
                int byte_len = buf_count * (int)sizeof(int16_t);
                if (!send_chunk(sock, send_buf, byte_len)) {
                    ESP_LOGW(TAG, "Server disconnected — reconnecting...");
                    error = true;
                } else {
                    ESP_LOGD(TAG, "Flushed %d samples (%s)",
                             buf_count, buf_full ? "buf full" : "interval");
                    buf_count  = 0;
                    last_flush = xTaskGetTickCount();
                }
            }
        }

        /* Terminate chunked stream cleanly */
        sock_send_all(sock, "0\r\n\r\n", 5);
        close(sock);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ═══════════════════════════════════════════════════════════════════
 *  Main
 * ═══════════════════════════════════════════════════════════════════ */
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== INMP441 WiFi Audio Pusher ===");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();
    i2s_init();

    xTaskCreatePinnedToCore(audio_push_task, "audio_push", 8192, NULL, 5, NULL, 1);
}