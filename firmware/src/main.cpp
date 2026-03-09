/**
 * INMP441 I2S Microphone → WiFi HTTP Audio Stream
 * Board: NodeMCU ESP-32S
 * Framework: ESP-IDF (PlatformIO)
 *
 * How it works:
 *   1. ESP32 connects to your WiFi network
 *   2. Starts an HTTP server on port 8000
 *   3. GET http://<ESP32_IP>:8000/audio  → streams raw 16-bit signed PCM
 *
 * Receive on your computer:
 *   curl http://<ESP32_IP>:8000/audio > recording.raw
 *   ffmpeg -f s16le -ar 16000 -ac 1 -i recording.raw recording.wav
 *
 * Wiring:
 *   INMP441 VDD  → 3.3V
 *   INMP441 GND  → GND
 *   INMP441 SD   → GPIO32
 *   INMP441 SCK  → GPIO14
 *   INMP441 WS   → GPIO15
 *   INMP441 L/R  → GND
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
#include "esp_http_server.h"

/* ═══════════════════════════════════════════════════════════════════
 *  CONFIGURATION — EDIT THESE
 * ═══════════════════════════════════════════════════════════════════ */
#define WIFI_SSID      "UCScamDeigo"
#define WIFI_PASS      "mainwater348"
#define HTTP_PORT      8000

/* ── I2S / Mic Pins ─────────────────────────────────────────────── */
#define I2S_SD_PIN     32
#define I2S_SCK_PIN    14
#define I2S_WS_PIN     15
#define I2S_PORT       I2S_NUM_0
#define SAMPLE_RATE    16000
#define DMA_BUF_COUNT  8
#define DMA_BUF_LEN    1024
#define READ_BUF_SAMPLES 512

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
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "  Endpoint: http://" IPSTR ":%d/audio",
                 IP2STR(&event->ip_info.ip), HTTP_PORT);
        ESP_LOGI(TAG, "========================================");
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
    i2s_config_t i2s_config = {};
    i2s_config.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    i2s_config.sample_rate          = SAMPLE_RATE;
    i2s_config.bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT;
    i2s_config.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_config.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    i2s_config.dma_buf_count        = DMA_BUF_COUNT;
    i2s_config.dma_buf_len          = DMA_BUF_LEN;
    i2s_config.use_apll             = false;
    i2s_config.tx_desc_auto_clear   = false;
    i2s_config.fixed_mclk           = 0;

    i2s_pin_config_t pin_config = {};
    pin_config.bck_io_num   = I2S_SCK_PIN;
    pin_config.ws_io_num    = I2S_WS_PIN;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num  = I2S_SD_PIN;

    ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT, &pin_config));
    ESP_ERROR_CHECK(i2s_start(I2S_PORT));

    ESP_LOGI(TAG, "I2S mic initialized (SR=%d Hz)", SAMPLE_RATE);
}

/* ═══════════════════════════════════════════════════════════════════
 *  HTTP handler: GET /audio  — streams raw 16-bit PCM indefinitely
 * ═══════════════════════════════════════════════════════════════════ */
static esp_err_t audio_get_handler(httpd_req_t *req)
{
    int32_t raw[READ_BUF_SAMPLES];
    int16_t pcm[READ_BUF_SAMPLES];

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "X-Sample-Rate",   "16000");
    httpd_resp_set_hdr(req, "X-Bits-Per-Sample", "16");
    httpd_resp_set_hdr(req, "X-Channels",      "1");

    ESP_LOGI(TAG, "== Client connected to /audio — streaming... ==");

    while (1) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_read(I2S_PORT, raw, sizeof(raw),
                                 &bytes_read, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_read error: %s", esp_err_to_name(err));
            break;
        }

        int count = bytes_read / sizeof(int32_t);
        for (int i = 0; i < count; i++) {
            pcm[i] = (int16_t)(raw[i] >> 16);
        }

        if (httpd_resp_send_chunk(req, (const char *)pcm,
                                  count * sizeof(int16_t)) != ESP_OK) {
            ESP_LOGI(TAG, "Client disconnected from /audio");
            break;
        }
    }

    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static void http_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = HTTP_PORT;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t audio_uri = {};
    audio_uri.uri     = "/audio";
    audio_uri.method  = HTTP_GET;
    audio_uri.handler = audio_get_handler;

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &audio_uri));
    ESP_LOGI(TAG, "HTTP server started on port %d, endpoint: /audio", HTTP_PORT);
}

/* ═══════════════════════════════════════════════════════════════════
 *  Main
 * ═══════════════════════════════════════════════════════════════════ */
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== INMP441 WiFi Audio Streamer ===");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();
    i2s_init();
    http_server_start();
}
