#include "ruin_air_bridge.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define RUIN_CHANNEL 6
#define RUIN_EVENT_QUEUE_LENGTH 16
#define RUIN_MAX_PAYLOAD 32
#define RUIN_TELEMETRY_TIMEOUT_US 1000000LL
#define RUIN_RELEASE_DELAY_US 250000
#define RUIN_RELEASE_RETRY_INTERVAL_US 50000
#define RUIN_RELEASE_ATTEMPTS 4
#define RUIN_AIR_OUT_DELAY_US 10000000

static const char *TAG = "ruin_air";
static const uint8_t BROADCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static const uint8_t RUIN_REMOTE_STA_MAC[6] = {0xf4, 0x65, 0x0b, 0xa9, 0xdd, 0xc4};
static const uint8_t RUIN_REMOTE_AP_MAC[6] = {0xf4, 0x65, 0x0b, 0xa9, 0xdd, 0xc5};

typedef struct {
    uint8_t source[6];
    uint8_t length;
    uint8_t data[RUIN_MAX_PAYLOAD];
} ruin_event_t;

static QueueHandle_t s_event_queue;
static esp_timer_handle_t s_release_timer;
static ruin_air_state_callback_t s_state_callback;
static void *s_state_context;
static ruin_air_tx_callback_t s_tx_callback;
static void *s_tx_context;
static ruin_air_state_t s_state;
static uint8_t s_controller_mac[6];
static uint16_t s_tx_sequence_control;
static int64_t s_last_telemetry_us;
static uint8_t s_release_attempts_remaining;
static bool s_initialized;
static bool s_controller_peer_added;

static uint16_t get_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void put_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static esp_err_t send_v1_frame(const uint8_t *destination, const uint8_t *payload,
                               size_t payload_length)
{
    if (!destination || !payload || payload_length > 250) return ESP_ERR_INVALID_ARG;
    uint8_t frame[39 + RUIN_MAX_PAYLOAD] = {0};
    size_t frame_length = 39 + payload_length;
    frame[0] = 0xd0;
    frame[2] = 0x3a;
    frame[3] = 0x01;
    memcpy(&frame[4], destination, 6);
    memcpy(&frame[10], RUIN_REMOTE_STA_MAC, 6);
    memcpy(&frame[16], BROADCAST_MAC, 6);
    frame[22] = (uint8_t)s_tx_sequence_control;
    frame[23] = (uint8_t)(s_tx_sequence_control >> 8);
    frame[24] = 0x7f;
    frame[25] = 0x18;
    frame[26] = 0xfe;
    frame[27] = 0x34;
    esp_fill_random(&frame[28], 4);
    frame[32] = 0xdd;
    frame[33] = (uint8_t)(payload_length + 5);
    frame[34] = 0x18;
    frame[35] = 0xfe;
    frame[36] = 0x34;
    frame[37] = 0x04;
    frame[38] = 0x01;
    memcpy(&frame[39], payload, payload_length);
    esp_err_t error = esp_wifi_80211_tx(WIFI_IF_STA, frame, (int)frame_length, false);
    if (error == ESP_OK) {
        s_tx_sequence_control += 0x10;
        if (s_tx_callback) s_tx_callback(frame, frame_length, s_tx_context);
    }
    return error;
}

static void publish_state(void)
{
    if (s_state_callback) s_state_callback(&s_state, s_state_context);
}

void ruin_air_bridge_set_tx_callback(ruin_air_tx_callback_t callback, void *context)
{
    s_tx_callback = callback;
    s_tx_context = context;
}

static bool learn_controller(const uint8_t *mac)
{
    if (!s_controller_peer_added || memcmp(s_controller_mac, mac, 6) != 0) {
        memcpy(s_controller_mac, mac, sizeof(s_controller_mac));
        s_controller_peer_added = true;
        ESP_LOGI(TAG, "Ruin controller peer %02X:%02X:%02X:%02X:%02X:%02X",
                 s_controller_mac[0], s_controller_mac[1], s_controller_mac[2],
                 s_controller_mac[3], s_controller_mac[4], s_controller_mac[5]);
    }
    return true;
}

static esp_err_t send_release(void)
{
    if (!s_controller_peer_added) return ESP_ERR_INVALID_STATE;
    const uint8_t payload[3] = {0x03, 0x00, 0x00};
    return send_v1_frame(s_controller_mac, payload, sizeof(payload));
}

static esp_err_t schedule_release(int64_t delay_us)
{
    esp_err_t error = esp_timer_stop(s_release_timer);
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) return error;
    s_release_attempts_remaining = RUIN_RELEASE_ATTEMPTS;
    error = esp_timer_start_once(s_release_timer, delay_us);
    if (error != ESP_OK) {
        s_release_attempts_remaining = 0;
        send_release();
    }
    return error;
}

static void release_timer_callback(void *argument)
{
    (void)argument;
    esp_err_t error = send_release();
    if (error != ESP_OK) ESP_LOGE(TAG, "Release send failed: %s", esp_err_to_name(error));
    if (s_release_attempts_remaining > 0) --s_release_attempts_remaining;
    if (s_release_attempts_remaining > 0) {
        error = esp_timer_start_once(s_release_timer, RUIN_RELEASE_RETRY_INTERVAL_US);
        if (error != ESP_OK) {
            s_release_attempts_remaining = 0;
            ESP_LOGE(TAG, "Release retry schedule failed: %s", esp_err_to_name(error));
        }
    }
}

void ruin_air_bridge_ingest_frame(const uint8_t *frame, size_t length)
{
    if (!s_event_queue || !frame || length < 39 || frame[0] != 0xd0 ||
        frame[24] != 0x7f || memcmp(&frame[25], "\x18\xfe\x34", 3) != 0 ||
        frame[32] != 0xdd || memcmp(&frame[34], "\x18\xfe\x34\x04", 4) != 0 ||
        (frame[38] & 0x0f) != 1) return;
    size_t payload_length = frame[33] >= 5 ? frame[33] - 5 : 0;
    if (payload_length == 0 || payload_length > RUIN_MAX_PAYLOAD ||
        39 + payload_length > length) return;
    ruin_event_t event = {.length = (uint8_t)payload_length};
    memcpy(event.source, &frame[10], sizeof(event.source));
    memcpy(event.data, &frame[39], payload_length);
    xQueueSend(s_event_queue, &event, 0);
}

static void handle_discovery(const ruin_event_t *event)
{
    if (event->length != 16 || event->data[0] != 0x00 || event->data[1] != 0x01 ||
        event->data[2] != RUIN_CHANNEL) return;

    if (!learn_controller(event->source)) return;

    uint8_t response[16];
    memcpy(response, event->data, sizeof(response));
    response[1] = 0x00;
    esp_wifi_get_mac(WIFI_IF_AP, &response[3]);
    esp_err_t error = send_v1_frame(s_controller_mac, response, sizeof(response));
    if (error == ESP_OK) {
        ESP_LOGI(TAG, "Paired with Ruin controller %02X:%02X:%02X:%02X:%02X:%02X",
                 s_controller_mac[0], s_controller_mac[1], s_controller_mac[2],
                 s_controller_mac[3], s_controller_mac[4], s_controller_mac[5]);
    } else {
        ESP_LOGE(TAG, "Pair response failed: %s", esp_err_to_name(error));
    }
}

static void handle_telemetry(const ruin_event_t *event)
{
    if (event->length != 12 || event->data[0] != 0x02 || event->data[1] != 0x01) return;
    if (!learn_controller(event->source)) return;
    s_state.connected = true;
    s_state.pressure_valid = true;
    s_state.tank_psi = get_u16(&event->data[2]);
    s_state.front_left_psi = get_u16(&event->data[4]);
    s_state.front_right_psi = get_u16(&event->data[6]);
    s_state.rear_left_psi = get_u16(&event->data[8]);
    s_state.rear_right_psi = get_u16(&event->data[10]);
    s_last_telemetry_us = esp_timer_get_time();
    publish_state();
}

static void bridge_task(void *argument)
{
    (void)argument;
    ruin_event_t event;
    while (true) {
        if (xQueueReceive(s_event_queue, &event, pdMS_TO_TICKS(100)) == pdTRUE) {
            handle_discovery(&event);
            handle_telemetry(&event);
        }
        if (s_state.connected &&
            esp_timer_get_time() - s_last_telemetry_us > RUIN_TELEMETRY_TIMEOUT_US) {
            esp_err_t release_error = schedule_release(1);
            if (release_error != ESP_OK) {
                ESP_LOGE(TAG, "Timeout release schedule failed: %s",
                         esp_err_to_name(release_error));
            }
            memset(&s_state, 0, sizeof(s_state));
            publish_state();
            ESP_LOGW(TAG, "Ruin telemetry timed out");
        }
    }
}

esp_err_t ruin_air_bridge_init(ruin_air_state_callback_t callback, void *context)
{
    s_state_callback = callback;
    s_state_context = context;
    if (s_initialized) return ESP_OK;

    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
        error = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(error, TAG, "NVS init failed");
    error = esp_netif_init();
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) return error;
    error = esp_event_loop_create_default();
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) return error;

    wifi_init_config_t wifi_configuration = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_configuration), TAG, "Wi-Fi init failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "Wi-Fi storage failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG, "Wi-Fi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mac(WIFI_IF_STA, RUIN_REMOTE_STA_MAC),
                        TAG, "STA MAC takeover failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mac(WIFI_IF_AP, RUIN_REMOTE_AP_MAC),
                        TAG, "AP MAC takeover failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(RUIN_CHANNEL, WIFI_SECOND_CHAN_NONE),
                        TAG, "Wi-Fi channel failed");

    s_event_queue = xQueueCreate(RUIN_EVENT_QUEUE_LENGTH, sizeof(ruin_event_t));
    if (!s_event_queue) return ESP_ERR_NO_MEM;

    const esp_timer_create_args_t timer_configuration = {
        .callback = release_timer_callback,
        .name = "ruin_release",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_configuration, &s_release_timer),
                        TAG, "Release timer failed");
    if (xTaskCreate(bridge_task, "ruin_air", 4096, NULL, 7, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_initialized = true;
    ESP_LOGI(TAG, "Ruin ESP-NOW bridge listening on channel %d", RUIN_CHANNEL);
    return ESP_OK;
}

esp_err_t ruin_air_bridge_request(uint8_t command)
{
    if (!s_initialized || !s_controller_peer_added || !s_state.connected) {
        return ESP_ERR_INVALID_STATE;
    }

    if (command == RUIN_AIR_RELEASE_COMMAND) {
        return schedule_release(1);
    }

    static const uint8_t manual_codes[] = {
        0x09, 0x0a, 0x07, 0x08, 0x0d, 0x0e, 0x0b, 0x0c,
        0x01, 0x02,
    };
    if (command < sizeof(manual_codes)) {
        uint8_t payload[3] = {0x03, 0x00, manual_codes[command]};
        esp_timer_stop(s_release_timer);
        ESP_RETURN_ON_ERROR(send_v1_frame(s_controller_mac, payload, sizeof(payload)),
                            TAG, "Manual command failed");
        return schedule_release(RUIN_RELEASE_DELAY_US);
    }

    if (command >= 10 && command <= 13) {
        static const uint16_t preset_targets[] = {85, 95, 40, 1};
        uint16_t target = preset_targets[command - 10];
        uint8_t payload[6] = {0x04, 0x00};
        put_u16(&payload[2], target);
        put_u16(&payload[4], target);
        return send_v1_frame(s_controller_mac, payload, sizeof(payload));
    }

    static const uint8_t axle_codes[] = {0x03, 0x04, 0x05, 0x06};
    if (command >= 16 && command <= 19) {
        uint8_t payload[3] = {0x03, 0x00, axle_codes[command - 16]};
        esp_timer_stop(s_release_timer);
        ESP_RETURN_ON_ERROR(send_v1_frame(s_controller_mac, payload, sizeof(payload)),
                            TAG, "Axle command failed");
        return schedule_release(RUIN_RELEASE_DELAY_US);
    }

    if (command == 20 || command == 21) {
        return ruin_air_bridge_set_compressor(command == 20, 120, 150);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ruin_air_bridge_air_out(void)
{
    if (!s_initialized || !s_controller_peer_added || !s_state.connected) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t payload[3] = {0x03, 0x00, 0x02};
    esp_timer_stop(s_release_timer);
    ESP_RETURN_ON_ERROR(send_v1_frame(s_controller_mac, payload, sizeof(payload)),
                        TAG, "Air out command failed");
    return schedule_release(RUIN_AIR_OUT_DELAY_US);
}

esp_err_t ruin_air_bridge_set_compressor(bool enabled, uint16_t on_psi, uint16_t off_psi)
{
    if (!s_initialized || !s_controller_peer_added || !s_state.connected) {
        return ESP_ERR_INVALID_STATE;
    }
    if (on_psi < 80 || off_psi > 200 || on_psi + 5 > off_psi) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t payload[8] = {0x07, 0x00, enabled ? 1 : 0, 0x00};
    put_u16(&payload[4], on_psi);
    put_u16(&payload[6], off_psi);
    esp_err_t error = send_v1_frame(s_controller_mac, payload, sizeof(payload));
    if (error == ESP_OK) {
        s_state.compressor_valid = true;
        s_state.compressor_on = enabled;
        publish_state();
    }
    return error;
}