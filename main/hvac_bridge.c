#include "hvac_bridge.h"

#include <stddef.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define HVAC_MAGIC UINT32_C(0x43415648)
#define HVAC_VERSION 1
#define HVAC_COMMAND 1
#define HVAC_STATE 2
#define HVAC_CHANNEL 6
#define HVAC_TIMEOUT_US 2000000LL

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t sequence;
    uint8_t fan;
    uint8_t mode;
    uint8_t temp;
    uint8_t reserved;
    uint16_t crc;
} command_packet_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t sequence;
    uint8_t fan;
    uint8_t mode;
    uint8_t temp;
    uint8_t target_fan;
    uint8_t target_mode;
    uint8_t target_temp;
    uint8_t moving_mask;
    uint8_t fault_flags;
    uint16_t crc;
} state_packet_t;

static const char *TAG = "hvac_bridge";
static const uint8_t NODE_MAC[ESP_NOW_ETH_ALEN] = {0xb4, 0xbf, 0xe9, 0x6a, 0x23, 0x14};
static const uint8_t PMK[ESP_NOW_KEY_LEN] = {
    0x4d, 0x61, 0x63, 0x6b, 0x6f, 0x44, 0x61, 0x73,
    0x68, 0x48, 0x56, 0x41, 0x43, 0x50, 0x4d, 0x4b,
};
static const uint8_t LMK[ESP_NOW_KEY_LEN] = {
    0x43, 0x69, 0x76, 0x69, 0x63, 0x48, 0x56, 0x41,
    0x43, 0x4c, 0x69, 0x6e, 0x6b, 0x4c, 0x4d, 0x4b,
};

static QueueHandle_t s_queue;
static hvac_bridge_state_callback_t s_callback;
static void *s_context;
static hvac_bridge_state_t s_state;
static uint16_t s_sequence;
static int64_t s_last_state_us;

static uint16_t crc16(const void *buffer, size_t length)
{
    const uint8_t *data = buffer;
    uint16_t crc = 0xffff;
    for (size_t index = 0; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) :
                                   (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static bool valid_state(const state_packet_t *packet)
{
    return packet->magic == HVAC_MAGIC && packet->version == HVAC_VERSION &&
           packet->type == HVAC_STATE && packet->fan <= 10 && packet->mode <= 4 &&
           packet->temp <= 14 && packet->target_fan <= 10 &&
           packet->target_mode <= 4 && packet->target_temp <= 14 &&
           packet->crc == crc16(packet, offsetof(state_packet_t, crc));
}

static void receive_callback(const esp_now_recv_info_t *info,
                             const uint8_t *data, int length)
{
    if (!info || !data || length != sizeof(state_packet_t) ||
        memcmp(info->src_addr, NODE_MAC, ESP_NOW_ETH_ALEN) != 0) return;
    state_packet_t packet;
    memcpy(&packet, data, sizeof(packet));
    if (valid_state(&packet)) xQueueOverwrite(s_queue, &packet);
}

static void state_task(void *argument)
{
    (void)argument;
    state_packet_t packet;
    while (true) {
        if (xQueueReceive(s_queue, &packet, pdMS_TO_TICKS(100)) == pdTRUE) {
            s_state = (hvac_bridge_state_t) {
                .connected = true,
                .fan = packet.fan,
                .mode = packet.mode,
                .temp = packet.temp,
                .target_fan = packet.target_fan,
                .target_mode = packet.target_mode,
                .target_temp = packet.target_temp,
                .moving_mask = packet.moving_mask,
                .fault_flags = packet.fault_flags,
            };
            s_last_state_us = esp_timer_get_time();
            if (s_callback) s_callback(&s_state, s_context);
        }
        if (s_state.connected &&
            esp_timer_get_time() - s_last_state_us > HVAC_TIMEOUT_US) {
            s_state.connected = false;
            if (s_callback) s_callback(&s_state, s_context);
            ESP_LOGW(TAG, "HVAC node timed out");
        }
    }
}

esp_err_t hvac_bridge_set(uint8_t fan, uint8_t mode, uint8_t temp)
{
    if (!s_queue) return ESP_ERR_INVALID_STATE;
    if (fan > 10 || mode > 4 || temp > 14) return ESP_ERR_INVALID_ARG;
    command_packet_t packet = {
        .magic = HVAC_MAGIC,
        .version = HVAC_VERSION,
        .type = HVAC_COMMAND,
        .sequence = ++s_sequence,
        .fan = fan,
        .mode = mode,
        .temp = temp,
    };
    packet.crc = crc16(&packet, offsetof(command_packet_t, crc));
    return esp_now_send(NODE_MAC, (const uint8_t *)&packet, sizeof(packet));
}

esp_err_t hvac_bridge_init(hvac_bridge_state_callback_t callback, void *context)
{
    s_callback = callback;
    s_context = context;
    s_queue = xQueueCreate(1, sizeof(state_packet_t));
    if (!s_queue) return ESP_ERR_NO_MEM;
    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "ESP-NOW init failed");
    ESP_RETURN_ON_ERROR(esp_now_set_pmk(PMK), TAG, "PMK failed");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(receive_callback), TAG, "RX callback failed");

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, NODE_MAC, ESP_NOW_ETH_ALEN);
    memcpy(peer.lmk, LMK, ESP_NOW_KEY_LEN);
    peer.channel = HVAC_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = true;
    ESP_RETURN_ON_ERROR(esp_now_add_peer(&peer), TAG, "Peer add failed");
    if (xTaskCreate(state_task, "hvac_state", 3072, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Encrypted HVAC peer ready on channel %d", HVAC_CHANNEL);
    return ESP_OK;
}

_Static_assert(sizeof(command_packet_t) == 14, "Unexpected HVAC command size");
_Static_assert(sizeof(state_packet_t) == 18, "Unexpected HVAC state size");