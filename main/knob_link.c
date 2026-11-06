#include "knob_link.h"

#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "nvs_flash.h"

/* must match the dash (main/wifi_update.h) */
#define DASH_SSID_PREFIX "MackoDash-"
#define DASH_PASSWORD    "mackodash-knob-link"
#define DASH_UDP_PORT    4210

#define BIT_GOT_IP       BIT0
#define BIT_DISCONNECTED BIT1

static const char *TAG = "knob_link";
static EventGroupHandle_t s_events;
static knob_link_frame_cb_t s_on_frame;
static volatile bool s_up;
static volatile int s_sock = -1;
static uint32_t s_gateway;
static uint8_t s_last_bssid[6];
static bool s_have_last_bssid;

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_up = false;
        xEventGroupSetBits(s_events, BIT_DISCONNECTED);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_gateway = ((ip_event_got_ip_t *)data)->ip_info.gw.addr;
        xEventGroupSetBits(s_events, BIT_GOT_IP);
    }
}

static void load_last_dash(void)
{
    nvs_handle_t nvs;
    if (nvs_open("knob_link", NVS_READONLY, &nvs) != ESP_OK) return;
    size_t len = sizeof(s_last_bssid);
    s_have_last_bssid = nvs_get_blob(nvs, "bssid", s_last_bssid, &len) == ESP_OK && len == 6;
    nvs_close(nvs);
}

static void save_last_dash(const uint8_t bssid[6])
{
    if (s_have_last_bssid && memcmp(bssid, s_last_bssid, 6) == 0) return;
    memcpy(s_last_bssid, bssid, 6);
    s_have_last_bssid = true;
    nvs_handle_t nvs;
    if (nvs_open("knob_link", NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_blob(nvs, "bssid", bssid, 6);
    nvs_commit(nvs);
    nvs_close(nvs);
}

/* Strongest MackoDash network, but the one used last wins whenever it is heard. */
static bool find_dash(wifi_ap_record_t *best)
{
    wifi_scan_config_t scan = {.show_hidden = false};
    if (esp_wifi_scan_start(&scan, true) != ESP_OK) return false;
    uint16_t count = 0;
    esp_wifi_scan_get_ap_num(&count);
    if (count == 0) return false;
    if (count > 24) count = 24;
    wifi_ap_record_t *recs = calloc(count, sizeof(*recs));
    if (!recs) {
        esp_wifi_clear_ap_list();
        return false;
    }
    esp_wifi_scan_get_ap_records(&count, recs);
    int pick = -1;
    for (int i = 0; i < count; ++i) {
        if (strncmp((const char *)recs[i].ssid, DASH_SSID_PREFIX, strlen(DASH_SSID_PREFIX)) != 0) continue;
        if (s_have_last_bssid && memcmp(recs[i].bssid, s_last_bssid, 6) == 0) {
            pick = i;
            break;
        }
        if (pick < 0 || recs[i].rssi > recs[pick].rssi) pick = i;
    }
    if (pick >= 0) *best = recs[pick];
    free(recs);
    return pick >= 0;
}

static void open_socket(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) return;
    struct sockaddr_in dash = {
        .sin_family = AF_INET,
        .sin_port = htons(DASH_UDP_PORT),
        .sin_addr.s_addr = s_gateway,      /* the dash is the network's gateway */
    };
    if (connect(sock, (struct sockaddr *)&dash, sizeof(dash)) != 0) {
        close(sock);
        return;
    }
    struct timeval timeout = {.tv_sec = 0, .tv_usec = 200000};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    s_sock = sock;
}

static void close_socket(void)
{
    int sock = s_sock;
    s_sock = -1;
    if (sock >= 0) close(sock);
}

static void link_task(void *arg)
{
    (void)arg;
    for (;;) {
        wifi_ap_record_t dash;
        if (!find_dash(&dash)) {
            vTaskDelay(pdMS_TO_TICKS(1500));
            continue;
        }
        wifi_config_t wc = {0};
        memcpy(wc.sta.ssid, dash.ssid, sizeof(wc.sta.ssid));
        strcpy((char *)wc.sta.password, DASH_PASSWORD);
        memcpy(wc.sta.bssid, dash.bssid, 6);
        wc.sta.bssid_set = true;
        wc.sta.channel = dash.primary;
        wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        wc.sta.pmf_cfg.capable = true;
        esp_wifi_set_config(WIFI_IF_STA, &wc);
        xEventGroupClearBits(s_events, BIT_GOT_IP | BIT_DISCONNECTED);
        ESP_LOGI(TAG, "joining %s (%d dBm)", (char *)dash.ssid, dash.rssi);
        esp_wifi_connect();
        EventBits_t bits = xEventGroupWaitBits(s_events, BIT_GOT_IP | BIT_DISCONNECTED, pdTRUE, pdFALSE,
                                               pdMS_TO_TICKS(15000));
        if (!(bits & BIT_GOT_IP)) {
            esp_wifi_disconnect();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        save_last_dash(dash.bssid);
        open_socket();
        s_up = s_sock >= 0;
        ESP_LOGI(TAG, "linked to %s", (char *)dash.ssid);

        xEventGroupWaitBits(s_events, BIT_DISCONNECTED, pdTRUE, pdFALSE, portMAX_DELAY);
        s_up = false;
        close_socket();
        ESP_LOGI(TAG, "link lost, searching");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[128];
    for (;;) {
        int sock = s_sock;
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        int n = recv(sock, buf, sizeof(buf), 0);
        if (n > 0 && s_on_frame) s_on_frame(buf, (size_t)n);
    }
}

void knob_link_start(knob_link_frame_cb_t on_frame)
{
    s_on_frame = on_frame;
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) ESP_LOGW(TAG, "nvs: %s", esp_err_to_name(err));
    load_last_dash();

    s_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, on_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);   /* no power-save delay on every knob turn */

    xTaskCreate(link_task, "knob_link", 4096, NULL, 4, NULL);
    xTaskCreate(rx_task, "knob_rx", 3072, NULL, 6, NULL);
}

bool knob_link_send(const uint8_t *frame, size_t length)
{
    int sock = s_sock;
    if (!s_up || sock < 0) return false;
    return send(sock, frame, length, 0) == (int)length;
}

bool knob_link_up(void)
{
    return s_up;
}
