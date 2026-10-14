#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "honda_emblem.h"
#include "hvac_bridge.h"
#include "nvs_flash.h"
#include "rotary_protocol.h"
#include "ruin_air_bridge.h"

#define LCD_WIDTH 240
#define LCD_HEIGHT 240
#define LCD_HOST SPI2_HOST
#define LCD_SCLK_GPIO 10
#define LCD_MOSI_GPIO 11
#define LCD_DC_GPIO 3
#define LCD_CS_GPIO 9
#define LCD_RST_GPIO 14
#define LCD_BACKLIGHT_GPIO 46
#define TOUCH_SDA_GPIO 6
#define TOUCH_SCL_GPIO 7
#define TOUCH_INT_GPIO 5
#define TOUCH_RST_GPIO 13
#define ENCODER_A_GPIO 45
#define ENCODER_B_GPIO 42
#define ENCODER_BUTTON_GPIO 41
#define BOARD_POWER_GPIO_1 1
#define BOARD_POWER_GPIO_2 2
#define LINK_UART UART_NUM_0
#define LINK_TX_GPIO 43
#define LINK_RX_GPIO 44
#define LINK_BAUD 115200
#define CAPTURE_CHANNEL 6
#define CAPTURE_QUEUE_LENGTH 8
#define CAPTURE_MAX_FRAME_SIZE 512
#define CST816_ADDRESS 0x15
#define SCREENSAVER_TIMEOUT_US (60LL * 1000 * 1000)

#define RGB565(red, green, blue) \
    ((uint16_t)((((red) & 0xf8) << 8) | (((green) & 0xfc) << 3) | ((blue) >> 3)))

typedef enum {
    PAGE_HOME = 0,
    PAGE_VIEW,
    PAGE_BRIGHTNESS,
    PAGE_RECORD,
    PAGE_TRIP,
    PAGE_PEAKS,
    PAGE_SHIFT,
    PAGE_LOGS,
    PAGE_TRIP_RESET,
    PAGE_ALERT,
    PAGE_AIR_RIDE,
    PAGE_CLIMATE,
} controller_page_t;

typedef enum {
    CLIMATE_FAN = 0,
    CLIMATE_MODE,
    CLIMATE_TEMP,
    CLIMATE_CONTROL_COUNT,
} climate_control_t;

typedef enum {
    CLIMATE_BROWSE = 0,
    CLIMATE_ADJUST,
} climate_controller_mode_t;

typedef enum {
    AIR_CONTROLLER_BROWSE = 0,
    AIR_CONTROLLER_ADJUST,
} air_controller_mode_t;

enum {
    AIR_TARGET_ALL_UP = 0,
    AIR_TARGET_ALL_DOWN,
    AIR_TARGET_FRONT_LEFT,
    AIR_TARGET_FRONT_RIGHT,
    AIR_TARGET_REAR_LEFT,
    AIR_TARGET_REAR_RIGHT,
    AIR_TARGET_COUNT,
};

typedef struct {
    controller_page_t page;
    int16_t selection;
    int16_t count;
    int32_t value;
    bool recording;
    bool air_pressure_valid;
    uint16_t air_pressure_psi[4];
    char label[55];
} controller_state_t;

typedef struct {
    uint8_t data[ROTARY_MAX_FRAME];
    size_t length;
    size_t expected_length;
} frame_parser_t;

typedef struct {
    uint64_t timestamp_us;
    uint16_t captured_length;
    uint16_t original_length;
    int8_t rssi;
    uint8_t channel;
    uint8_t data[CAPTURE_MAX_FRAME_SIZE];
} capture_packet_t;

static const char *TAG = "mackodash_rotary";
static const char *const HOME_LABELS[] = {
    "VIEW", "AIR RIDE", "CLIMATE", "BRIGHT", "REC", "TRIP", "PEAKS", "SHIFT", "LOGS",
};
#define HOME_ITEM_COUNT ((int)(sizeof(HOME_LABELS) / sizeof(HOME_LABELS[0])))
#define CLIMATE_HOME_INDEX 2

static const controller_page_t HOME_PAGE_VALUES[] = {
    PAGE_VIEW, PAGE_AIR_RIDE, PAGE_HOME, PAGE_BRIGHTNESS, PAGE_RECORD,
    PAGE_TRIP, PAGE_PEAKS, PAGE_SHIFT, PAGE_LOGS,
};

static const char *const CONTROLLER_PAGE_LABELS[] = {
    "VIEW", "BRIGHT", "REC", "TRIP", "PEAKS", "SHIFT", "LOGS",
};

static const char *const CLIMATE_LABELS[] = {"FAN", "BLEND", "TEMP"};
static const char *const CLIMATE_FAN_VALUES[] = {"OFF", "1", "2", "3", "4"};
static const char *const CLIMATE_MODE_VALUES[] = {
    "FACE", "BI-LEVEL", "FLOOR", "MIX", "DEFROST",
};
static const char *const CLIMATE_TEMP_VALUES[] = {
    "COLD", "COOL", "NEUTRAL", "WARM", "HOT",
};
static const uint8_t CLIMATE_FAN_NATIVE[] = {0, 2, 4, 7, 10};
static const uint8_t CLIMATE_TEMP_NATIVE[] = {0, 4, 7, 10, 14};
static const int CLIMATE_VALUE_COUNTS[] = {5, 5, 5};
static esp_lcd_panel_io_handle_t s_lcd_io;
static SemaphoreHandle_t s_lcd_done;
static SemaphoreHandle_t s_uart_tx_mutex;
static QueueHandle_t s_encoder_queue;
static QueueHandle_t s_capture_queue;
static uint16_t *s_framebuffer;
static controller_state_t s_state = {.page = PAGE_HOME, .count = HOME_ITEM_COUNT};
static int s_home_selection;
static int s_climate_values[CLIMATE_CONTROL_COUNT] = {0, 0, 2};
static climate_controller_mode_t s_climate_mode;
static hvac_bridge_state_t s_hvac_state;
static air_controller_mode_t s_air_mode;
static int s_air_action;
static int64_t s_air_action_until;
static int64_t s_last_link_rx;
static int64_t s_last_activity;
static uint8_t s_sequence;
static volatile bool s_capture_enabled;
static bool s_wifi_initialized;
static uint32_t s_capture_packets;
static uint32_t s_capture_drops;
static uint16_t s_capture_packet_id;
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;

static const uint8_t RUIN_REMOTE_MAC[6] = {0xf4, 0x65, 0x0b, 0xa9, 0xdd, 0xc5};
static const uint8_t RUIN_CONTROLLER_MAC[6] = {0xec, 0xe3, 0x34, 0x88, 0x96, 0xe0};

static void send_frame(rotary_message_type_t type, const uint8_t *payload, uint16_t length);

static void send_hvac_uart_state(void)
{
    uint8_t payload[9];
    portENTER_CRITICAL(&s_state_lock);
    payload[0] = s_hvac_state.connected ? 1 : 0;
    payload[1] = s_hvac_state.fan;
    payload[2] = s_hvac_state.mode;
    payload[3] = s_hvac_state.temp;
    payload[4] = s_hvac_state.target_fan;
    payload[5] = s_hvac_state.target_mode;
    payload[6] = s_hvac_state.target_temp;
    payload[7] = s_hvac_state.moving_mask;
    payload[8] = s_hvac_state.fault_flags;
    portEXIT_CRITICAL(&s_state_lock);
    send_frame(ROTARY_HVAC_STATE, payload, sizeof(payload));
}

static void hvac_state_changed(const hvac_bridge_state_t *state, void *context)
{
    (void)context;
    portENTER_CRITICAL(&s_state_lock);
    s_hvac_state = *state;
    portEXIT_CRITICAL(&s_state_lock);
    send_hvac_uart_state();
    ESP_LOGI(TAG, "HVAC %s fan=%u mode=%u temp=%u faults=0x%02x",
             state->connected ? "online" : "offline", state->fan,
             state->mode, state->temp, state->fault_flags);
}

static void encoder_edge_isr(void *argument)
{
    (void)argument;
    uint8_t state = (uint8_t)((gpio_get_level(ENCODER_A_GPIO) << 1) |
                              gpio_get_level(ENCODER_B_GPIO));
    BaseType_t higher_priority_task_woken = pdFALSE;
    xQueueSendFromISR(s_encoder_queue, &state, &higher_priority_task_woken);
    if (higher_priority_task_woken) portYIELD_FROM_ISR();
}

static uint16_t swap16(uint16_t value)
{
    return (uint16_t)((value << 8) | (value >> 8));
}

static void pixel(int x, int y, uint16_t color)
{
    if ((unsigned)x < LCD_WIDTH && (unsigned)y < LCD_HEIGHT) {
        s_framebuffer[y * LCD_WIDTH + x] = swap16(color);
    }
}

static void fill(uint16_t color)
{
    uint16_t swapped = swap16(color);
    for (int index = 0; index < LCD_WIDTH * LCD_HEIGHT; ++index) s_framebuffer[index] = swapped;
}

static void fill_rect(int x, int y, int width, int height, uint16_t color)
{
    for (int row = 0; row < height; ++row) {
        for (int column = 0; column < width; ++column) pixel(x + column, y + row, color);
    }
}

static void draw_2bpp_mask(int x, int y, int width, int height,
                           const uint8_t *mask, const uint16_t colors[4])
{
    for (int row = 0; row < height; ++row) {
        for (int column = 0; column < width; ++column) {
            size_t index = (size_t)row * width + column;
            uint8_t alpha = (mask[index >> 2] >> (6 - (index & 3) * 2)) & 0x03;
            if (alpha) pixel(x + column, y + row, colors[alpha]);
        }
    }
}

static void outline_rect(int x, int y, int width, int height, int thickness, uint16_t color)
{
    fill_rect(x, y, width, thickness, color);
    fill_rect(x, y + height - thickness, width, thickness, color);
    fill_rect(x, y + thickness, thickness, height - thickness * 2, color);
    fill_rect(x + width - thickness, y + thickness, thickness, height - thickness * 2, color);
}

static void line(int x0, int y0, int x1, int y1, int thickness, uint16_t color)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (true) {
        fill_rect(x0 - thickness / 2, y0 - thickness / 2, thickness, thickness, color);
        if (x0 == x1 && y0 == y1) break;
        int doubled = 2 * error;
        if (doubled >= dy) { error += dy; x0 += sx; }
        if (doubled <= dx) { error += dx; y0 += sy; }
    }
}

static void ring(int center_x, int center_y, int radius, int thickness, uint16_t color)
{
    int outer = radius * radius;
    int inner_radius = radius - thickness;
    int inner = inner_radius * inner_radius;
    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
            int distance = x * x + y * y;
            if (distance <= outer && distance >= inner) pixel(center_x + x, center_y + y, color);
        }
    }
}

static void arc(int center_x, int center_y, int radius, int start, int end,
                int thickness, uint16_t color)
{
    for (int degree = start; degree <= end; ++degree) {
        float angle = (float)degree * 3.14159265f / 180.0f;
        int x = center_x + (int)lroundf(cosf(angle) * radius);
        int y = center_y + (int)lroundf(sinf(angle) * radius);
        fill_rect(x - thickness / 2, y - thickness / 2, thickness, thickness, color);
    }
}

static void draw_gauge_scale(int selection, int count,
                             uint16_t bright, uint16_t dim, uint16_t red)
{
    for (int index = 0; index <= 25; ++index) {
        float angle = (205.0f + index * (130.0f / 25.0f)) * 3.14159265f / 180.0f;
        line(120 + (int)lroundf(cosf(angle) * 102),
             120 + (int)lroundf(sinf(angle) * 102),
             120 + (int)lroundf(cosf(angle) * 109),
             120 + (int)lroundf(sinf(angle) * 109),
             1, dim);
    }

    int major_count = count >= 2 && count <= 10 ? count : 6;
    for (int index = 0; index < major_count; ++index) {
        float angle = (205.0f + index * (130.0f / (major_count - 1))) *
                      3.14159265f / 180.0f;
        line(120 + (int)lroundf(cosf(angle) * 95),
             120 + (int)lroundf(sinf(angle) * 95),
             120 + (int)lroundf(cosf(angle) * 109),
             120 + (int)lroundf(sinf(angle) * 109), 2, bright);
    }

    float fraction = count > 1 ? (float)selection / (float)(count - 1) : 0.5f;
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    float marker_angle = (205.0f + fraction * 130.0f) * 3.14159265f / 180.0f;
    line(120 + (int)lroundf(cosf(marker_angle) * 93),
         120 + (int)lroundf(sinf(marker_angle) * 93),
         120 + (int)lroundf(cosf(marker_angle) * 110),
         120 + (int)lroundf(sinf(marker_angle) * 110), 4, red);
}

static uint8_t glyph_row(char character, int row)
{
    static const struct { char character; uint8_t rows[7]; } glyphs[] = {
        {' ',{0,0,0,0,0,0,0}}, {'-',{0,0,0,31,0,0,0}}, {'_',{0,0,0,0,0,0,31}},
        {'.',{0,0,0,0,0,6,6}}, {'/',{1,2,4,8,16,0,0}}, {'%',{17,2,4,8,17,0,0}},
        {'0',{14,17,19,21,25,17,14}}, {'1',{4,12,4,4,4,4,14}},
        {'2',{14,17,1,2,4,8,31}}, {'3',{30,1,1,14,1,1,30}},
        {'4',{2,6,10,18,31,2,2}}, {'5',{31,16,16,30,1,1,30}},
        {'6',{14,16,16,30,17,17,14}}, {'7',{31,1,2,4,8,8,8}},
        {'8',{14,17,17,14,17,17,14}}, {'9',{14,17,17,15,1,1,14}},
        {'A',{14,17,17,31,17,17,17}}, {'B',{30,17,17,30,17,17,30}},
        {'C',{14,17,16,16,16,17,14}}, {'D',{30,17,17,17,17,17,30}},
        {'E',{31,16,16,30,16,16,31}}, {'F',{31,16,16,30,16,16,16}},
        {'G',{14,17,16,23,17,17,15}}, {'H',{17,17,17,31,17,17,17}},
        {'I',{14,4,4,4,4,4,14}}, {'J',{7,2,2,2,2,18,12}},
        {'K',{17,18,20,24,20,18,17}}, {'L',{16,16,16,16,16,16,31}},
        {'M',{17,27,21,21,17,17,17}}, {'N',{17,25,21,19,17,17,17}},
        {'O',{14,17,17,17,17,17,14}}, {'P',{30,17,17,30,16,16,16}},
        {'Q',{14,17,17,17,21,18,13}}, {'R',{30,17,17,30,20,18,17}},
        {'S',{15,16,16,14,1,1,30}}, {'T',{31,4,4,4,4,4,4}},
        {'U',{17,17,17,17,17,17,14}}, {'V',{17,17,17,17,17,10,4}},
        {'W',{17,17,17,21,21,21,10}}, {'X',{17,17,10,4,10,17,17}},
        {'Y',{17,17,10,4,4,4,4}}, {'Z',{31,1,2,4,8,16,31}},
        {':',{0,6,6,0,6,6,0}}, {'?',{14,17,1,2,4,0,4}},
    };
    if (character >= 'a' && character <= 'z') character -= 'a' - 'A';
    for (size_t index = 0; index < sizeof(glyphs) / sizeof(glyphs[0]); ++index) {
        if (glyphs[index].character == character) return glyphs[index].rows[row];
    }
    return glyphs[sizeof(glyphs) / sizeof(glyphs[0]) - 1].rows[row];
}

static int text_width(const char *text, int scale)
{
    return text && *text ? ((int)strlen(text) * 6 - 1) * scale : 0;
}

static void text_at(int x, int y, const char *text, int scale, uint16_t color)
{
    if (!text) return;
    for (; *text; ++text, x += 6 * scale) {
        for (int row = 0; row < 7; ++row) {
            uint8_t bits = glyph_row(*text, row);
            for (int column = 0; column < 5; ++column) {
                if (bits & (1U << (4 - column))) {
                    fill_rect(x + column * scale, y + row * scale, scale, scale, color);
                }
            }
        }
    }
}

static void centered_text(int y, const char *text, int scale, uint16_t color)
{
    text_at((LCD_WIDTH - text_width(text, scale)) / 2, y, text, scale, color);
}

static void draw_icon(int item, int center_x, int center_y,
                      uint16_t primary, uint16_t accent)
{
    switch (item) {
    case 0: /* Dashboard view */
        outline_rect(center_x - 34, center_y - 25, 68, 50, 4, primary);
        arc(center_x, center_y + 8, 22, 205, 335, 3, accent);
        line(center_x, center_y + 8, center_x + 13, center_y - 3, 3, primary);
        break;
    case 1: /* Air ride coilover */
        line(center_x - 25, center_y - 35, center_x + 25, center_y - 35, 4, primary);
        ring(center_x, center_y - 35, 5, 4, accent);
        line(center_x, center_y - 29, center_x, center_y + 33, 4, primary);
        line(center_x - 15, center_y - 20, center_x + 15, center_y - 12, 4, accent);
        line(center_x + 15, center_y - 12, center_x - 15, center_y - 4, 4, accent);
        line(center_x - 15, center_y - 4, center_x + 15, center_y + 4, 4, accent);
        line(center_x + 15, center_y + 4, center_x - 15, center_y + 12, 4, accent);
        line(center_x - 15, center_y + 12, center_x + 15, center_y + 20, 4, accent);
        outline_rect(center_x - 10, center_y + 20, 20, 20, 3, primary);
        break;
    case 2: /* Climate fan */
        ring(center_x, center_y, 7, 5, accent);
        for (int blade = 0; blade < 4; ++blade) {
            float angle = blade * 3.14159265f / 2.0f;
            int blade_x = center_x + (int)lroundf(cosf(angle) * 22);
            int blade_y = center_y + (int)lroundf(sinf(angle) * 22);
            ring(blade_x, blade_y, 13, 8, primary);
        }
        ring(center_x, center_y, 38, 3, accent);
        break;
    case 3: /* Brightness */
        ring(center_x, center_y, 22, 7, accent);
        for (int degree = 0; degree < 360; degree += 45) {
            float angle = degree * 3.14159265f / 180.0f;
            line(center_x + (int)(cosf(angle) * 31), center_y + (int)(sinf(angle) * 31),
                 center_x + (int)(cosf(angle) * 40), center_y + (int)(sinf(angle) * 40), 4, primary);
        }
        break;
    case 4: /* Record */
        ring(center_x, center_y, 37, 5, primary);
        ring(center_x, center_y, 20, 21, accent);
        break;
    case 5: /* Trip timer */
        ring(center_x, center_y + 4, 32, 5, primary);
        fill_rect(center_x - 8, center_y - 37, 16, 7, primary);
        line(center_x, center_y + 4, center_x, center_y - 16, 4, accent);
        line(center_x, center_y + 4, center_x + 17, center_y + 14, 4, accent);
        break;
    case 6: /* Session peaks */
        line(center_x - 34, center_y + 28, center_x - 10, center_y - 4, 4, primary);
        line(center_x - 10, center_y - 4, center_x + 4, center_y + 10, 4, primary);
        line(center_x + 4, center_y + 10, center_x + 32, center_y - 30, 4, accent);
        fill_rect(center_x + 25, center_y - 30, 8, 8, accent);
        break;
    case 7: /* Shift light brightness */
        for (int index = 0; index < 5; ++index) {
            int x = center_x - 34 + index * 17;
            ring(x, center_y, 7, 5, index >= 3 ? accent : primary);
        }
        line(center_x - 35, center_y + 22, center_x + 35, center_y + 22, 3, primary);
        break;
    default: /* Driving logs */
        outline_rect(center_x - 31, center_y - 34, 62, 68, 4, primary);
        for (int row = 0; row < 3; ++row) {
            fill_rect(center_x - 20, center_y - 20 + row * 18, 7, 7, accent);
            line(center_x - 6, center_y - 17 + row * 18,
                 center_x + 20, center_y - 17 + row * 18, 3, primary);
        }
        break;
        break;
    }
}

static void draw_climate_icon(int control, int center_x, int center_y,
                              uint16_t primary, uint16_t accent)
{
    if (control == CLIMATE_FAN) {
        ring(center_x, center_y, 5, 5, accent);
        for (int blade = 0; blade < 4; ++blade) {
            float angle = blade * 3.14159265f / 2.0f;
            int x = center_x + (int)lroundf(cosf(angle) * 13);
            int y = center_y + (int)lroundf(sinf(angle) * 13);
            ring(x, y, 8, 5, primary);
        }
        ring(center_x, center_y, 25, 2, accent);
    } else if (control == CLIMATE_MODE) {
        arc(center_x, center_y + 8, 23, 200, 340, 3, primary);
        line(center_x - 22, center_y + 10, center_x + 22, center_y + 10, 3, primary);
        for (int offset = -12; offset <= 12; offset += 12) {
            line(center_x + offset, center_y + 4, center_x + offset - 5,
                 center_y - 14, 2, accent);
            line(center_x + offset - 5, center_y - 14, center_x + offset - 10,
                 center_y - 8, 2, accent);
        }
    } else {
        ring(center_x, center_y + 15, 10, 10, accent);
        outline_rect(center_x - 5, center_y - 23, 10, 38, 3, primary);
        fill_rect(center_x - 2, center_y - 9, 4, 26, accent);
        for (int offset = -14; offset <= 2; offset += 8) {
            line(center_x + 8, center_y + offset, center_x + 15,
                 center_y + offset, 2, primary);
        }
    }
}

static bool air_target_includes_bag(int target, int bag)
{
    if (target == AIR_TARGET_ALL_UP || target == AIR_TARGET_ALL_DOWN) return true;
    if (target >= AIR_TARGET_FRONT_LEFT && target <= AIR_TARGET_REAR_RIGHT) {
        return bag == target - AIR_TARGET_FRONT_LEFT;
    }
    return false;
}

static const char *climate_value_label(int control)
{
    int value = s_climate_values[control];
    switch ((climate_control_t)control) {
    case CLIMATE_FAN:
        return CLIMATE_FAN_VALUES[value];
    case CLIMATE_TEMP:
        return CLIMATE_TEMP_VALUES[value];
    case CLIMATE_MODE:
    default:
        return CLIMATE_MODE_VALUES[value];
    }
}

static bool link_connected(void)
{
    return s_last_link_rx && esp_timer_get_time() - s_last_link_rx < 2500000;
}

static bool screensaver_active(void)
{
    bool active;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_state_lock);
    active = now - s_last_activity >= SCREENSAVER_TIMEOUT_US;
    portEXIT_CRITICAL(&s_state_lock);
    return active;
}

static bool wake_screen(void)
{
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_state_lock);
    bool was_active = now - s_last_activity >= SCREENSAVER_TIMEOUT_US;
    s_last_activity = now;
    portEXIT_CRITICAL(&s_state_lock);
    return was_active;
}

static void draw_screensaver(void)
{
    const uint16_t background = RGB565(8, 9, 10);
    const uint16_t red = RGB565(228, 0, 43);
    const uint16_t emblem_colors[] = {
        background, RGB565(81, 6, 21), RGB565(154, 3, 32), red,
    };
    const int center_x = 120;
    const int center_y = 115;

    fill(background);
    draw_2bpp_mask(center_x - HONDA_EMBLEM_WIDTH / 2,
                   center_y - HONDA_EMBLEM_HEIGHT / 2,
                   HONDA_EMBLEM_WIDTH, HONDA_EMBLEM_HEIGHT,
                   honda_emblem_alpha, emblem_colors);
}

static void render(void)
{
    controller_state_t state;
    int home_selection;
    portENTER_CRITICAL(&s_state_lock);
    state = s_state;
    home_selection = s_home_selection;
    portEXIT_CRITICAL(&s_state_lock);

    if (screensaver_active()) {
        draw_screensaver();
        goto transmit;
    }

    const uint16_t background = RGB565(3, 4, 5);
    const uint16_t panel = RGB565(16, 18, 19);
    const uint16_t line_color = RGB565(48, 51, 53);
    const uint16_t white = RGB565(246, 246, 240);
    const uint16_t muted = RGB565(126, 130, 132);
    const uint16_t red = RGB565(238, 18, 46);
    fill(background);

    if (state.page == PAGE_HOME) {
        draw_gauge_scale(home_selection, HOME_ITEM_COUNT, white, line_color, red);
        arc(120, 120, 92, 214, 326, 2, panel);
        draw_icon(home_selection, 120, 91, white, red);
        centered_text(150, HOME_LABELS[home_selection], 3, white);

        for (int index = 0; index < HOME_ITEM_COUNT; ++index) {
            int x = 120 - (HOME_ITEM_COUNT * 13) / 2 + index * 13;
            if (index == home_selection) {
                fill_rect(x, 204, 11, 5, red);
            } else {
                outline_rect(x + 2, 204, 7, 5, 1, line_color);
            }
        }
    } else if (state.page == PAGE_CLIMATE) {
        int active = state.selection;
        fill_rect(96, 29, 48, 2, red);
        centered_text(39, "CLIMATE", 2, red);
        if (s_climate_mode == CLIMATE_BROWSE) {
            for (int control = 0; control < CLIMATE_CONTROL_COUNT; ++control) {
                int x = 13 + control * 73;
                bool selected = control == active;
                fill_rect(x, 66, 68, 105, selected ? panel : background);
                outline_rect(x, 66, 68, 105, selected ? 3 : 1,
                             selected ? red : line_color);
                draw_climate_icon(control, x + 34, 96,
                                  selected ? white : muted, red);
                text_at(x + (68 - text_width(CLIMATE_LABELS[control], 1)) / 2,
                        127, CLIMATE_LABELS[control], 1, selected ? red : muted);
                const char *value = climate_value_label(control);
                text_at(x + (68 - text_width(value, 1)) / 2, 148, value, 1, white);
            }
            centered_text(188, "ROTATE TO SELECT", 1, muted);
            centered_text(204, "PRESS TO ADJUST", 1, white);
        } else {
            draw_climate_icon(active, 120, 91, white, red);
            centered_text(126, CLIMATE_LABELS[active], 1, red);
            centered_text(145, climate_value_label(active), 2, white);
            int count = CLIMATE_VALUE_COUNTS[active];
            int width = count * 22 - 4;
            for (int index = 0; index < count; ++index) {
                fill_rect((LCD_WIDTH - width) / 2 + index * 22, 176, 18, 6,
                          index <= s_climate_values[active] ? red : line_color);
            }
            centered_text(198, "PRESS WHEN DONE", 1, muted);
        }
        centered_text(218, "HOLD FOR HOME", 1, muted);
    } else if (state.page == PAGE_AIR_RIDE) {
        static const char *const targets[AIR_TARGET_COUNT] = {
            "ALL UP", "ALL DOWN", "FRONT LEFT", "FRONT RIGHT",
            "REAR LEFT", "REAR RIGHT",
        };
        static const char *const corner_names[] = {"FL", "FR", "RL", "RR"};
        static const int corner_x[] = {24, 122, 24, 122};
        static const int corner_y[] = {103, 103, 143, 143};
        fill_rect(91, 13, 58, 3, red);
        centered_text(22, "AIR RIDE", 2, red);
        centered_text(49, targets[state.selection], 3, white);

        for (int corner = 0; corner < 4; ++corner) {
            bool selected = air_target_includes_bag(state.selection, corner);
            uint16_t border = selected ? red : line_color;
            outline_rect(corner_x[corner], corner_y[corner], 94, 34,
                         selected ? 3 : 1, border);
            char pressure_text[16];
            if (state.air_pressure_valid) {
                snprintf(pressure_text, sizeof(pressure_text), "%s %u",
                         corner_names[corner], state.air_pressure_psi[corner]);
            } else {
                snprintf(pressure_text, sizeof(pressure_text), "%s --", corner_names[corner]);
            }
            text_at(corner_x[corner] + 9, corner_y[corner] + 10,
                    pressure_text, 2, state.air_pressure_valid ? white : muted);
        }

        const char *instruction;
        bool action_active = esp_timer_get_time() < s_air_action_until;
        if (action_active) {
            instruction = s_air_action > 0 ? "SENDING UP" :
                          (s_air_action < 0 ? "SENDING DOWN" : "AIR OFFLINE");
        } else if (s_air_mode == AIR_CONTROLLER_ADJUST) {
            instruction = "TURN DOWN / UP";
        } else if (state.selection <= AIR_TARGET_ALL_DOWN) {
            instruction = "PRESS TO SEND";
        } else {
            instruction = "PRESS TO ADJUST";
        }
        centered_text(190, instruction, 1, action_active ? red : muted);
        char position[12];
        snprintf(position, sizeof(position), "%d / %d", state.selection + 1, AIR_TARGET_COUNT);
        centered_text(207, position, 1, muted);
        centered_text(222, "HOLD FOR HOME", 1, muted);
    } else {
        int item = (int)state.page - 1;
        if (item < 0 || item >= (int)(sizeof(CONTROLLER_PAGE_LABELS) /
                          sizeof(CONTROLLER_PAGE_LABELS[0]))) item = 3;
        draw_gauge_scale(state.selection, state.count, white, line_color, red);
        fill_rect(96, 36, 48, 2, red);
        const char *title = state.page == PAGE_ALERT ? "WARNING" :
                    (state.page == PAGE_TRIP_RESET ? "RESET TRIP" :
                                                     CONTROLLER_PAGE_LABELS[item]);
        centered_text(48, title, 2, red);
        const char *label = state.label[0] ? state.label : "WAITING";
        int scale = text_width(label, 3) <= 190 ? 3 : (text_width(label, 2) <= 190 ? 2 : 1);
        centered_text(99, label, scale, white);
        if (state.count > 1) {
            char position[20];
            snprintf(position, sizeof(position), "%02d // %02d", state.selection + 1, state.count);
            centered_text(147, position, 1, red);
        }
        if ((state.page == PAGE_TRIP && state.selection > 0) ||
            state.page == PAGE_PEAKS) {
            centered_text(174, "HOLD TO RESET", 1, muted);
        }
        outline_rect(84, 194, 72, 24, 2, line_color);
        fill_rect(84, 194, 5, 24, red);
        centered_text(202, state.page == PAGE_ALERT ? "DISMISS" : "BACK", 1, white);
    }

    uint16_t status = link_connected() ? RGB565(57, 255, 140) : line_color;
    fill_rect(116, 226, 8, 2, status);

transmit:
    esp_lcd_panel_io_tx_param(s_lcd_io, 0x2a, (uint8_t[]){0, 0, 0, 239}, 4);
    esp_lcd_panel_io_tx_param(s_lcd_io, 0x2b, (uint8_t[]){0, 0, 0, 239}, 4);
    esp_lcd_panel_io_tx_color(s_lcd_io, 0x2c, s_framebuffer,
                              LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t));
    xSemaphoreTake(s_lcd_done, pdMS_TO_TICKS(100));
}

static bool lcd_color_done(esp_lcd_panel_io_handle_t panel_io,
                           esp_lcd_panel_io_event_data_t *event_data, void *user_context)
{
    (void)panel_io;
    (void)event_data;
    (void)user_context;
    BaseType_t task_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_lcd_done, &task_woken);
    return task_woken == pdTRUE;
}

static void lcd_command(uint8_t command, const uint8_t *parameters, size_t parameter_count)
{
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_lcd_io, command, parameters, parameter_count));
}

static void lcd_init(void)
{
    gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << LCD_RST_GPIO) | (1ULL << BOARD_POWER_GPIO_1) |
                        (1ULL << BOARD_POWER_GPIO_2),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&outputs));
    gpio_set_level(BOARD_POWER_GPIO_1, 1);
    gpio_set_level(BOARD_POWER_GPIO_2, 1);
    gpio_set_level(LCD_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    spi_bus_config_t bus_config = {
        .sclk_io_num = LCD_SCLK_GPIO,
        .mosi_io_num = LCD_MOSI_GPIO,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_DC_GPIO,
        .cs_gpio_num = LCD_CS_GPIO,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 1,
        .on_color_trans_done = lcd_color_done,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                              &io_config, &s_lcd_io));

    lcd_command(0xef, (uint8_t[]){0x00}, 1);
    lcd_command(0xeb, (uint8_t[]){0x14}, 1);
    lcd_command(0xfe, NULL, 0);
    lcd_command(0xef, NULL, 0);
    lcd_command(0x84, (uint8_t[]){0x40}, 1);
    lcd_command(0x85, (uint8_t[]){0xff}, 1);
    lcd_command(0x86, (uint8_t[]){0xff}, 1);
    lcd_command(0x87, (uint8_t[]){0xff}, 1);
    lcd_command(0x88, (uint8_t[]){0x0a}, 1);
    lcd_command(0x89, (uint8_t[]){0x21}, 1);
    lcd_command(0x8a, (uint8_t[]){0x00}, 1);
    lcd_command(0x8b, (uint8_t[]){0x80}, 1);
    lcd_command(0x8c, (uint8_t[]){0x01}, 1);
    lcd_command(0x8d, (uint8_t[]){0x01}, 1);
    lcd_command(0x8e, (uint8_t[]){0xff}, 1);
    lcd_command(0x8f, (uint8_t[]){0xff}, 1);
    lcd_command(0xb6, (uint8_t[]){0x00, 0x20}, 2);
    lcd_command(0x36, (uint8_t[]){0x08}, 1);
    lcd_command(0x3a, (uint8_t[]){0x05}, 1);
    lcd_command(0x90, (uint8_t[]){0x08, 0x08, 0x08, 0x08}, 4);
    lcd_command(0xbd, (uint8_t[]){0x06}, 1);
    lcd_command(0xbc, (uint8_t[]){0x00}, 1);
    lcd_command(0xff, (uint8_t[]){0x60, 0x01, 0x04}, 3);
    lcd_command(0xc3, (uint8_t[]){0x13}, 1);
    lcd_command(0xc4, (uint8_t[]){0x13}, 1);
    lcd_command(0xc9, (uint8_t[]){0x22}, 1);
    lcd_command(0xbe, (uint8_t[]){0x11}, 1);
    lcd_command(0xe1, (uint8_t[]){0x10, 0x0e}, 2);
    lcd_command(0xdf, (uint8_t[]){0x21, 0x0c, 0x02}, 3);
    lcd_command(0xf0, (uint8_t[]){0x45, 0x09, 0x08, 0x08, 0x26, 0x2a}, 6);
    lcd_command(0xf1, (uint8_t[]){0x43, 0x70, 0x72, 0x36, 0x37, 0x6f}, 6);
    lcd_command(0xf2, (uint8_t[]){0x45, 0x09, 0x08, 0x08, 0x26, 0x2a}, 6);
    lcd_command(0xf3, (uint8_t[]){0x43, 0x70, 0x72, 0x36, 0x37, 0x6f}, 6);
    lcd_command(0xed, (uint8_t[]){0x1b, 0x0b}, 2);
    lcd_command(0xae, (uint8_t[]){0x77}, 1);
    lcd_command(0xcd, (uint8_t[]){0x63}, 1);
    lcd_command(0x70, (uint8_t[]){0x07, 0x07, 0x04, 0x0e, 0x0f, 0x09, 0x07, 0x08, 0x03}, 9);
    lcd_command(0xe8, (uint8_t[]){0x34}, 1);
    lcd_command(0x62, (uint8_t[]){0x18, 0x0d, 0x71, 0xed, 0x70, 0x70, 0x18, 0x0f, 0x71, 0xef, 0x70, 0x70}, 12);
    lcd_command(0x63, (uint8_t[]){0x18, 0x11, 0x71, 0xf1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xf3, 0x70, 0x70}, 12);
    lcd_command(0x64, (uint8_t[]){0x28, 0x29, 0xf1, 0x01, 0xf1, 0x00, 0x07}, 7);
    lcd_command(0x66, (uint8_t[]){0x3c, 0x00, 0xcd, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00}, 10);
    lcd_command(0x67, (uint8_t[]){0x00, 0x3c, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98}, 10);
    lcd_command(0x74, (uint8_t[]){0x10, 0x85, 0x80, 0x00, 0x00, 0x4e, 0x00}, 7);
    lcd_command(0x98, (uint8_t[]){0x3e, 0x07}, 2);
    lcd_command(0x21, NULL, 0);
    lcd_command(0x11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_command(0x29, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t channel = {
        .gpio_num = LCD_BACKLIGHT_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 820,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

static void send_frame(rotary_message_type_t type, const uint8_t *payload, uint16_t length)
{
    uint8_t frame[ROTARY_MAX_FRAME];
    if (length > ROTARY_MAX_PAYLOAD) return;
    if (xSemaphoreTake(s_uart_tx_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
    frame[0] = ROTARY_MAGIC_0;
    frame[1] = ROTARY_MAGIC_1;
    frame[2] = ROTARY_VERSION;
    frame[3] = (uint8_t)type;
    frame[4] = ++s_sequence;
    rotary_put_u16(&frame[5], length);
    if (length) memcpy(&frame[ROTARY_HEADER_SIZE], payload, length);
    size_t crc_offset = ROTARY_HEADER_SIZE + length;
    rotary_put_u16(&frame[crc_offset], rotary_crc16(frame, crc_offset));
    uart_write_bytes(LINK_UART, frame, crc_offset + ROTARY_CRC_SIZE);
    xSemaphoreGive(s_uart_tx_mutex);
}

static void ruin_air_state_changed(const ruin_air_state_t *state, void *context)
{
    (void)context;
    portENTER_CRITICAL(&s_state_lock);
    if (s_state.page == PAGE_AIR_RIDE) {
        s_state.air_pressure_valid = state->pressure_valid;
        s_state.air_pressure_psi[0] = state->front_left_psi;
        s_state.air_pressure_psi[1] = state->front_right_psi;
        s_state.air_pressure_psi[2] = state->rear_left_psi;
        s_state.air_pressure_psi[3] = state->rear_right_psi;
    }
    portEXIT_CRITICAL(&s_state_lock);
    uint8_t payload[14] = {
        state->connected ? 1 : 0,
        state->pressure_valid ? 1 : 0,
        state->compressor_valid ? 1 : 0,
        state->compressor_on ? 1 : 0,
    };
    rotary_put_u16(&payload[4], state->tank_psi);
    rotary_put_u16(&payload[6], state->front_left_psi);
    rotary_put_u16(&payload[8], state->front_right_psi);
    rotary_put_u16(&payload[10], state->rear_left_psi);
    rotary_put_u16(&payload[12], state->rear_right_psi);
    send_frame(ROTARY_AIR_RIDE_STATE, payload, sizeof(payload));
}

static bool ruin_air_is_known_mac(const uint8_t *address)
{
    return memcmp(address, RUIN_REMOTE_MAC, 6) == 0 ||
           memcmp(address, RUIN_CONTROLLER_MAC, 6) == 0;
}

static void capture_transmit(const uint8_t *frame, size_t length, void *context)
{
    (void)context;
    if (!s_capture_enabled || !frame || length > CAPTURE_MAX_FRAME_SIZE) return;
    capture_packet_t packet = {
        .timestamp_us = (uint64_t)esp_timer_get_time(),
        .captured_length = (uint16_t)length,
        .original_length = (uint16_t)length,
        .channel = CAPTURE_CHANNEL,
    };
    memcpy(packet.data, frame, length);
    if (xQueueSend(s_capture_queue, &packet, 0) != pdTRUE) ++s_capture_drops;
}

static void ruin_air_receive(void *buffer, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_MGMT || !buffer) return;
    const wifi_promiscuous_pkt_t *received = buffer;
    uint16_t original_length = received->rx_ctrl.sig_len;
    const uint8_t *frame = received->payload;
    if (original_length < 39 || frame[0] != 0xd0 || frame[24] != 0x7f ||
        memcmp(&frame[25], "\x18\xfe\x34", 3) != 0 ||
        (!ruin_air_is_known_mac(&frame[4]) && !ruin_air_is_known_mac(&frame[10]) &&
         !ruin_air_is_known_mac(&frame[16]))) return;
    ruin_air_bridge_ingest_frame(frame, original_length);
    if (!s_capture_enabled) return;

    capture_packet_t packet = {
        .timestamp_us = (uint64_t)esp_timer_get_time(),
        .captured_length = original_length > CAPTURE_MAX_FRAME_SIZE ?
                           CAPTURE_MAX_FRAME_SIZE : original_length,
        .original_length = original_length,
        .rssi = received->rx_ctrl.rssi,
        .channel = received->rx_ctrl.channel,
    };
    memcpy(packet.data, received->payload, packet.captured_length);
    if (xQueueSend(s_capture_queue, &packet, 0) != pdTRUE) ++s_capture_drops;
}

static void send_capture_status(void)
{
    uint8_t payload[10] = {s_capture_enabled ? 1 : 0, CAPTURE_CHANNEL};
    rotary_put_u32(&payload[2], s_capture_packets);
    rotary_put_u32(&payload[6], s_capture_drops);
    send_frame(ROTARY_CAPTURE_STATUS, payload, sizeof(payload));
}

static esp_err_t capture_set_enabled(bool enabled)
{
    if (!s_wifi_initialized) return ESP_ERR_INVALID_STATE;
    esp_err_t error = enabled ?
        esp_wifi_set_channel(CAPTURE_CHANNEL, WIFI_SECOND_CHAN_NONE) : ESP_OK;
    if (error == ESP_OK) {
        s_capture_enabled = enabled;
        if (enabled) {
            s_capture_packets = 0;
            s_capture_drops = 0;
            xQueueReset(s_capture_queue);
        }
    }
    send_capture_status();
    return error;
}

static void capture_task(void *argument)
{
    (void)argument;
    capture_packet_t packet;
    while (true) {
        if (xQueueReceive(s_capture_queue, &packet, portMAX_DELAY) != pdTRUE) continue;
        uint16_t packet_id = ++s_capture_packet_id;
        for (uint16_t offset = 0; offset < packet.captured_length;) {
            uint16_t remaining = packet.captured_length - offset;
            uint16_t chunk_length = remaining > ROTARY_CAPTURE_CHUNK_DATA_SIZE ?
                                    ROTARY_CAPTURE_CHUNK_DATA_SIZE : remaining;
            uint8_t payload[ROTARY_MAX_PAYLOAD];
            rotary_put_u16(&payload[0], packet_id);
            rotary_put_u64(&payload[2], packet.timestamp_us);
            rotary_put_u16(&payload[10], packet.captured_length);
            rotary_put_u16(&payload[12], packet.original_length);
            rotary_put_u16(&payload[14], offset);
            payload[16] = (offset == 0 ? ROTARY_CAPTURE_FLAG_FIRST : 0) |
                          (offset + chunk_length == packet.captured_length ?
                           ROTARY_CAPTURE_FLAG_LAST : 0);
            payload[17] = (uint8_t)packet.rssi;
            payload[18] = packet.channel;
            memcpy(&payload[ROTARY_CAPTURE_CHUNK_HEADER_SIZE], &packet.data[offset], chunk_length);
            send_frame(ROTARY_CAPTURE_CHUNK, payload,
                       ROTARY_CAPTURE_CHUNK_HEADER_SIZE + chunk_length);
            offset += chunk_length;
        }
        ++s_capture_packets;
    }
}

static void send_event(rotary_event_t event, int16_t value)
{
    uint8_t payload[3] = {(uint8_t)event};
    rotary_put_u16(&payload[1], (uint16_t)value);
    send_frame(ROTARY_EVENT, payload, sizeof(payload));
}

static void show_offline_page(int selected)
{
    portENTER_CRITICAL(&s_state_lock);
    s_state = (controller_state_t) {
        .page = selected >= 0 && selected < (int)(sizeof(HOME_PAGE_VALUES) / sizeof(HOME_PAGE_VALUES[0])) ?
                HOME_PAGE_VALUES[selected] : PAGE_HOME,
    };
    snprintf(s_state.label, sizeof(s_state.label), "NO P4 LINK");
    portEXIT_CRITICAL(&s_state_lock);
}

static void show_home(void)
{
    portENTER_CRITICAL(&s_state_lock);
    s_state = (controller_state_t) {
        .page = PAGE_HOME,
        .count = HOME_ITEM_COUNT,
    };
    portEXIT_CRITICAL(&s_state_lock);
}

static void air_open(void)
{
    portENTER_CRITICAL(&s_state_lock);
    s_air_mode = AIR_CONTROLLER_BROWSE;
    s_air_action = 0;
    s_state = (controller_state_t) {
        .page = PAGE_AIR_RIDE,
        .count = AIR_TARGET_COUNT,
    };
    portEXIT_CRITICAL(&s_state_lock);
    send_event(ROTARY_OPEN, PAGE_AIR_RIDE);
}

static void air_send(uint8_t command, int direction)
{
    esp_err_t error = ruin_air_bridge_request(command);
    portENTER_CRITICAL(&s_state_lock);
    s_air_action = error == ESP_OK ? direction : 0;
    s_air_action_until = esp_timer_get_time() + 700000;
    portEXIT_CRITICAL(&s_state_lock);
    if (error != ESP_OK) {
        ESP_LOGW(TAG, "Air Ride command failed: %s", esp_err_to_name(error));
    }
}

static void air_rotate(int direction)
{
    int selection;
    air_controller_mode_t mode;
    portENTER_CRITICAL(&s_state_lock);
    mode = s_air_mode;
    if (mode == AIR_CONTROLLER_BROWSE) {
        s_state.selection = (s_state.selection + direction + AIR_TARGET_COUNT) % AIR_TARGET_COUNT;
    }
    selection = s_state.selection;
    portEXIT_CRITICAL(&s_state_lock);
    if (mode == AIR_CONTROLLER_BROWSE) {
        send_event(ROTARY_ROTATE, direction);
        return;
    }
    if (mode != AIR_CONTROLLER_ADJUST) return;

    static const uint8_t up_commands[] = {0, 0, 0, 2, 4, 6};
    static const uint8_t down_commands[] = {0, 0, 1, 3, 5, 7};
    air_send(direction > 0 ? up_commands[selection] : down_commands[selection], direction);
}

static void air_press(void)
{
    int selection;
    bool mode_changed = false;
    portENTER_CRITICAL(&s_state_lock);
    selection = s_state.selection;
    if (selection >= AIR_TARGET_FRONT_LEFT) {
        s_air_mode = s_air_mode == AIR_CONTROLLER_BROWSE ?
                     AIR_CONTROLLER_ADJUST : AIR_CONTROLLER_BROWSE;
        mode_changed = true;
    }
    portEXIT_CRITICAL(&s_state_lock);
    if (mode_changed) send_event(ROTARY_PRESS, 0);
    if (selection == AIR_TARGET_ALL_UP) air_send(8, 1);
    if (selection == AIR_TARGET_ALL_DOWN) air_send(9, -1);
}

static void climate_open(void)
{
    portENTER_CRITICAL(&s_state_lock);
    s_climate_mode = CLIMATE_BROWSE;
    s_state = (controller_state_t) {
        .page = PAGE_CLIMATE,
        .selection = CLIMATE_FAN,
        .count = CLIMATE_CONTROL_COUNT,
    };
    portEXIT_CRITICAL(&s_state_lock);
    send_event(ROTARY_OPEN, PAGE_CLIMATE);
}

static void climate_apply(void)
{
    portENTER_CRITICAL(&s_state_lock);
    uint8_t fan = CLIMATE_FAN_NATIVE[s_climate_values[CLIMATE_FAN]];
    uint8_t mode = (uint8_t)s_climate_values[CLIMATE_MODE];
    uint8_t temp = CLIMATE_TEMP_NATIVE[s_climate_values[CLIMATE_TEMP]];
    s_hvac_state.target_fan = fan;
    s_hvac_state.target_mode = mode;
    s_hvac_state.target_temp = temp;
    portEXIT_CRITICAL(&s_state_lock);
    esp_err_t error = hvac_bridge_set(fan, mode, temp);
    if (error != ESP_OK) ESP_LOGW(TAG, "HVAC command failed: %s", esp_err_to_name(error));
    send_hvac_uart_state();
}

static void climate_rotate(int direction)
{
    portENTER_CRITICAL(&s_state_lock);
    int control = s_state.selection;
    if (s_climate_mode == CLIMATE_BROWSE) {
        s_state.selection = (control + direction + CLIMATE_CONTROL_COUNT) %
                            CLIMATE_CONTROL_COUNT;
    } else {
        int value = s_climate_values[control] + direction;
        if (value < 0) value = 0;
        if (value >= CLIMATE_VALUE_COUNTS[control]) value = CLIMATE_VALUE_COUNTS[control] - 1;
        s_climate_values[control] = value;
    }
    climate_controller_mode_t mode = s_climate_mode;
    portEXIT_CRITICAL(&s_state_lock);
    send_event(ROTARY_ROTATE, direction);
    if (mode == CLIMATE_ADJUST) climate_apply();
}

static void climate_press(void)
{
    portENTER_CRITICAL(&s_state_lock);
    s_climate_mode = s_climate_mode == CLIMATE_BROWSE ?
                     CLIMATE_ADJUST : CLIMATE_BROWSE;
    portEXIT_CRITICAL(&s_state_lock);
    send_event(ROTARY_PRESS, 0);
}

static void apply_frame(const uint8_t *frame, size_t frame_length)
{
    uint16_t payload_length = rotary_get_u16(&frame[5]);
    if (frame_length != ROTARY_HEADER_SIZE + payload_length + ROTARY_CRC_SIZE) return;
    if (rotary_get_u16(&frame[frame_length - 2]) != rotary_crc16(frame, frame_length - 2)) return;
    if (frame[3] == ROTARY_CAPTURE_CONTROL) {
        const uint8_t *payload = &frame[ROTARY_HEADER_SIZE];
        if (payload_length == 1 && payload[0] <= ROTARY_CAPTURE_START) {
            esp_err_t error = capture_set_enabled(payload[0] == ROTARY_CAPTURE_START);
            if (error != ESP_OK) ESP_LOGE(TAG, "Capture control failed: %s", esp_err_to_name(error));
        }
        return;
    }
    if (frame[3] == ROTARY_AIR_RIDE_COMMAND) {
        const uint8_t *payload = &frame[ROTARY_HEADER_SIZE];
        if (payload_length == 1 ||
            (payload_length == 5 && (payload[0] == 20 || payload[0] == 21))) {
            esp_err_t error = payload[0] == 22 ? ruin_air_bridge_air_out() :
                payload_length == 5 ?
                ruin_air_bridge_set_compressor(payload[0] == 20,
                                               rotary_get_u16(&payload[1]),
                                               rotary_get_u16(&payload[3])) :
                ruin_air_bridge_request(payload[0]);
            if (error != ESP_OK) ESP_LOGE(TAG, "Air Ride command failed: %s",
                                          esp_err_to_name(error));
        }
        return;
    }
    if (frame[3] == ROTARY_HVAC_COMMAND) {
        const uint8_t *payload = &frame[ROTARY_HEADER_SIZE];
        if (payload_length != 3 || payload[0] > 4 || payload[1] > 4 ||
            payload[2] > 4) return;
        portENTER_CRITICAL(&s_state_lock);
        s_climate_values[CLIMATE_FAN] = payload[0];
        s_climate_values[CLIMATE_MODE] = payload[1];
        s_climate_values[CLIMATE_TEMP] = payload[2];
        portEXIT_CRITICAL(&s_state_lock);
        climate_apply();
        return;
    }
    if (frame[3] != ROTARY_STATE && frame[3] != ROTARY_PONG) return;
    s_last_link_rx = esp_timer_get_time();
    if (frame[3] == ROTARY_PONG) return;
    if (payload_length < 19) return;

    const uint8_t *payload = &frame[ROTARY_HEADER_SIZE];
    controller_state_t next = {
        .page = (controller_page_t)payload[0],
        .selection = (int16_t)rotary_get_u16(&payload[1]),
        .count = (int16_t)rotary_get_u16(&payload[3]),
        .value = (int32_t)rotary_get_u32(&payload[5]),
        .recording = payload[9] != 0,
        .air_pressure_valid = payload[10] != 0,
    };
    for (size_t index = 0; index < 4; ++index) {
        next.air_pressure_psi[index] = rotary_get_u16(&payload[11 + index * 2]);
    }
    size_t label_length = payload_length - 19;
    if (label_length >= sizeof(next.label)) label_length = sizeof(next.label) - 1;
    memcpy(next.label, &payload[19], label_length);
    next.label[label_length] = '\0';
    portENTER_CRITICAL(&s_state_lock);
    if (s_state.page != PAGE_CLIMATE && s_state.page != PAGE_AIR_RIDE) s_state = next;
    portEXIT_CRITICAL(&s_state_lock);
}

static void parser_push(frame_parser_t *parser, uint8_t byte)
{
    if (parser->length == 0 && byte != ROTARY_MAGIC_0) return;
    if (parser->length == 1 && byte != ROTARY_MAGIC_1) {
        parser->length = byte == ROTARY_MAGIC_0 ? 1 : 0;
        parser->data[0] = ROTARY_MAGIC_0;
        return;
    }
    parser->data[parser->length++] = byte;
    if (parser->length == ROTARY_HEADER_SIZE) {
        uint16_t payload_length = rotary_get_u16(&parser->data[5]);
        if (parser->data[2] != ROTARY_VERSION || payload_length > ROTARY_MAX_PAYLOAD) {
            parser->length = 0;
            return;
        }
        parser->expected_length = ROTARY_HEADER_SIZE + payload_length + ROTARY_CRC_SIZE;
    }
    if (parser->expected_length && parser->length == parser->expected_length) {
        apply_frame(parser->data, parser->length);
        parser->length = 0;
        parser->expected_length = 0;
    } else if (parser->length >= sizeof(parser->data)) {
        parser->length = 0;
        parser->expected_length = 0;
    }
}

static void uart_task(void *argument)
{
    (void)argument;
    frame_parser_t parser = {0};
    uint8_t bytes[64];
    while (true) {
        int count = uart_read_bytes(LINK_UART, bytes, sizeof(bytes), pdMS_TO_TICKS(50));
        for (int index = 0; index < count; ++index) parser_push(&parser, bytes[index]);
    }
}

static void input_task(void *argument)
{
    (void)argument;
    static const int8_t transitions[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
    int previous = (gpio_get_level(ENCODER_A_GPIO) << 1) | gpio_get_level(ENCODER_B_GPIO);
    int accumulator = 0;
    bool button_was_down = false;
    bool button_woke_screen = false;
    bool long_sent = false;
    int64_t button_down_at = 0;
    int64_t last_click_at = 0;
    while (true) {
        uint8_t current;
        while (xQueueReceive(s_encoder_queue, &current, 0) == pdTRUE) {
            accumulator += transitions[(previous << 2) | current];
            previous = current;
            if (accumulator >= 4 || accumulator <= -4) {
                int direction = accumulator > 0 ? 1 : -1;
                accumulator = 0;
                if (wake_screen()) continue;
                portENTER_CRITICAL(&s_state_lock);
                bool home = s_state.page == PAGE_HOME;
                bool climate = s_state.page == PAGE_CLIMATE;
                bool air_ride = s_state.page == PAGE_AIR_RIDE;
                if (home) {
                    s_home_selection = (s_home_selection + direction + HOME_ITEM_COUNT) %
                                       HOME_ITEM_COUNT;
                }
                portEXIT_CRITICAL(&s_state_lock);
                if (climate) climate_rotate(direction);
                else if (air_ride) air_rotate(direction);
                else if (!home) send_event(ROTARY_ROTATE, direction);
            }
        }

        bool button_down = gpio_get_level(ENCODER_BUTTON_GPIO) == 0;
        int64_t now = esp_timer_get_time();
        if (button_down && !button_was_down) {
            button_down_at = now;
            button_woke_screen = wake_screen();
            long_sent = false;
        } else if (button_down && !button_woke_screen && !long_sent &&
                   now - button_down_at >= 700000) {
            portENTER_CRITICAL(&s_state_lock);
            bool home = s_state.page == PAGE_HOME;
            bool climate = s_state.page == PAGE_CLIMATE;
            bool air_ride = s_state.page == PAGE_AIR_RIDE;
            portEXIT_CRITICAL(&s_state_lock);
            if (air_ride) {
                show_home();
                send_event(ROTARY_LONG_PRESS, 0);
            }
            else if (climate) {
                show_home();
                send_event(ROTARY_LONG_PRESS, 0);
            }
            else if (!home) send_event(ROTARY_LONG_PRESS, 0);
            long_sent = true;
        } else if (!button_down && button_was_down && !button_woke_screen &&
                    !long_sent && now - button_down_at > 25000 &&
                    now - last_click_at >= 250000) {
                last_click_at = now;
            portENTER_CRITICAL(&s_state_lock);
            bool home = s_state.page == PAGE_HOME;
            bool climate = s_state.page == PAGE_CLIMATE;
            bool air_ride = s_state.page == PAGE_AIR_RIDE;
            int selected = s_home_selection;
            portEXIT_CRITICAL(&s_state_lock);
            if (air_ride) {
                air_press();
                button_was_down = button_down;
                vTaskDelay(1);
                continue;
            }
            if (climate) {
                climate_press();
                button_was_down = button_down;
                vTaskDelay(1);
                continue;
            }
            if (home && selected == CLIMATE_HOME_INDEX) {
                climate_open();
                button_was_down = button_down;
                vTaskDelay(1);
                continue;
            }
            if (home && selected == 1) {
                air_open();
                button_was_down = button_down;
                vTaskDelay(1);
                continue;
            }
            bool connected = link_connected();
            if (!connected) {
                if (home) show_offline_page(selected);
                else show_home();
            }
            ESP_LOGI(TAG, "Encoder click: %s", home ? HOME_LABELS[selected] : "BACK");
            send_event(home ? ROTARY_OPEN : ROTARY_PRESS,
                       home && selected < (int)(sizeof(HOME_PAGE_VALUES) / sizeof(HOME_PAGE_VALUES[0])) ?
                       HOME_PAGE_VALUES[selected] : 0);
        }
        button_was_down = button_down;
        vTaskDelay(1);
    }
}

static bool touch_pressed(int *x, int *y)
{
    uint8_t register_address = 0x01;
    uint8_t data[6] = {0};
    esp_err_t result = i2c_master_write_read_device(I2C_NUM_0, CST816_ADDRESS,
                                                     &register_address, 1, data, sizeof(data),
                                                     pdMS_TO_TICKS(20));
    if (result != ESP_OK || (data[1] & 0x0f) == 0) return false;
    if (x) *x = ((data[2] & 0x0f) << 8) | data[3];
    if (y) *y = ((data[4] & 0x0f) << 8) | data[5];
    return true;
}

static void touch_task(void *argument)
{
    (void)argument;
    bool was_pressed = false;
    while (true) {
        int touch_x = 0;
        int touch_y = 0;
        bool pressed = gpio_get_level(TOUCH_INT_GPIO) == 0 &&
                       touch_pressed(&touch_x, &touch_y);
        if (pressed && !was_pressed) {
            if (wake_screen()) {
                was_pressed = pressed;
                vTaskDelay(pdMS_TO_TICKS(25));
                continue;
            }
            portENTER_CRITICAL(&s_state_lock);
            bool home = s_state.page == PAGE_HOME;
            bool climate = s_state.page == PAGE_CLIMATE;
            bool air_ride = s_state.page == PAGE_AIR_RIDE;
            portEXIT_CRITICAL(&s_state_lock);
            bool home_button = air_ride ?
                (touch_y >= 210) :
                (touch_x >= 76 && touch_x <= 164 && touch_y >= 188 && touch_y <= 224);
            if (!home && home_button) {
                if (air_ride) {
                    show_home();
                    send_event(ROTARY_LONG_PRESS, 0);
                } else {
                    if (climate || !link_connected()) show_home();
                    send_event(ROTARY_BACK, 0);
                }
            }
        }
        was_pressed = pressed;
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

static void ui_task(void *argument)
{
    (void)argument;
    while (true) {
        render();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void communication_task(void *argument)
{
    (void)argument;
    int counter = 0;
    while (true) {
        send_frame((counter++ % 4) == 0 ? ROTARY_PING : ROTARY_GET_STATE, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void peripherals_init(void)
{
    gpio_config_t inputs = {
        .pin_bit_mask = (1ULL << ENCODER_A_GPIO) | (1ULL << ENCODER_B_GPIO) |
                        (1ULL << ENCODER_BUTTON_GPIO) | (1ULL << TOUCH_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&inputs));
    ESP_ERROR_CHECK(gpio_set_intr_type(ENCODER_A_GPIO, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(ENCODER_B_GPIO, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_A_GPIO, encoder_edge_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_B_GPIO, encoder_edge_isr, NULL));

    gpio_config_t touch_reset = {
        .pin_bit_mask = 1ULL << TOUCH_RST_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&touch_reset));
    gpio_set_level(TOUCH_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(TOUCH_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TOUCH_SDA_GPIO,
        .scl_io_num = TOUCH_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    uart_config_t uart_config = {
        .baud_rate = LINK_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(LINK_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LINK_UART, LINK_TX_GPIO, LINK_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(LINK_UART, 512, 0, 0, NULL, 0));
}

void app_main(void)
{
    s_lcd_done = xSemaphoreCreateBinary();
    s_uart_tx_mutex = xSemaphoreCreateMutex();
    s_encoder_queue = xQueueCreate(64, sizeof(uint8_t));
    s_capture_queue = xQueueCreate(CAPTURE_QUEUE_LENGTH, sizeof(capture_packet_t));
    s_framebuffer = heap_caps_malloc(LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t), MALLOC_CAP_DMA);
    ESP_ERROR_CHECK(s_lcd_done && s_uart_tx_mutex && s_encoder_queue && s_capture_queue &&
                    s_framebuffer ? ESP_OK : ESP_ERR_NO_MEM);
    lcd_init();
    peripherals_init();
    esp_err_t ruin_error = ruin_air_bridge_init(ruin_air_state_changed, NULL);
    if (ruin_error == ESP_OK) {
        s_wifi_initialized = true;
        ruin_air_bridge_set_tx_callback(capture_transmit, NULL);
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_promiscuous_rx_cb(ruin_air_receive));
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_promiscuous(true));
    } else {
        ESP_LOGE(TAG, "Ruin Air bridge init failed: %s", esp_err_to_name(ruin_error));
    }
    esp_err_t hvac_error = hvac_bridge_init(hvac_state_changed, NULL);
    if (hvac_error != ESP_OK) {
        ESP_LOGE(TAG, "HVAC bridge init failed: %s", esp_err_to_name(hvac_error));
    }
    s_last_activity = esp_timer_get_time();
    ESP_LOGI(TAG, "MackoDash rotary controller started");
    xTaskCreate(uart_task, "rotary_uart", 4096, NULL, 6, NULL);
    xTaskCreate(capture_task, "ruin_capture", 4096, NULL, 5, NULL);
    xTaskCreate(input_task, "rotary_input", 3072, NULL, 5, NULL);
    xTaskCreate(touch_task, "rotary_touch", 3072, NULL, 4, NULL);
    xTaskCreate(communication_task, "rotary_link", 3072, NULL, 4, NULL);
    xTaskCreate(ui_task, "rotary_ui", 4096, NULL, 3, NULL);
}