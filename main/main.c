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
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "honda_emblem.h"
#include "knob_ui.h"
#include "rotary_protocol.h"

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
#define CST816_ADDRESS 0x15
#define SCREENSAVER_TIMEOUT_US (60LL * 1000 * 1000)
#define LINK_TIMEOUT_US 2500000
#define TOAST_US 1400000
#define TOAST_WAIT_US 700000
#define BACKLIGHT_DAY 820
#define BACKLIGHT_NIGHT 160

#define RGB565(red, green, blue) \
    ((uint16_t)((((red) & 0xf8) << 8) | (((green) & 0xfc) << 3) | ((blue) >> 3)))

/* Page numbers are shared with the dashboard (honda_dash_controller_page_t). */
typedef enum {
    PAGE_HOME = 0,
    PAGE_VIEW = 1,
    PAGE_BRIGHTNESS = 2,
    PAGE_RECORD = 3,
    PAGE_TRIP = 4,
    PAGE_PEAKS = 5,
    PAGE_SHIFT = 6,
    PAGE_LOGS = 7,
    PAGE_TRIP_RESET = 8,
    PAGE_ALERT = 9,
    PAGE_GAUGE = 12,
    PAGE_NIGHT = 13,
} controller_page_t;

typedef struct {
    controller_page_t page;
    int16_t selection;
    int16_t count;
    int32_t value;
    bool recording;
    bool night;
    char label[48];
} controller_state_t;

typedef struct {
    const char *title;
    knob_icon_t icon;
    controller_page_t page;
    const char *hint;
} home_item_t;

typedef struct {
    knob_icon_t icon;
    knob_tone_t tone;
    char caption[16];
    char text[24];
} toast_t;

typedef struct {
    uint8_t data[ROTARY_MAX_FRAME];
    size_t length;
    size_t expected_length;
} frame_parser_t;

/* Everything a frame depends on; the panel is only redrawn when this changes. */
typedef struct {
    controller_state_t state;
    int home_selection;
    bool linked;
    bool saver;
    bool toast_active;
    toast_t toast;
} render_key_t;

static const char *TAG = "mackodash_rotary";

static const home_item_t HOME_ITEMS[] = {
    {"GAUGE", KNOB_ICON_GAUGE, PAGE_GAUGE, "LIVE READOUT"},
    {"VIEW", KNOB_ICON_VIEW, PAGE_VIEW, "DASH THEME"},
    {"BRIGHT", KNOB_ICON_BRIGHT, PAGE_BRIGHTNESS, "SCREEN LEVEL"},
    {"NIGHT", KNOB_ICON_NIGHT, PAGE_NIGHT, NULL},
    {"REC", KNOB_ICON_REC, PAGE_RECORD, NULL},
    {"TRIP", KNOB_ICON_TRIP, PAGE_TRIP, "ODO / TRIP A / B"},
    {"PEAKS", KNOB_ICON_PEAKS, PAGE_PEAKS, "SESSION MAX"},
};
#define HOME_ITEM_COUNT ((int)(sizeof(HOME_ITEMS) / sizeof(HOME_ITEMS[0])))

static esp_lcd_panel_io_handle_t s_lcd_io;
static SemaphoreHandle_t s_lcd_done;
static SemaphoreHandle_t s_uart_tx_mutex;
static QueueHandle_t s_encoder_queue;
static uint16_t *s_framebuffer;
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;

/* all of the following are guarded by s_state_lock */
static controller_state_t s_state = {.page = PAGE_HOME};
static int s_home_selection;
static int64_t s_last_link_rx;
static int64_t s_last_activity;
static toast_t s_toast;
static int64_t s_toast_until;
static controller_page_t s_toast_pending;   /* PAGE_RECORD / PAGE_NIGHT awaiting the dash reply */
static bool s_toast_prev_flag;
static int64_t s_toast_deadline;

static uint8_t s_sequence;

/* ------------------------------------------------------------- link */

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

static void send_event(rotary_event_t event, int16_t value)
{
    uint8_t payload[3] = {(uint8_t)event};
    rotary_put_u16(&payload[1], (uint16_t)value);
    send_frame(ROTARY_EVENT, payload, sizeof(payload));
}

static bool link_connected_locked(int64_t now)
{
    return s_last_link_rx && now - s_last_link_rx < LINK_TIMEOUT_US;
}

/* ------------------------------------------------------------ toasts */

static void toast_set_locked(knob_icon_t icon, knob_tone_t tone, const char *caption, const char *text)
{
    memset(&s_toast, 0, sizeof(s_toast));
    s_toast.icon = icon;
    s_toast.tone = tone;
    snprintf(s_toast.caption, sizeof(s_toast.caption), "%s", caption ? caption : "");
    snprintf(s_toast.text, sizeof(s_toast.text), "%s", text);
    s_toast_until = esp_timer_get_time() + TOAST_US;
}

/* Called with the lock held once the dash has answered a REC / NIGHT press. */
static void toast_resolve_locked(bool force)
{
    if (s_toast_pending == PAGE_HOME) return;
    bool flag = s_toast_pending == PAGE_RECORD ? s_state.recording : s_state.night;
    if (flag == s_toast_prev_flag && !force) return;
    if (s_toast_pending == PAGE_RECORD) {
        toast_set_locked(KNOB_ICON_REC, flag ? KNOB_TONE_CRITICAL : KNOB_TONE_NORMAL, NULL,
                         flag ? "LOGGING STARTED" : "LOGGING STOPPED");
    } else {
        toast_set_locked(flag ? KNOB_ICON_NIGHT : KNOB_ICON_BRIGHT, KNOB_TONE_NORMAL, NULL,
                         flag ? "NIGHT MODE ON" : "DAY MODE ON");
    }
    s_toast_pending = PAGE_HOME;
}

/* ------------------------------------------------------------ actions */

static void go_home_locally(void)
{
    portENTER_CRITICAL(&s_state_lock);
    s_state.page = PAGE_HOME;
    s_state.label[0] = '\0';
    portEXIT_CRITICAL(&s_state_lock);
}

static void open_home_item(int index)
{
    const home_item_t *item = &HOME_ITEMS[index];
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_state_lock);
    if (!link_connected_locked(now)) {
        toast_set_locked(KNOB_ICON_LINK, KNOB_TONE_NORMAL, "NO DASH LINK", "CHECK KNOB CABLE");
        portEXIT_CRITICAL(&s_state_lock);
        return;
    }
    if (item->page == PAGE_RECORD || item->page == PAGE_NIGHT) {
        s_toast_pending = item->page;
        s_toast_prev_flag = item->page == PAGE_RECORD ? s_state.recording : s_state.night;
        s_toast_deadline = now + TOAST_WAIT_US;
    } else {
        /* show the page frame straight away; the dash fills in the values */
        s_state.page = item->page;
        s_state.label[0] = '\0';
        s_state.selection = 0;
        s_state.count = 0;
        s_state.value = item->page == PAGE_GAUGE ? 0xFFFF : 0;
    }
    portEXIT_CRITICAL(&s_state_lock);
    ESP_LOGI(TAG, "Open %s", item->title);
    send_event(ROTARY_OPEN, item->page);
}

static void apply_frame(const uint8_t *frame, size_t frame_length)
{
    uint16_t payload_length = rotary_get_u16(&frame[5]);
    if (frame_length != ROTARY_HEADER_SIZE + payload_length + ROTARY_CRC_SIZE) return;
    if (rotary_get_u16(&frame[frame_length - 2]) != rotary_crc16(frame, frame_length - 2)) return;
    if (frame[3] != ROTARY_STATE && frame[3] != ROTARY_PONG) return;
    int64_t now = esp_timer_get_time();
    if (frame[3] == ROTARY_PONG) {
        portENTER_CRITICAL(&s_state_lock);
        s_last_link_rx = now;
        portEXIT_CRITICAL(&s_state_lock);
        return;
    }
    if (payload_length < 19) return;

    const uint8_t *payload = &frame[ROTARY_HEADER_SIZE];
    controller_state_t next = {
        .page = (controller_page_t)payload[0],
        .selection = (int16_t)rotary_get_u16(&payload[1]),
        .count = (int16_t)rotary_get_u16(&payload[3]),
        .value = (int32_t)rotary_get_u32(&payload[5]),
        .recording = payload[9] != 0,
        .night = (payload[10] & 0x01) != 0,
    };
    size_t label_length = payload_length - 19;
    if (label_length >= sizeof(next.label)) label_length = sizeof(next.label) - 1;
    memcpy(next.label, &payload[19], label_length);
    next.label[label_length] = '\0';

    portENTER_CRITICAL(&s_state_lock);
    s_last_link_rx = now;
    s_state = next;
    toast_resolve_locked(false);
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

/* ------------------------------------------------------------ screen */

static bool page_keeps_screen_on(controller_page_t page)
{
    return page == PAGE_GAUGE || page == PAGE_ALERT;
}

static bool wake_screen(void)
{
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_state_lock);
    bool was_saver = now - s_last_activity >= SCREENSAVER_TIMEOUT_US &&
                     !page_keeps_screen_on(s_state.page);
    s_last_activity = now;
    portEXIT_CRITICAL(&s_state_lock);
    return was_saver;
}

static void draw_screensaver(void)
{
    const uint16_t background = RGB565(7, 8, 10);
    const uint16_t colors[4] = {
        background, RGB565(81, 6, 21), RGB565(154, 3, 32), RGB565(228, 0, 43),
    };
    uint16_t bg = (uint16_t)((background << 8) | (background >> 8));
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) s_framebuffer[i] = bg;
    int x0 = 120 - HONDA_EMBLEM_WIDTH / 2, y0 = 115 - HONDA_EMBLEM_HEIGHT / 2;
    for (int row = 0; row < HONDA_EMBLEM_HEIGHT; ++row) {
        for (int col = 0; col < HONDA_EMBLEM_WIDTH; ++col) {
            size_t index = (size_t)row * HONDA_EMBLEM_WIDTH + col;
            uint8_t alpha = (honda_emblem_alpha[index >> 2] >> (6 - (index & 3) * 2)) & 0x03;
            if (!alpha) continue;
            uint16_t c = colors[alpha];
            s_framebuffer[(y0 + row) * LCD_WIDTH + x0 + col] = (uint16_t)((c << 8) | (c >> 8));
        }
    }
}

/* Splits "CAPTION|VALUE|UNIT"; missing parts become empty strings. */
static void split_label(char *label, char **parts, int count)
{
    for (int i = 0; i < count; ++i) parts[i] = "";
    char *cursor = label;
    for (int i = 0; i < count && cursor; ++i) {
        parts[i] = cursor;
        char *bar = strchr(cursor, '|');
        if (bar) *bar++ = '\0';
        cursor = bar;
    }
}

static void build_view(render_key_t *key, knob_view_t *v, char *parts_buffer, char **parts)
{
    controller_state_t *st = &key->state;
    memset(v, 0, sizeof(*v));
    v->linked = key->linked;
    v->recording = st->recording;
    v->level_permille = -1;

    if (key->toast_active) {
        v->screen = KNOB_SCREEN_MESSAGE;
        v->icon = key->toast.icon;
        v->tone = key->toast.tone;
        v->ring = KNOB_RING_FULL;
        v->caption = key->toast.caption[0] ? key->toast.caption : NULL;
        v->name = key->toast.text;
        return;
    }

    memcpy(parts_buffer, st->label, sizeof(st->label));
    split_label(parts_buffer, parts, 3);
    const char *value = parts[1][0] ? parts[1] : "--";

    switch (st->page) {
    case PAGE_HOME: {
        const home_item_t *item = &HOME_ITEMS[key->home_selection];
        v->screen = KNOB_SCREEN_HOME;
        v->icon = item->icon;
        v->title = item->title;
        v->selection = key->home_selection;
        v->count = HOME_ITEM_COUNT;
        v->hint = item->hint;
        if (item->page == PAGE_NIGHT) v->hint = st->night ? "ON - PRESS FOR DAY" : "PRESS TO DIM";
        if (item->page == PAGE_RECORD) {
            v->hint = st->recording ? "RECORDING - PRESS TO STOP" : "PRESS TO START";
            v->hint_accent = st->recording;
        }
        break;
    }
    case PAGE_GAUGE: {
        uint32_t packed = (uint32_t)st->value;
        int permille = (int)(packed & 0xFFFF);
        v->screen = KNOB_SCREEN_READOUT;
        v->ring = KNOB_RING_LEVEL;
        v->tone = (knob_tone_t)((packed >> 16) & 0x03);
        v->level_permille = permille == 0xFFFF ? -1 : permille;
        v->caption = parts[0][0] ? parts[0] : "GAUGE";
        v->value = value;
        v->unit = parts[2];
        v->show_dots = true;
        v->selection = st->selection;
        v->count = st->count;
        break;
    }
    case PAGE_BRIGHTNESS: {
        snprintf(parts_buffer, sizeof(st->label), "%d", (int)st->value);
        v->screen = KNOB_SCREEN_LEVEL;
        v->ring = KNOB_RING_LEVEL;
        v->level_permille = (int)st->value * 10;
        v->caption = "BRIGHTNESS";
        v->value = st->count ? parts_buffer : "--";
        v->unit = "PERCENT";
        v->hint = "PRESS TO SAVE";
        break;
    }
    case PAGE_TRIP:
    case PAGE_PEAKS:
        v->screen = KNOB_SCREEN_READOUT;
        v->ring = KNOB_RING_POSITION;
        v->selection = st->selection;
        v->count = st->count ? st->count : (st->page == PAGE_TRIP ? 3 : 10);
        v->caption = parts[0][0] ? parts[0] : (st->page == PAGE_TRIP ? "TRIP" : "PEAKS");
        v->value = value;
        v->unit = parts[2];
        if (st->page == PAGE_PEAKS) v->hint = "HOLD TO RESET ALL";
        else v->hint = st->selection > 0 ? "HOLD TO RESET" : "TURN FOR TRIP A / B";
        break;
    case PAGE_TRIP_RESET:
        v->screen = KNOB_SCREEN_CONFIRM;
        v->selection = st->selection;
        v->caption = "RESET";
        v->title = parts[0][0] ? parts[0] : "TRIP";
        v->hint = "PRESS TO CONFIRM";
        break;
    case PAGE_ALERT:
        v->screen = KNOB_SCREEN_MESSAGE;
        v->icon = KNOB_ICON_ALERT;
        v->ring = KNOB_RING_FULL;
        v->tone = KNOB_TONE_CRITICAL;
        v->caption = "WARNING";
        v->name = parts[0];
        v->hint = "PRESS TO DISMISS";
        break;
    case PAGE_VIEW:
    default:
        v->screen = KNOB_SCREEN_CHOICE;
        v->selection = st->selection;
        v->count = st->count;
        v->caption = st->page == PAGE_VIEW ? "DASH THEME" : NULL;
        v->name = parts[0][0] ? parts[0] : "...";
        v->hint = st->page == PAGE_VIEW ? "PRESS TO APPLY" : "PRESS FOR HOME";
        break;
    }
}

static void lcd_transmit(void)
{
    esp_lcd_panel_io_tx_param(s_lcd_io, 0x2a, (uint8_t[]){0, 0, 0, 239}, 4);
    esp_lcd_panel_io_tx_param(s_lcd_io, 0x2b, (uint8_t[]){0, 0, 0, 239}, 4);
    esp_lcd_panel_io_tx_color(s_lcd_io, 0x2c, s_framebuffer,
                              LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t));
    xSemaphoreTake(s_lcd_done, pdMS_TO_TICKS(100));
}

static void set_backlight(int duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void ui_task(void *argument)
{
    (void)argument;
    static render_key_t last;
    static bool have_last;
    int backlight = BACKLIGHT_DAY;
    while (true) {
        render_key_t key;
        memset(&key, 0, sizeof(key));
        int64_t now = esp_timer_get_time();
        portENTER_CRITICAL(&s_state_lock);
        bool linked = link_connected_locked(now);
        if (!linked && s_state.page != PAGE_HOME) {
            s_state.page = PAGE_HOME;     /* dash went away: fall back to the menu */
            s_state.label[0] = '\0';
        }
        if (s_toast_pending != PAGE_HOME && now >= s_toast_deadline) toast_resolve_locked(true);
        key.state = s_state;
        key.home_selection = s_home_selection;
        key.linked = linked;
        key.toast_active = now < s_toast_until;
        if (key.toast_active) key.toast = s_toast;
        key.saver = !key.toast_active && !page_keeps_screen_on(s_state.page) &&
                    now - s_last_activity >= SCREENSAVER_TIMEOUT_US;
        portEXIT_CRITICAL(&s_state_lock);

        int wanted = key.state.night ? BACKLIGHT_NIGHT : BACKLIGHT_DAY;
        if (wanted != backlight) {
            backlight = wanted;
            set_backlight(backlight);
        }

        if (!have_last || memcmp(&key, &last, sizeof(key)) != 0) {
            if (key.saver) {
                draw_screensaver();
            } else {
                knob_view_t view;
                char parts_buffer[sizeof(key.state.label)];
                char *parts[3];
                build_view(&key, &view, parts_buffer, parts);
                knob_ui_render(s_framebuffer, &view);
            }
            lcd_transmit();
            last = key;
            have_last = true;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/* ------------------------------------------------------------ input */

static void encoder_edge_isr(void *argument)
{
    (void)argument;
    uint8_t state = (uint8_t)((gpio_get_level(ENCODER_A_GPIO) << 1) |
                              gpio_get_level(ENCODER_B_GPIO));
    BaseType_t higher_priority_task_woken = pdFALSE;
    xQueueSendFromISR(s_encoder_queue, &state, &higher_priority_task_woken);
    if (higher_priority_task_woken) portYIELD_FROM_ISR();
}

static bool on_home(void)
{
    portENTER_CRITICAL(&s_state_lock);
    bool home = s_state.page == PAGE_HOME;
    portEXIT_CRITICAL(&s_state_lock);
    return home;
}

static void dismiss_toast(void)
{
    portENTER_CRITICAL(&s_state_lock);
    s_toast_until = 0;
    portEXIT_CRITICAL(&s_state_lock);
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
                dismiss_toast();
                portENTER_CRITICAL(&s_state_lock);
                bool home = s_state.page == PAGE_HOME;
                if (home) {
                    s_home_selection = (s_home_selection + direction + HOME_ITEM_COUNT) % HOME_ITEM_COUNT;
                }
                portEXIT_CRITICAL(&s_state_lock);
                if (!home) send_event(ROTARY_ROTATE, direction);
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
            if (!on_home()) send_event(ROTARY_LONG_PRESS, 0);
            long_sent = true;
        } else if (!button_down && button_was_down && !button_woke_screen &&
                   !long_sent && now - button_down_at > 25000 &&
                   now - last_click_at >= 250000) {
            last_click_at = now;
            dismiss_toast();
            if (on_home()) {
                int selected;
                portENTER_CRITICAL(&s_state_lock);
                selected = s_home_selection;
                portEXIT_CRITICAL(&s_state_lock);
                open_home_item(selected);
            } else {
                send_event(ROTARY_PRESS, 0);
            }
        }
        button_was_down = button_down;
        vTaskDelay(1);
    }
}

static bool touch_pressed(void)
{
    uint8_t register_address = 0x01;
    uint8_t data[6] = {0};
    esp_err_t result = i2c_master_write_read_device(I2C_NUM_0, CST816_ADDRESS,
                                                     &register_address, 1, data, sizeof(data),
                                                     pdMS_TO_TICKS(20));
    return result == ESP_OK && (data[1] & 0x0f) != 0;
}

/* Any tap on a page returns to the menu. */
static void touch_task(void *argument)
{
    (void)argument;
    bool was_pressed = false;
    while (true) {
        bool pressed = gpio_get_level(TOUCH_INT_GPIO) == 0 && touch_pressed();
        if (pressed && !was_pressed && !wake_screen()) {
            dismiss_toast();
            if (!on_home()) {
                go_home_locally();
                send_event(ROTARY_BACK, 0);
            }
        }
        was_pressed = pressed;
        vTaskDelay(pdMS_TO_TICKS(25));
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

/* Polls the dash: 10 Hz while the live gauge is shown, 2 Hz otherwise. */
static void communication_task(void *argument)
{
    (void)argument;
    unsigned tick = 0;
    while (true) {
        portENTER_CRITICAL(&s_state_lock);
        bool fast = s_state.page == PAGE_GAUGE;
        portEXIT_CRITICAL(&s_state_lock);
        if (tick % 20 == 0) send_frame(ROTARY_PING, NULL, 0);
        else if (fast || tick % 5 == 0) send_frame(ROTARY_GET_STATE, NULL, 0);
        ++tick;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ------------------------------------------------------------ hardware */

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
        .duty = BACKLIGHT_DAY,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
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
    s_framebuffer = heap_caps_malloc(LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t), MALLOC_CAP_DMA);
    ESP_ERROR_CHECK(s_lcd_done && s_uart_tx_mutex && s_encoder_queue && s_framebuffer ?
                    ESP_OK : ESP_ERR_NO_MEM);
    lcd_init();
    peripherals_init();
    s_last_activity = esp_timer_get_time();
    ESP_LOGI(TAG, "MackoDash rotary controller started");
    xTaskCreate(uart_task, "rotary_uart", 4096, NULL, 6, NULL);
    xTaskCreate(input_task, "rotary_input", 3072, NULL, 5, NULL);
    xTaskCreate(touch_task, "rotary_touch", 3072, NULL, 4, NULL);
    xTaskCreate(communication_task, "rotary_link", 3072, NULL, 4, NULL);
    xTaskCreate(ui_task, "rotary_ui", 6144, NULL, 3, NULL);
}
