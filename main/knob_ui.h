#pragma once

#include <stdbool.h>
#include <stdint.h>

#define KNOB_UI_SIZE 240

typedef enum {
    KNOB_SCREEN_HOME = 0,   /* menu: icon + title + status line           */
    KNOB_SCREEN_READOUT,    /* caption + big value + unit (gauge/trip/peak) */
    KNOB_SCREEN_LEVEL,      /* caption + percentage + progress ring        */
    KNOB_SCREEN_CHOICE,     /* caption + name, ring shows list position    */
    KNOB_SCREEN_CONFIRM,    /* question + CANCEL / RESET                   */
    KNOB_SCREEN_MESSAGE,    /* icon + message (alert, offline, toast)      */
} knob_screen_t;

typedef enum {
    KNOB_ICON_NONE = 0,
    KNOB_ICON_GAUGE,
    KNOB_ICON_VIEW,
    KNOB_ICON_BRIGHT,
    KNOB_ICON_NIGHT,
    KNOB_ICON_REC,
    KNOB_ICON_TRIP,
    KNOB_ICON_PEAKS,
    KNOB_ICON_ALERT,
    KNOB_ICON_LINK,
} knob_icon_t;

typedef enum {
    KNOB_TONE_NORMAL = 0,
    KNOB_TONE_WARN,
    KNOB_TONE_CRITICAL,
} knob_tone_t;

/* Ring modes for READOUT screens */
typedef enum {
    KNOB_RING_POSITION = 0,  /* segment per list entry, selected one red      */
    KNOB_RING_LEVEL,         /* progress fill from level_permille              */
    KNOB_RING_FULL,          /* whole ring in tone colour                      */
} knob_ring_t;

typedef struct {
    knob_screen_t screen;
    knob_icon_t icon;
    knob_ring_t ring;
    knob_tone_t tone;
    int selection;           /* list position (ring segments / dots)          */
    int count;
    int level_permille;      /* 0..1000, <0 = no level                        */
    const char *caption;     /* small label above the main content            */
    const char *title;       /* HOME title, CONFIRM question                  */
    const char *value;       /* READOUT/LEVEL big number                      */
    const char *unit;
    const char *name;        /* CHOICE / MESSAGE text (wraps to two lines)    */
    const char *hint;        /* bottom instruction line                       */
    bool hint_accent;        /* draw hint in red instead of grey              */
    bool show_dots;          /* READOUT: list position dots under the value   */
    bool recording;          /* REC badge at the top                          */
    bool linked;             /* status dot green/grey                         */
} knob_view_t;

/* Renders a full frame into fb (240x240 RGB565, byte-swapped for the panel). */
void knob_ui_render(uint16_t *fb, const knob_view_t *view);

/* Draws an icon into an existing frame (used by the screensaver too). */
void knob_ui_draw_icon(uint16_t *fb, knob_icon_t icon, int cx, int cy, bool active);
