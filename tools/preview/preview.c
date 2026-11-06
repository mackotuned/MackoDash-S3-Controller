/* PC preview of the knob UI: renders sample screens to <out_dir>/NN.raw
 * (240x240 byte-swapped RGB565, exactly what the panel receives).
 * Build: tools/preview/build_preview.ps1 */
#include <stdio.h>
#include <string.h>

#include "knob_ui.h"

static uint16_t fb[KNOB_UI_SIZE * KNOB_UI_SIZE];

static void save(const char *dir, int index, const knob_view_t *view)
{
    char path[512];
    knob_ui_render(fb, view);
    snprintf(path, sizeof(path), "%s/%02d.raw", dir, index);
    FILE *f = fopen(path, "wb");
    if (!f) return;
    fwrite(fb, sizeof(fb), 1, f);
    fclose(f);
}

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : ".";
    int n = 0;
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_HOME, .icon = KNOB_ICON_GAUGE,
        .selection = 0, .count = 7, .title = "GAUGE", .hint = "LIVE READOUT", .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_HOME, .icon = KNOB_ICON_VIEW,
        .selection = 1, .count = 7, .title = "VIEW", .hint = "DASH THEME", .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_HOME, .icon = KNOB_ICON_BRIGHT,
        .selection = 2, .count = 7, .title = "BRIGHT", .hint = "SCREEN LEVEL", .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_HOME, .icon = KNOB_ICON_NIGHT,
        .selection = 3, .count = 7, .title = "NIGHT", .hint = "PRESS TO DIM", .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_HOME, .icon = KNOB_ICON_REC,
        .selection = 4, .count = 7, .title = "REC", .hint = "RECORDING", .hint_accent = true,
        .recording = true, .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_HOME, .icon = KNOB_ICON_TRIP,
        .selection = 5, .count = 7, .title = "TRIP", .hint = "ODO / TRIP A / B", .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_HOME, .icon = KNOB_ICON_PEAKS,
        .selection = 6, .count = 7, .title = "PEAKS", .hint = "SESSION MAX", .linked = false});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_READOUT, .ring = KNOB_RING_LEVEL,
        .level_permille = 640, .caption = "BOOST", .value = "12.4", .unit = "PSI",
        .show_dots = true, .selection = 0, .count = 11, .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_READOUT, .ring = KNOB_RING_LEVEL,
        .tone = KNOB_TONE_CRITICAL, .level_permille = 760, .caption = "COOLANT", .value = "228",
        .unit = "\xC2\xB0" "F", .show_dots = true, .selection = 1, .count = 11, .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_LEVEL, .ring = KNOB_RING_LEVEL,
        .level_permille = 750, .caption = "BRIGHTNESS", .value = "75", .unit = "PERCENT",
        .hint = "PRESS TO SAVE", .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_CHOICE, .selection = 1, .count = 6,
        .caption = "DASH THEME", .name = "MACKODASH V2", .hint = "PRESS TO APPLY", .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_READOUT, .selection = 1, .count = 3,
        .caption = "TRIP A", .value = "1284.6", .unit = "MILES", .hint = "HOLD TO RESET", .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_READOUT, .selection = 2, .count = 10,
        .caption = "MAX BOOST", .value = "18.7", .unit = "PSI", .hint = "HOLD TO RESET ALL", .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_CONFIRM, .selection = 1, .count = 2,
        .caption = "RESET", .title = "TRIP A", .hint = "PRESS TO CONFIRM", .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_MESSAGE, .icon = KNOB_ICON_ALERT,
        .ring = KNOB_RING_FULL, .tone = KNOB_TONE_CRITICAL, .caption = "WARNING",
        .name = "OIL PRESSURE LOW", .hint = "PRESS TO DISMISS", .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_MESSAGE, .icon = KNOB_ICON_NIGHT,
        .ring = KNOB_RING_FULL, .name = "NIGHT MODE ON", .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_MESSAGE, .icon = KNOB_ICON_REC,
        .ring = KNOB_RING_FULL, .tone = KNOB_TONE_CRITICAL, .name = "LOGGING STARTED",
        .recording = true, .linked = true});
    save(dir, n++, &(knob_view_t){.screen = KNOB_SCREEN_MESSAGE, .icon = KNOB_ICON_LINK,
        .caption = "NO DASH LINK", .name = "CHECK KNOB CABLE", .linked = false});
    printf("%d screens\n", n);
    return 0;
}
