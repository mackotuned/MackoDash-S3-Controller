/* MackoDash knob UI renderer: anti-aliased software drawing into a 240x240
 * RGB565 frame. No hardware dependencies, so it also builds on a PC for
 * previews (tools/preview). */
#include "knob_ui.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "knob_fonts.h"

#define W KNOB_UI_SIZE
#define CX 120.0f
#define CY 120.0f
#define PI_F 3.14159265f

#define RGB(r, g, b) ((uint16_t)((((r) & 0xf8) << 8) | (((g) & 0xfc) << 3) | ((b) >> 3)))

#define C_BG      RGB(7, 8, 10)
#define C_TRACK   RGB(30, 32, 36)
#define C_TRACK2  RGB(44, 47, 51)
#define C_TEXT    RGB(244, 244, 240)
#define C_DIM     RGB(126, 130, 134)
#define C_FAINT   RGB(70, 73, 77)
#define C_RED     RGB(228, 0, 43)
#define C_AMBER   RGB(255, 176, 32)
#define C_GREEN   RGB(57, 255, 140)

/* Ring geometry: 270 degree sweep with the opening at the bottom. */
#define RING_OUT   117.0f
#define RING_IN    109.0f
#define RING_START 135.0f
#define RING_SWEEP 270.0f

static uint16_t *s_fb;

static inline uint16_t swap16(uint16_t v) { return (uint16_t)((v << 8) | (v >> 8)); }

static inline float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

static void blend(int x, int y, uint16_t color, int alpha)
{
    if ((unsigned)x >= W || (unsigned)y >= W || alpha <= 0) return;
    uint16_t *p = &s_fb[y * W + x];
    if (alpha >= 255) { *p = swap16(color); return; }
    uint16_t bg = swap16(*p);
    int inv = 255 - alpha;
    int r = (((color >> 11) & 0x1f) * alpha + ((bg >> 11) & 0x1f) * inv + 127) / 255;
    int g = (((color >> 5) & 0x3f) * alpha + ((bg >> 5) & 0x3f) * inv + 127) / 255;
    int b = ((color & 0x1f) * alpha + (bg & 0x1f) * inv + 127) / 255;
    *p = swap16((uint16_t)((r << 11) | (g << 5) | b));
}

static void fill(uint16_t color)
{
    uint16_t v = swap16(color);
    for (int i = 0; i < W * W; ++i) s_fb[i] = v;
}

/* ---------------------------------------------------------------- shapes */

typedef struct {
    float a0, a1;      /* degrees, clockwise from 3 o'clock, a1 > a0 */
    uint16_t color;
} arc_seg_t;

/* Anti-aliased annulus split into angular segments. */
static void ring_segments(float cx, float cy, float r_out, float r_in,
                          const arc_seg_t *segs, int count)
{
    int y0 = (int)(cy - r_out - 1), y1 = (int)(cy + r_out + 2);
    int x0 = (int)(cx - r_out - 1), x1 = (int)(cx + r_out + 2);
    float outer2 = (r_out + 1.0f) * (r_out + 1.0f);
    float inner2 = r_in > 1.0f ? (r_in - 1.0f) * (r_in - 1.0f) : 0.0f;
    for (int y = y0; y < y1; ++y) {
        float dy = y + 0.5f - cy;
        for (int x = x0; x < x1; ++x) {
            float dx = x + 0.5f - cx;
            float d2 = dx * dx + dy * dy;
            if (d2 > outer2 || d2 < inner2) continue;
            float d = sqrtf(d2);
            float radial = clamp01(fminf(r_out - d, d - r_in) + 0.5f);
            if (radial <= 0.0f) continue;
            float angle = atan2f(dy, dx) * (180.0f / PI_F);
            if (angle < 0.0f) angle += 360.0f;
            float px_per_deg = d * PI_F / 180.0f;
            for (int i = 0; i < count; ++i) {
                float len = segs[i].a1 - segs[i].a0;
                if (len <= 0.0f) continue;
                float rel = fmodf(angle - segs[i].a0 + 720.0f, 360.0f);
                float cov;
                if (rel <= len) cov = clamp01(fminf(rel, len - rel) * px_per_deg + 0.5f);
                else cov = clamp01(0.5f - fminf(rel - len, 360.0f - rel) * px_per_deg);
                if (cov > 0.0f) blend(x, y, segs[i].color, (int)(radial * cov * 255.0f + 0.5f));
            }
        }
    }
}

static void arc(float cx, float cy, float r, float width, float a0, float a1, uint16_t color)
{
    arc_seg_t seg = {a0, a1, color};
    ring_segments(cx, cy, r + width / 2.0f, r - width / 2.0f, &seg, 1);
}

static void disc(float cx, float cy, float r, uint16_t color)
{
    for (int y = (int)(cy - r - 1); y <= (int)(cy + r + 1); ++y) {
        for (int x = (int)(cx - r - 1); x <= (int)(cx + r + 1); ++x) {
            float dx = x + 0.5f - cx, dy = y + 0.5f - cy;
            float cov = clamp01(r - sqrtf(dx * dx + dy * dy) + 0.5f);
            if (cov > 0.0f) blend(x, y, color, (int)(cov * 255.0f + 0.5f));
        }
    }
}

/* Thick anti-aliased line with round caps. */
static void stroke(float ax, float ay, float bx, float by, float width, uint16_t color)
{
    float h = width / 2.0f;
    int x0 = (int)(fminf(ax, bx) - h - 1), x1 = (int)(fmaxf(ax, bx) + h + 2);
    int y0 = (int)(fminf(ay, by) - h - 1), y1 = (int)(fmaxf(ay, by) + h + 2);
    float vx = bx - ax, vy = by - ay;
    float len2 = vx * vx + vy * vy;
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            float px = x + 0.5f - ax, py = y + 0.5f - ay;
            float t = len2 > 0.0f ? clamp01((px * vx + py * vy) / len2) : 0.0f;
            float ex = px - vx * t, ey = py - vy * t;
            float cov = clamp01(h - sqrtf(ex * ex + ey * ey) + 0.5f);
            if (cov > 0.0f) blend(x, y, color, (int)(cov * 255.0f + 0.5f));
        }
    }
}

static void rect(int x, int y, int w, int h, uint16_t color)
{
    for (int row = y; row < y + h; ++row)
        for (int col = x; col < x + w; ++col) blend(col, row, color, 255);
}

/* ------------------------------------------------------------------ text */

static int glyph_code(const char **text)
{
    unsigned char c = (unsigned char)**text;
    if (c == 0xC2 && (unsigned char)(*text)[1] == 0xB0) {  /* UTF-8 degree sign */
        *text += 2;
        return KFONT_DEGREE;
    }
    *text += 1;
    if (c >= 'a' && c <= 'z') c = (unsigned char)(c - 'a' + 'A');
    if (c < KFONT_FIRST || c > KFONT_LAST) return '?';
    return c;
}

static const kfont_glyph_t *glyph(const kfont_t *font, int code)
{
    const kfont_glyph_t *g = &font->glyphs[code - KFONT_FIRST];
    if (!g->adv && code != ' ') g = &font->glyphs['?' - KFONT_FIRST];
    return g;
}

static int text_width(const kfont_t *font, const char *text, int spacing)
{
    int width = 0, n = 0;
    while (text && *text) {
        width += glyph(font, glyph_code(&text))->adv + spacing;
        ++n;
    }
    return n ? width - spacing : 0;
}

static void text_at(const kfont_t *font, int x, int baseline, const char *text,
                    int spacing, uint16_t color)
{
    while (text && *text) {
        const kfont_glyph_t *g = glyph(font, glyph_code(&text));
        const uint8_t *bits = font->bitmap + g->offset;
        for (int row = 0; row < g->h; ++row) {
            for (int col = 0; col < g->w; ++col) {
                int i = row * g->w + col;
                int a = (i & 1) ? (bits[i >> 1] & 0x0f) : (bits[i >> 1] >> 4);
                if (a) blend(x + g->x + col, baseline + g->y + row, color, a * 17);
            }
        }
        x += g->adv + spacing;
    }
}

static void text_center(const kfont_t *font, int baseline, const char *text,
                        int spacing, uint16_t color)
{
    text_at(font, (W - text_width(font, text, spacing)) / 2, baseline, text, spacing, color);
}

/* Centres text on one line, or splits it at the space nearest the middle. */
static void text_wrap_center(const kfont_t *font, int baseline, int max_width,
                             const char *text, uint16_t color)
{
    if (!text) return;
    if (text_width(font, text, 0) <= max_width) {
        text_center(font, baseline, text, 0, color);
        return;
    }
    char buffer[64];
    strncpy(buffer, text, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    int len = (int)strlen(buffer), split = -1;
    for (int i = 0; i < len; ++i) {
        if (buffer[i] == ' ' && (split < 0 || abs(i - len / 2) < abs(split - len / 2))) split = i;
    }
    if (split < 0) {
        text_center(font, baseline, buffer, 0, color);
        return;
    }
    buffer[split] = '\0';
    int line = font->line_height - 2;
    text_center(font, baseline - line / 2, buffer, 0, color);
    text_center(font, baseline + (line + 1) / 2, buffer + split + 1, 0, color);
}

/* ----------------------------------------------------------------- icons */

void knob_ui_draw_icon(uint16_t *fb, knob_icon_t icon, int icx, int icy, bool active)
{
    s_fb = fb;
    float cx = (float)icx, cy = (float)icy;
    uint16_t main = active ? C_TEXT : C_DIM;
    uint16_t accent = C_RED;
    switch (icon) {
    case KNOB_ICON_GAUGE:
        arc(cx, cy + 8, 24, 5, 180, 292, main);
        arc(cx, cy + 8, 24, 5, 300, 360, accent);
        stroke(cx, cy + 8, cx + 12, cy - 9, 4, main);
        disc(cx, cy + 8, 5, accent);
        break;
    case KNOB_ICON_VIEW:
        stroke(cx - 25, cy - 17, cx + 25, cy - 17, 4, main);
        stroke(cx - 25, cy + 17, cx + 25, cy + 17, 4, main);
        stroke(cx - 25, cy - 17, cx - 25, cy + 17, 4, main);
        stroke(cx + 25, cy - 17, cx + 25, cy + 17, 4, main);
        stroke(cx - 14, cy - 6, cx + 14, cy - 6, 4, accent);
        stroke(cx - 14, cy + 5, cx - 4, cy + 5, 4, main);
        stroke(cx + 4, cy + 5, cx + 14, cy + 5, 4, main);
        break;
    case KNOB_ICON_BRIGHT:
        disc(cx, cy, 10, accent);
        for (int i = 0; i < 8; ++i) {
            float a = i * PI_F / 4.0f;
            stroke(cx + cosf(a) * 17, cy + sinf(a) * 17, cx + cosf(a) * 24, cy + sinf(a) * 24, 4, main);
        }
        break;
    case KNOB_ICON_NIGHT:
        disc(cx - 2, cy + 1, 20, main);
        disc(cx + 9, cy - 7, 17, C_BG);
        disc(cx + 17, cy + 12, 3, accent);
        disc(cx + 8, cy + 21, 2, accent);
        break;
    case KNOB_ICON_REC:
        arc(cx, cy, 21, 4, 0, 360, main);
        disc(cx, cy, 11, accent);
        break;
    case KNOB_ICON_TRIP:
        stroke(cx - 14, cy - 24, cx - 14, cy + 24, 4, main);
        stroke(cx - 14, cy - 22, cx + 20, cy - 12, 4, accent);
        stroke(cx + 20, cy - 12, cx - 14, cy - 1, 4, accent);
        stroke(cx - 22, cy + 24, cx - 6, cy + 24, 4, main);
        break;
    case KNOB_ICON_PEAKS:
        stroke(cx - 26, cy + 22, cx + 26, cy + 22, 3, C_FAINT);
        stroke(cx - 24, cy + 14, cx - 9, cy - 2, 4, main);
        stroke(cx - 9, cy - 2, cx + 2, cy + 8, 4, main);
        stroke(cx + 2, cy + 8, cx + 18, cy - 18, 4, main);
        disc(cx + 18, cy - 18, 6, accent);
        break;
    case KNOB_ICON_ALERT:
        stroke(cx, cy - 24, cx - 26, cy + 20, 5, accent);
        stroke(cx, cy - 24, cx + 26, cy + 20, 5, accent);
        stroke(cx - 26, cy + 20, cx + 26, cy + 20, 5, accent);
        stroke(cx, cy - 8, cx, cy + 6, 5, main);
        disc(cx, cy + 13, 3, main);
        break;
    case KNOB_ICON_LINK:
        arc(cx - 10, cy, 13, 4, 30, 330, main);
        arc(cx + 10, cy, 13, 4, 210, 510, main);
        stroke(cx - 22, cy + 22, cx + 22, cy - 22, 4, accent);
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------ components */

static uint16_t tone_color(knob_tone_t tone)
{
    return tone == KNOB_TONE_CRITICAL ? C_RED : (tone == KNOB_TONE_WARN ? C_AMBER : C_TEXT);
}

static void ring_position(int selection, int count, uint16_t active)
{
    arc_seg_t segs[24];
    if (count < 1) count = 1;
    if (count > 24) {
        /* too many entries for segments: track + a single marker */
        float frac = count > 1 ? (float)selection / (float)(count - 1) : 0.0f;
        float a = RING_START + frac * (RING_SWEEP - 10.0f);
        segs[0] = (arc_seg_t){RING_START, RING_START + RING_SWEEP, C_TRACK};
        segs[1] = (arc_seg_t){a, a + 10.0f, active};
        ring_segments(CX, CY, RING_OUT, RING_IN, &segs[0], 1);
        ring_segments(CX, CY, RING_OUT, RING_IN, &segs[1], 1);
        return;
    }
    float gap = count > 12 ? 2.0f : 4.0f;
    float step = (RING_SWEEP + gap) / count;
    for (int i = 0; i < count; ++i) {
        segs[i].a0 = RING_START + i * step;
        segs[i].a1 = segs[i].a0 + step - gap;
        segs[i].color = i == selection ? active : C_TRACK2;
    }
    ring_segments(CX, CY, RING_OUT, RING_IN, segs, count);
}

static void ring_level(int permille, uint16_t color)
{
    if (permille < 0) permille = 0;
    if (permille > 1000) permille = 1000;
    float end = RING_START + RING_SWEEP * permille / 1000.0f;
    arc_seg_t segs[2] = {
        {RING_START, end, color},
        {end + 3.0f, RING_START + RING_SWEEP, C_TRACK},
    };
    ring_segments(CX, CY, RING_OUT, RING_IN, segs, permille >= 1000 ? 1 : 2);
}

static void dots(int selection, int count, int y)
{
    if (count < 2) return;
    if (count > 16) count = 16;
    int pitch = 10, x = 120 - (count - 1) * pitch / 2;
    for (int i = 0; i < count; ++i, x += pitch) {
        if (i == selection) disc((float)x, (float)y, 3.2f, C_RED);
        else disc((float)x, (float)y, 2.2f, C_TRACK2);
    }
}

static void caption(const char *text, int baseline, uint16_t color)
{
    if (text) text_center(&kf_label, baseline, text, 2, color);
}

/* Big number: shrinks to the title font when it would not fit. */
static void big_value(const char *value, int baseline, uint16_t color)
{
    if (!value) return;
    if (text_width(&kf_value, value, 0) <= 176) {
        text_center(&kf_value, baseline, value, 0, color);
    } else if (text_width(&kf_value_m, value, 0) <= 184) {
        text_center(&kf_value_m, baseline - 6, value, 0, color);
    } else {
        text_center(&kf_title, baseline - 14, value, 0, color);
    }
}

static void status(const knob_view_t *v)
{
    disc(120, 226, 3, v->linked ? C_GREEN : C_FAINT);
    if (v->recording) {
        int w = text_width(&kf_small, "REC", 1);
        int x = 120 - (w + 10) / 2;
        disc((float)x + 3.5f, 28.5f, 3.5f, C_RED);
        text_at(&kf_small, x + 10, 33, "REC", 1, C_RED);
    }
}

/* ---------------------------------------------------------------- screens */

static void screen_home(const knob_view_t *v)
{
    ring_position(v->selection, v->count, C_RED);
    knob_ui_draw_icon(s_fb, v->icon, 120, 84, true);
    if (v->title) text_center(&kf_title, 152, v->title, 1, C_TEXT);
    if (v->hint) text_center(&kf_small, 176, v->hint, 1, v->hint_accent ? C_RED : C_DIM);
}

static void screen_readout(const knob_view_t *v)
{
    uint16_t color = tone_color(v->tone);
    if (v->ring == KNOB_RING_LEVEL) {
        if (v->level_permille >= 0) ring_level(v->level_permille, v->tone == KNOB_TONE_NORMAL ? C_RED : color);
        else ring_level(0, C_TRACK);
    } else if (v->ring == KNOB_RING_FULL) {
        ring_level(1000, v->tone == KNOB_TONE_NORMAL ? C_RED : color);
    } else {
        ring_position(v->selection, v->count, C_RED);
    }
    caption(v->caption, 68, C_DIM);
    big_value(v->value, 139, color);
    if (v->unit) text_center(&kf_label, 166, v->unit, 2, C_DIM);
    if (v->show_dots) dots(v->selection, v->count, 190);
    if (v->hint) text_center(&kf_small, v->show_dots ? 208 : 196, v->hint, 1,
                             v->hint_accent ? C_RED : C_FAINT);
}

static void screen_choice(const knob_view_t *v)
{
    ring_position(v->selection, v->count, C_RED);
    caption(v->caption, 68, C_DIM);
    text_wrap_center(&kf_name, 124, 176, v->name, C_TEXT);
    if (v->count > 1) {
        char pos[16];
        int a = v->selection + 1, b = v->count;
        pos[0] = (char)('0' + (a / 10) % 10); pos[1] = (char)('0' + a % 10);
        pos[2] = ' '; pos[3] = '/'; pos[4] = ' ';
        pos[5] = (char)('0' + (b / 10) % 10); pos[6] = (char)('0' + b % 10); pos[7] = '\0';
        text_center(&kf_small, 162, pos, 1, C_RED);
    }
    if (v->hint) text_center(&kf_small, 196, v->hint, 1, v->hint_accent ? C_RED : C_FAINT);
}

static void screen_confirm(const knob_view_t *v)
{
    ring_level(1000, C_RED);
    caption(v->caption, 68, C_DIM);
    if (v->title) text_center(&kf_title, 108, v->title, 1, C_TEXT);
    static const char *const options[2] = {"CANCEL", "RESET"};
    for (int i = 0; i < 2; ++i) {
        bool sel = v->selection == i;
        int w = text_width(&kf_name, options[i], 1);
        int x = i == 0 ? 78 - w / 2 : 162 - w / 2;
        text_at(&kf_name, x, 150, options[i], 1, sel ? (i ? C_RED : C_TEXT) : C_FAINT);
        if (sel) rect(x, 157, w, 3, i ? C_RED : C_TEXT);
    }
    if (v->hint) text_center(&kf_small, 196, v->hint, 1, C_FAINT);
}

static void screen_message(const knob_view_t *v)
{
    if (v->ring == KNOB_RING_FULL) ring_level(1000, v->tone == KNOB_TONE_NORMAL ? C_TEXT : tone_color(v->tone));
    else ring_level(0, C_TRACK);
    knob_ui_draw_icon(s_fb, v->icon, 120, 76, true);
    caption(v->caption, 126, v->tone == KNOB_TONE_NORMAL ? C_DIM : tone_color(v->tone));
    text_wrap_center(&kf_name, v->caption ? 158 : 142, 184, v->name, C_TEXT);
    if (v->hint) text_center(&kf_small, 196, v->hint, 1, v->hint_accent ? C_RED : C_FAINT);
}

void knob_ui_render(uint16_t *fb, const knob_view_t *v)
{
    s_fb = fb;
    fill(C_BG);
    switch (v->screen) {
    case KNOB_SCREEN_HOME: screen_home(v); break;
    case KNOB_SCREEN_READOUT:
    case KNOB_SCREEN_LEVEL: screen_readout(v); break;
    case KNOB_SCREEN_CHOICE: screen_choice(v); break;
    case KNOB_SCREEN_CONFIRM: screen_confirm(v); break;
    case KNOB_SCREEN_MESSAGE: screen_message(v); break;
    }
    s_fb = fb;
    status(v);
}
