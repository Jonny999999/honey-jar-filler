#include "ui_menu.h"

#include <stdio.h>
#include <string.h>

#include "app.h"
#include "buzzer.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "scale_hx711.h"

#define LINE2PIXEL(n) ((n) * 8)
#define MENU_TARE_SAMPLES 16
#define MENU_CAL_SAMPLES  16

static const char *TAG = "ui_menu";

// Menu action entries (add here + handle in ui_menu_on_click).
typedef enum {
    MENU_ACTION_TARE = 0,
    MENU_ACTION_CAL,
    MENU_ACTION_COUNT
} menu_action_t;

// Labels shown in the settings list.
static const char *k_action_labels[MENU_ACTION_COUNT] = {
    "Tare scale",
    "Calibrate scale",
};

// Brief descriptions shown in the settings list (line 6/7 area).
static const char *k_action_brief[MENU_ACTION_COUNT] = {
    "Zero the scale (no weight)",
    "Calibrate with reference weight",
};

// Optional instruction line (line 4) for actions. Use %u for weight if needed.
static const char *k_action_instr_fmt[MENU_ACTION_COUNT] = {
    "Empty scale + Click",
    "Place %u g + Click",
};

static void menu_format_value(const app_param_meta_t *meta,
                              const app_params_t *p,
                              char *out,
                              size_t out_len)
{
    const uint8_t *base = (const uint8_t *)p;
    const uint8_t *ptr = base + meta->offset;

    switch (meta->type) {
    case APP_PARAM_FLOAT:
        snprintf(out, out_len, "%.1f", (double)(*(const float *)ptr));
        break;
    case APP_PARAM_U32:
        snprintf(out, out_len, "%u", (unsigned)(*(const uint32_t *)ptr));
        break;
    case APP_PARAM_U8:
        snprintf(out, out_len, "%u", (unsigned)(*(const uint8_t *)ptr));
        break;
    default:
        snprintf(out, out_len, "?");
        break;
    }
}

static void menu_format_meta_value(const app_param_meta_t *meta,
                                   const app_param_value_t *v,
                                   char *out,
                                   size_t out_len)
{
    switch (meta->type) {
    case APP_PARAM_FLOAT:
        snprintf(out, out_len, "%.1f", (double)v->f);
        break;
    case APP_PARAM_U32:
        snprintf(out, out_len, "%u", (unsigned)v->u32);
        break;
    case APP_PARAM_U8:
        snprintf(out, out_len, "%u", (unsigned)v->u8);
        break;
    default:
        snprintf(out, out_len, "?");
        break;
    }
}

static void menu_apply_delta(const app_param_meta_t *meta,
                             app_params_t *p,
                             int32_t delta)
{
    if (!p || delta == 0) return;
    uint8_t *base = (uint8_t *)p;
    uint8_t *ptr = base + meta->offset;

    switch (meta->type) {
    case APP_PARAM_FLOAT: {
        float v = *(float *)ptr;
        v += (float)delta * meta->step.f;
        if (v < meta->min.f) v = meta->min.f;
        if (v > meta->max.f) v = meta->max.f;
        *(float *)ptr = v;
        break;
    }
    case APP_PARAM_U32: {
        int64_t v = (int64_t)(*(uint32_t *)ptr);
        v += (int64_t)delta * (int64_t)meta->step.u32;
        if (v < (int64_t)meta->min.u32) v = (int64_t)meta->min.u32;
        if (v > (int64_t)meta->max.u32) v = (int64_t)meta->max.u32;
        *(uint32_t *)ptr = (uint32_t)v;
        break;
    }
    case APP_PARAM_U8: {
        int v = (int)(*(uint8_t *)ptr);
        v += delta * (int)meta->step.u8;
        if (v < (int)meta->min.u8) v = (int)meta->min.u8;
        if (v > (int)meta->max.u8) v = (int)meta->max.u8;
        *(uint8_t *)ptr = (uint8_t)v;
        break;
    }
    default:
        break;
    }
}

static void menu_wrap_brief(const char *brief, char *line_a, size_t len_a,
                            char *line_b, size_t len_b)
{
    if (!brief) {
        if (line_a && len_a) line_a[0] = '\0';
        if (line_b && len_b) line_b[0] = '\0';
        return;
    }

    size_t n = strlen(brief);
    size_t first = (n > 20) ? 20 : n;
    snprintf(line_a, len_a, "%.*s", (int)first, brief);
    if (n > first) {
        snprintf(line_b, len_b, "%.*s", (int)(n - first), brief + first);
    } else {
        if (line_b && len_b) line_b[0] = '\0';
    }
}

static const app_param_meta_t *menu_meta_at(size_t index, size_t *count_out)
{
    size_t count = 0;
    const app_param_meta_t *meta = app_params_meta_get(&count);
    if (count_out) *count_out = count;
    if (!meta || count == 0) return NULL;
    if (index >= count) index = count - 1;
    return &meta[index];
}

static bool menu_is_action_index(size_t index, size_t param_count)
{
    return index >= param_count;
}

static size_t menu_total_count(size_t param_count)
{
    return param_count + MENU_ACTION_COUNT;
}

void ui_menu_enter(ui_menu_t *m, const app_params_t *cur)
{
    if (!m || !cur) return;
    m->active = true;
    m->view = UI_MENU_VIEW_LIST;
    m->index = 0;
    m->working = *cur;
    ESP_LOGI(TAG, "enter");
}

void ui_menu_exit(ui_menu_t *m)
{
    if (!m) return;
    m->active = false;
    buzzer_beep_long(1);
    ESP_LOGI(TAG, "exit");
}

bool ui_menu_is_active(const ui_menu_t *m)
{
    return m && m->active;
}

ui_menu_view_t ui_menu_get_view(const ui_menu_t *m)
{
    return m ? m->view : UI_MENU_VIEW_LIST;
}

void ui_menu_on_rotate(ui_menu_t *m, int32_t delta)
{
    if (!m || delta == 0) return;
    size_t count = 0;
    const app_param_meta_t *meta = menu_meta_at(m->index, &count);
    if (!meta || count == 0) return;

    if (m->view == UI_MENU_VIEW_LIST) {
        size_t total = menu_total_count(count);
        int32_t idx = (int32_t)m->index + delta;
        if (idx < 0) idx = (int32_t)total - 1;
        if ((size_t)idx >= total) idx = 0;
        m->index = (size_t)idx;
        buzzer_beep_ms(30);
        ESP_LOGD(TAG, "list: index=%u", (unsigned)m->index);
    } else {
        menu_apply_delta(meta, &m->working, delta);
        buzzer_beep_ms(30);
        ESP_LOGD(TAG, "edit: %s delta=%ld", meta->name, (long)delta);
    }
}

bool ui_menu_on_click(ui_menu_t *m, app_params_t *out_apply)
{
    if (!m) return false;
    size_t param_count = 0;
    (void)app_params_meta_get(&param_count);
    if (m->view == UI_MENU_VIEW_LIST && menu_is_action_index(m->index, param_count)) {
        size_t action_idx = m->index - param_count;
        if (action_idx == MENU_ACTION_TARE) {
            ESP_LOGI(TAG, "action: tare");
            (void)scale_hx711_tare_default(MENU_TARE_SAMPLES);
            buzzer_beep_long(2);
        } else if (action_idx == MENU_ACTION_CAL) {
            ESP_LOGI(TAG, "action: calibrate (%u g)", (unsigned)m->working.scale_cal_ref_g);
            (void)scale_hx711_calibrate_default((float)m->working.scale_cal_ref_g, MENU_CAL_SAMPLES);
            buzzer_beep_long(2);
        }
        return false;
    }

    if (m->view == UI_MENU_VIEW_LIST) {
        m->view = UI_MENU_VIEW_EDIT;
        ESP_LOGI(TAG, "edit: enter");
        return false;
    }

    // Confirm edit: copy working params out for persistence.
    if (out_apply) *out_apply = m->working;
    m->view = UI_MENU_VIEW_LIST;
    buzzer_beep_short(2);
    ESP_LOGI(TAG, "edit: save");
    return true;
}

bool ui_menu_on_long_press(ui_menu_t *m)
{
    if (!m) return false;
    if (m->view == UI_MENU_VIEW_EDIT) {
        m->view = UI_MENU_VIEW_LIST;
        buzzer_beep_long(1);
        ESP_LOGI(TAG, "edit: cancel");
        return true;
    }
    return false;
}

void ui_menu_render(const ui_menu_t *m, ssd1306_handle_t disp)
{
    if (!m) return;

    size_t count = 0;
    const app_param_meta_t *meta = menu_meta_at(m->index, &count);
    if (!meta) return;
    bool is_action = menu_is_action_index(m->index, count);

    char line0[24];
    char line1[24];
    char line2[24];
    char line3[24];
    char line4[24];
    char line5[24];
    char line6[24];

    ssd1306_clear(disp);
    line1[0] = '\0';
    line2[0] = '\0';
    line3[0] = '\0';
    line4[0] = '\0';
    line5[0] = '\0';
    line6[0] = '\0';

    if (m->view == UI_MENU_VIEW_LIST) {
        size_t total = menu_total_count(count);
        snprintf(line0, sizeof(line0), "Settings %u/%u",
                 (unsigned)(m->index + 1), (unsigned)total);
        if (is_action) {
            size_t action_idx = m->index - count;
            char brief_a[24];
            char brief_b[24];
            snprintf(line1, sizeof(line1), "%s", k_action_labels[action_idx]);
            menu_wrap_brief(k_action_brief[action_idx], brief_a, sizeof(brief_a), brief_b, sizeof(brief_b));
            snprintf(line3, sizeof(line3), "%s", brief_a);
            snprintf(line4, sizeof(line4), "%s", brief_b);
            if (action_idx == MENU_ACTION_CAL) {
                snprintf(line2, sizeof(line2), k_action_instr_fmt[action_idx],
                         (unsigned)m->working.scale_cal_ref_g);
            } else {
                snprintf(line2, sizeof(line2), "%s", k_action_instr_fmt[action_idx]);
            }
        } else {
            snprintf(line1, sizeof(line1), "%s", meta->label ? meta->label : meta->name);
            char val[16];
            menu_format_value(meta, &m->working, val, sizeof(val));
            snprintf(line3, sizeof(line3), "Val: %s%s%s",
                     val,
                     (meta->unit && meta->unit[0]) ? " " : "",
                     (meta->unit && meta->unit[0]) ? meta->unit : "");
        }
    } else {
        char val[16];
        char defv[16];
        char brief_a[24];
        char brief_b[24];
        menu_format_value(meta, &m->working, val, sizeof(val));
        snprintf(line0, sizeof(line0), "%s", meta->label ? meta->label : meta->name);
        snprintf(line2, sizeof(line2), "Value: %s%s%s",
                 val,
                 (meta->unit && meta->unit[0]) ? " " : "",
                 (meta->unit && meta->unit[0]) ? meta->unit : "");
        menu_format_meta_value(meta, &meta->def, defv, sizeof(defv));
        if (meta->unit && meta->unit[0]) {
            snprintf(line3, sizeof(line4), "Default: %.11s %.2s", defv, meta->unit);
        } else {
            snprintf(line3, sizeof(line4), "Default: %.14s", defv);
        }
        menu_wrap_brief(meta->desc_brief, brief_a, sizeof(brief_a), brief_b, sizeof(brief_b));
        snprintf(line5, sizeof(line5), "%s", brief_a);
        snprintf(line6, sizeof(line6), "%s", brief_b);
    }

    static int64_t last_blink_us = 0;
    static bool blink_on = true;
    int64_t now_us = esp_timer_get_time();
    if (m->view == UI_MENU_VIEW_EDIT) {
        if (now_us - last_blink_us >= 400000) {
            blink_on = !blink_on;
            last_blink_us = now_us;
        }
        if (line2[0]) {
            char tmp[24];
            snprintf(tmp, sizeof(tmp), "%c %.21s", blink_on ? '>' : ' ', line2);
            snprintf(line2, sizeof(line2), "%s", tmp);
        }
    }

    ssd1306_draw_text(disp, 0, LINE2PIXEL(0), line0, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(2), line1, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(3), line2, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(4), line3, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(5), line4, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(6), line5, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(7), line6, true);
    ssd1306_display(disp);
}
