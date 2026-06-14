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

typedef enum {
    MENU_HOME_PRESET = 0,
    MENU_HOME_STRATEGY,
    MENU_HOME_RECIPE,
    MENU_HOME_MACHINE,
    MENU_HOME_TARE,
    MENU_HOME_CAL,
    MENU_HOME_COUNT
} menu_home_item_t;

static const char *k_home_labels[MENU_HOME_COUNT] = {
    "Select preset",
    "Select strategy",
    "Preset settings",
    "Machine settings",
    "Tare scale",
    "Calibrate scale",
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

static void menu_wrap_brief(const char *brief,
                            char *line_a,
                            size_t len_a,
                            char *line_b,
                            size_t len_b)
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
    } else if (line_b && len_b) {
        line_b[0] = '\0';
    }
}

static void menu_render_home_detail(const ui_menu_t *m,
                                    char *line4,
                                    size_t line4_len,
                                    char *line5,
                                    size_t line5_len,
                                    char *line6,
                                    size_t line6_len)
{
    menu_home_item_t item = (menu_home_item_t)m->index;
    if (item >= MENU_HOME_COUNT) return;

    switch (item) {
    case MENU_HOME_PRESET:
        snprintf(line4, line4_len, "Target + flow preset");
        snprintf(line5, line5_len, "Click: preset list");
        break;
    case MENU_HOME_STRATEGY:
        snprintf(line4, line4_len, "Fill control mode");
        snprintf(line5, line5_len, "Click: mode list");
        break;
    case MENU_HOME_RECIPE:
        snprintf(line4, line4_len, "Target, tol, honey");
        snprintf(line5, line5_len, "Active preset only");
        break;
    case MENU_HOME_MACHINE:
        snprintf(line4, line4_len, "Scale, servo, slots");
        snprintf(line5, line5_len, "Shared by all presets");
        break;
    case MENU_HOME_TARE:
        snprintf(line4, line4_len, "Empty scale + Click");
        snprintf(line5, line5_len, "No jar / no weight");
        break;
    case MENU_HOME_CAL:
        snprintf(line4, line4_len, "Place %u g + Click",
                 (unsigned)m->working.scale_cal_ref_g);
        snprintf(line5, line5_len, "Ref wt: %u g",
                 (unsigned)m->working.scale_cal_ref_g);
        break;
    default:
        break;
    }
    (void)line6;
    (void)line6_len;
}

static const app_param_meta_t *menu_meta_by_scope(app_param_scope_t scope,
                                                  size_t filtered_index,
                                                  size_t *out_count)
{
    size_t total = 0;
    size_t seen = 0;
    const app_param_meta_t *meta = app_params_meta_get(&total);
    if (!meta || total == 0) return NULL;

    const app_param_meta_t *last = NULL;
    for (size_t i = 0; i < total; ++i) {
        if (meta[i].scope != scope) continue;
        last = &meta[i];
        if (seen == filtered_index) {
            if (out_count) *out_count = 0;
            for (size_t j = 0; j < total; ++j) {
                if (meta[j].scope == scope) (*out_count)++;
            }
            return last;
        }
        seen++;
    }

    if (out_count) *out_count = seen;
    return last;
}

void ui_menu_enter(ui_menu_t *m, const app_params_t *cur)
{
    if (!m || !cur) return;
    m->active = true;
    m->view = UI_MENU_VIEW_HOME;
    m->index = 0;
    m->scope = APP_PARAM_SCOPE_PRESET;
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
    return m ? m->view : UI_MENU_VIEW_HOME;
}

void ui_menu_on_rotate(ui_menu_t *m, int32_t delta)
{
    if (!m || delta == 0) return;

    if (m->view == UI_MENU_VIEW_HOME) {
        int32_t idx = (int32_t)m->index + delta;
        if (idx < 0) idx = MENU_HOME_COUNT - 1;
        if (idx >= MENU_HOME_COUNT) idx = 0;
        m->index = (size_t)idx;
        buzzer_beep_ms(30);
        return;
    }

    if (m->view == UI_MENU_VIEW_PRESET_LIST) {
        int32_t idx = (int32_t)m->index + delta;
        if (idx < 0) idx = (int32_t)app_presets_count() - 1;
        if ((size_t)idx >= app_presets_count()) idx = 0;
        m->index = (size_t)idx;
        buzzer_beep_ms(30);
        return;
    }

    if (m->view == UI_MENU_VIEW_STRATEGY_LIST) {
        int32_t idx = (int32_t)m->index + delta;
        if (idx < 0) idx = (int32_t)app_fill_strategy_count() - 1;
        if ((size_t)idx >= app_fill_strategy_count()) idx = 0;
        m->index = (size_t)idx;
        buzzer_beep_ms(30);
        return;
    }

    size_t count = 0;
    const app_param_meta_t *meta = menu_meta_by_scope(m->scope, m->index, &count);
    if (!meta || count == 0) return;

    if (m->view == UI_MENU_VIEW_PARAM_LIST) {
        int32_t idx = (int32_t)m->index + delta;
        if (idx < 0) idx = (int32_t)count - 1;
        if ((size_t)idx >= count) idx = 0;
        m->index = (size_t)idx;
        buzzer_beep_ms(30);
        return;
    }

    if (m->view == UI_MENU_VIEW_PARAM_EDIT) {
        menu_apply_delta(meta, &m->working, delta);
        buzzer_beep_ms(30);
    }
}

bool ui_menu_on_click(ui_menu_t *m, app_params_t *out_apply)
{
    if (!m) return false;

    if (m->view == UI_MENU_VIEW_HOME) {
        switch ((menu_home_item_t)m->index) {
        case MENU_HOME_PRESET:
            m->view = UI_MENU_VIEW_PRESET_LIST;
            m->index = app_presets_get_active_index();
            return false;
        case MENU_HOME_STRATEGY:
            m->view = UI_MENU_VIEW_STRATEGY_LIST;
            m->index = (size_t)app_fill_strategy_get_active();
            return false;
        case MENU_HOME_RECIPE:
            m->view = UI_MENU_VIEW_PARAM_LIST;
            m->scope = APP_PARAM_SCOPE_PRESET;
            m->index = 0;
            return false;
        case MENU_HOME_MACHINE:
            m->view = UI_MENU_VIEW_PARAM_LIST;
            m->scope = APP_PARAM_SCOPE_MACHINE;
            m->index = 0;
            return false;
        case MENU_HOME_TARE:
            ESP_LOGI(TAG, "action: tare");
            {
                esp_err_t err = scale_hx711_tare_default(MENU_TARE_SAMPLES);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "action: tare ok");
                    buzzer_beep_long(2);
                } else {
                    ESP_LOGW(TAG, "action: tare failed: %s", esp_err_to_name(err));
                    buzzer_beep_long(1);
                }
            }
            app_params_get(&m->working);
            return false;
        case MENU_HOME_CAL:
            ESP_LOGI(TAG, "action: calibrate (%u g)", (unsigned)m->working.scale_cal_ref_g);
            {
                esp_err_t err = scale_hx711_calibrate_default((float)m->working.scale_cal_ref_g, MENU_CAL_SAMPLES);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "action: calibrate ok");
                    buzzer_beep_long(2);
                } else {
                    ESP_LOGW(TAG, "action: calibrate failed: %s", esp_err_to_name(err));
                    buzzer_beep_long(1);
                }
            }
            app_params_get(&m->working);
            return false;
        default:
            return false;
        }
    }

    if (m->view == UI_MENU_VIEW_PRESET_LIST) {
        (void)app_presets_select((uint8_t)m->index);
        app_params_get(&m->working);
        m->view = UI_MENU_VIEW_HOME;
        m->index = MENU_HOME_PRESET;
        buzzer_beep_short(2);
        ESP_LOGI(TAG, "preset: apply %u", (unsigned)app_presets_get_active_index());
        return false;
    }

    if (m->view == UI_MENU_VIEW_STRATEGY_LIST) {
        (void)app_fill_strategy_select((app_fill_strategy_t)m->index);
        app_params_get(&m->working);
        m->view = UI_MENU_VIEW_HOME;
        m->index = MENU_HOME_STRATEGY;
        buzzer_beep_short(2);
        ESP_LOGI(TAG, "strategy: apply %u", (unsigned)app_fill_strategy_get_active());
        return false;
    }

    if (m->view == UI_MENU_VIEW_PARAM_LIST) {
        m->view = UI_MENU_VIEW_PARAM_EDIT;
        ESP_LOGI(TAG, "edit: enter");
        return false;
    }

    if (m->view == UI_MENU_VIEW_PARAM_EDIT) {
        if (out_apply) *out_apply = m->working;
        m->view = UI_MENU_VIEW_PARAM_LIST;
        buzzer_beep_short(2);
        ESP_LOGI(TAG, "edit: save");
        return true;
    }

    return false;
}

bool ui_menu_on_long_press(ui_menu_t *m)
{
    if (!m) return false;

    if (m->view == UI_MENU_VIEW_PARAM_EDIT) {
        app_params_get(&m->working);
        m->view = UI_MENU_VIEW_PARAM_LIST;
        buzzer_beep_long(1);
        ESP_LOGI(TAG, "edit: cancel");
        return true;
    }

    if (m->view == UI_MENU_VIEW_PARAM_LIST ||
        m->view == UI_MENU_VIEW_PRESET_LIST ||
        m->view == UI_MENU_VIEW_STRATEGY_LIST) {
        app_params_get(&m->working);
        m->view = UI_MENU_VIEW_HOME;
        m->index = 0;
        buzzer_beep_long(1);
        ESP_LOGI(TAG, "menu: home");
        return true;
    }

    return false;
}

void ui_menu_render(const ui_menu_t *m, ssd1306_handle_t disp)
{
    if (!m) return;

    char line0[24] = {0};
    char line1[24] = {0};
    char line2[24] = {0};
    char line3[24] = {0};
    char line4[24] = {0};
    char line5[24] = {0};
    char line6[24] = {0};
    char line7[24] = {0};

    ssd1306_clear(disp);

    if (m->view == UI_MENU_VIEW_HOME) {
        const char *preset = app_presets_get_name(app_presets_get_active_index());
        const char *strategy = app_fill_strategy_get_name(app_fill_strategy_get_active());
        snprintf(line0, sizeof(line0), "Menu %u/%u",
                 (unsigned)(m->index + 1), (unsigned)MENU_HOME_COUNT);
        snprintf(line1, sizeof(line1), "Preset: %.14s", preset);
        snprintf(line2, sizeof(line2), "Mode: %.15s", strategy);
        snprintf(line3, sizeof(line3), "> %.20s", k_home_labels[m->index]);
        menu_render_home_detail(m, line4, sizeof(line4), line5, sizeof(line5), line6, sizeof(line6));
        snprintf(line7, sizeof(line7), "Long: Exit");
    } else if (m->view == UI_MENU_VIEW_PRESET_LIST) {
        uint8_t active = app_presets_get_active_index();
        const char *selected = app_presets_get_name((uint8_t)m->index);
        const char *active_name = app_presets_get_name(active);
        snprintf(line0, sizeof(line0), "Preset %u/%u",
                 (unsigned)(m->index + 1), (unsigned)app_presets_count());
        snprintf(line1, sizeof(line1), "> %.20s", selected);
        snprintf(line3, sizeof(line3), "Active: %.12s", active_name);
        snprintf(line5, sizeof(line5), "Click: Activate");
        snprintf(line6, sizeof(line6), "Keeps machine cfg");
        snprintf(line7, sizeof(line7), "Long: Back");
    } else if (m->view == UI_MENU_VIEW_STRATEGY_LIST) {
        app_fill_strategy_t active = app_fill_strategy_get_active();
        const char *selected = app_fill_strategy_get_name((app_fill_strategy_t)m->index);
        const char *active_name = app_fill_strategy_get_name(active);
        snprintf(line0, sizeof(line0), "Mode %u/%u",
                 (unsigned)(m->index + 1), (unsigned)app_fill_strategy_count());
        snprintf(line1, sizeof(line1), "> %.20s", selected);
        snprintf(line3, sizeof(line3), "Active: %.12s", active_name);
        snprintf(line5, sizeof(line5), "Click: Activate");
        snprintf(line6, sizeof(line6), "Global fill mode");
        snprintf(line7, sizeof(line7), "Long: Back");
    } else {
        size_t count = 0;
        const app_param_meta_t *meta = menu_meta_by_scope(m->scope, m->index, &count);
        if (!meta) return;

        char val[16];
        char defv[16];
        char brief_a[24];
        char brief_b[24];
        menu_format_value(meta, &m->working, val, sizeof(val));
        menu_format_meta_value(meta, &meta->def, defv, sizeof(defv));
        menu_wrap_brief(meta->desc_brief, brief_a, sizeof(brief_a), brief_b, sizeof(brief_b));

        const char *scope_title = (m->scope == APP_PARAM_SCOPE_PRESET) ? "Recipe" : "Machine";
        snprintf(line0, sizeof(line0), "%s %u/%u", scope_title,
                 (unsigned)(m->index + 1), (unsigned)count);
        if (m->scope == APP_PARAM_SCOPE_PRESET) {
            snprintf(line1, sizeof(line1), "Preset: %.13s",
                     app_presets_get_name(app_presets_get_active_index()));
        } else {
            snprintf(line1, sizeof(line1), "Global machine");
        }
        snprintf(line2, sizeof(line2), "%.20s", meta->label ? meta->label : meta->name);
        snprintf(line3, sizeof(line3), "%s%s%s",
                 (m->view == UI_MENU_VIEW_PARAM_EDIT) ? "> " : "V: ",
                 val,
                 (meta->unit && meta->unit[0]) ? " " : "");
        if (meta->unit && meta->unit[0]) {
            strncat(line3, meta->unit, sizeof(line3) - strlen(line3) - 1);
        }
        snprintf(line4, sizeof(line4), "Def: %.10s", defv);
        if (meta->unit && meta->unit[0]) {
            strncat(line4, " ", sizeof(line4) - strlen(line4) - 1);
            strncat(line4, meta->unit, sizeof(line4) - strlen(line4) - 1);
        }
        snprintf(line5, sizeof(line5), "%.20s", brief_a);
        snprintf(line6, sizeof(line6), "%.20s", brief_b);
        snprintf(line7, sizeof(line7), "%s",
                 (m->view == UI_MENU_VIEW_PARAM_EDIT) ? "Click Save Long Cxl"
                                                      : "Click Edit LongHome");
    }

    static int64_t last_blink_us = 0;
    static bool blink_on = true;
    if (m->view == UI_MENU_VIEW_PARAM_EDIT) {
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_blink_us >= 400000) {
            blink_on = !blink_on;
            last_blink_us = now_us;
        }
        if (!blink_on && line3[0] == '>') {
            line3[0] = ' ';
        }
    }

    ssd1306_draw_text(disp, 0, LINE2PIXEL(0), line0, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(1), line1, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(2), line2, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(3), line3, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(4), line4, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(5), line5, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(6), line6, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(7), line7, true);
    ssd1306_display(disp);
}
