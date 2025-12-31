#include "ui_menu.h"

#include <stdio.h>
#include <string.h>

#include "app.h"
#include "esp_log.h"

#define LINE2PIXEL(n) ((n) * 8)

static const char *TAG = "ui_menu";

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

static const app_param_meta_t *menu_meta_at(size_t index, size_t *count_out)
{
    size_t count = 0;
    const app_param_meta_t *meta = app_params_meta_get(&count);
    if (count_out) *count_out = count;
    if (!meta || count == 0) return NULL;
    if (index >= count) index = count - 1;
    return &meta[index];
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
        int32_t idx = (int32_t)m->index + delta;
        if (idx < 0) idx = 0;
        if ((size_t)idx >= count) idx = (int32_t)count - 1;
        m->index = (size_t)idx;
        ESP_LOGD(TAG, "list: index=%u", (unsigned)m->index);
    } else {
        menu_apply_delta(meta, &m->working, delta);
        ESP_LOGD(TAG, "edit: %s delta=%ld", meta->name, (long)delta);
    }
}

bool ui_menu_on_click(ui_menu_t *m, app_params_t *out_apply)
{
    if (!m) return false;
    if (m->view == UI_MENU_VIEW_LIST) {
        m->view = UI_MENU_VIEW_EDIT;
        ESP_LOGI(TAG, "edit: enter");
        return false;
    }

    // Confirm edit: copy working params out for persistence.
    if (out_apply) *out_apply = m->working;
    m->view = UI_MENU_VIEW_LIST;
    ESP_LOGI(TAG, "edit: save");
    return true;
}

bool ui_menu_on_long_press(ui_menu_t *m)
{
    if (!m) return false;
    if (m->view == UI_MENU_VIEW_EDIT) {
        m->view = UI_MENU_VIEW_LIST;
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

    char line0[24];
    char line1[24];
    char line2[24];
    char line3[24];

    ssd1306_clear(disp);

    if (m->view == UI_MENU_VIEW_LIST) {
        snprintf(line0, sizeof(line0), "Settings %u/%u",
                 (unsigned)(m->index + 1), (unsigned)count);
        snprintf(line1, sizeof(line1), "%s", meta->label ? meta->label : meta->name);

        char val[16];
        menu_format_value(meta, &m->working, val, sizeof(val));
        snprintf(line2, sizeof(line2), "Val: %s%s%s",
                 val,
                 (meta->unit && meta->unit[0]) ? " " : "",
                 (meta->unit && meta->unit[0]) ? meta->unit : "");
        snprintf(line3, sizeof(line3), "Click=Edit  Lng=Exit");
    } else {
        char val[16];
        char defv[16];
        menu_format_value(meta, &m->working, val, sizeof(val));
        snprintf(line0, sizeof(line0), "%s", meta->label ? meta->label : meta->name);
        snprintf(line1, sizeof(line1), "Val: %s%s%s",
                 val,
                 (meta->unit && meta->unit[0]) ? " " : "",
                 (meta->unit && meta->unit[0]) ? meta->unit : "");
        menu_format_meta_value(meta, &meta->def, defv, sizeof(defv));
        snprintf(line2, sizeof(line2), "Default: %s%s%s",
                 defv,
                 (meta->unit && meta->unit[0]) ? " " : "",
                 (meta->unit && meta->unit[0]) ? meta->unit : "");
        snprintf(line3, sizeof(line3), "%s", meta->desc_brief ? meta->desc_brief : "");
    }

    ssd1306_draw_text(disp, 0, LINE2PIXEL(0), line0, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(2), line1, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(4), line2, true);
    ssd1306_draw_text(disp, 0, LINE2PIXEL(7), line3, true);
    ssd1306_display(disp);
}
