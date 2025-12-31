#ifndef UI_MENU_H
#define UI_MENU_H

#include <stdbool.h>
#include <stdint.h>
#include "ssd1306.h"
#include "app.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UI_MENU_VIEW_LIST = 0,
    UI_MENU_VIEW_EDIT
} ui_menu_view_t;

typedef struct {
    bool           active;
    ui_menu_view_t view;
    size_t         index;
    app_params_t   working;
} ui_menu_t;

void ui_menu_enter(ui_menu_t *m, const app_params_t *cur);
void ui_menu_exit(ui_menu_t *m);
bool ui_menu_is_active(const ui_menu_t *m);
ui_menu_view_t ui_menu_get_view(const ui_menu_t *m);

void ui_menu_on_rotate(ui_menu_t *m, int32_t delta);
// Returns true when the current edit value should be saved (short click).
bool ui_menu_on_click(ui_menu_t *m, app_params_t *out_apply);
// Returns true when the edit was canceled and the view moved back to list.
bool ui_menu_on_long_press(ui_menu_t *m);

void ui_menu_render(const ui_menu_t *m, ssd1306_handle_t disp);

#ifdef __cplusplus
}
#endif

#endif // UI_MENU_H
