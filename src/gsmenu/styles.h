#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl/lvgl.h"

extern lv_style_t style_rootmenu;
extern lv_style_t style_openipc;
extern lv_style_t style_openipc_dropdown;
extern lv_style_t style_openipc_outline;
extern lv_style_t style_openipc_textcolor;
extern lv_style_t style_openipc_disabled;
extern lv_style_t style_openipc_section;
extern lv_style_t style_openipc_dark_background;
extern lv_style_t style_openipc_lightdark_background;

int style_init(void);

// Applies gsmenu.transparency (pixelpilot.yaml) to a menu page's own
// background -- lv_menu_page_create()'d objects each have their own
// theme-driven opaque background, separate from style_rootmenu (which only
// covers the outer lv_menu object itself), so this needs calling on every
// individual page for the video to actually show through consistently
// across the whole menu, not just its outermost frame.
void apply_menu_transparency(lv_obj_t *page);

#ifdef __cplusplus
}
#endif
