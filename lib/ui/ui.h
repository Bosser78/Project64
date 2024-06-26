// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#ifndef _SQUARELINE_PROJECT_UI_H
#define _SQUARELINE_PROJECT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#include "ui_helpers.h"
#include "ui_events.h"
#include "ui_theme_manager.h"
#include "ui_themes.h"

void blink_Animation(lv_obj_t * TargetObject, int delay);
// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
extern lv_obj_t * ui_Screen1;
void ui_event_Panel2(lv_event_t * e);
extern lv_obj_t * ui_Panel2;
extern lv_obj_t * ui_Panel1;
extern lv_obj_t * ui_Label2;
// SCREEN: ui_Screen2
void ui_Screen2_screen_init(void);
extern lv_obj_t * ui_Screen2;
void ui_event_PanelGreen(lv_event_t * e);
extern lv_obj_t * ui_PanelGreen;
void ui_event_PanelRed(lv_event_t * e);
extern lv_obj_t * ui_PanelRed;
void ui_event_PanelGray(lv_event_t * e);
extern lv_obj_t * ui_PanelGray;
extern lv_obj_t * ui_Label3;
extern lv_obj_t * ui_Label4;
extern lv_obj_t * ui_Label5;
extern lv_obj_t * ui_numGreen;
extern lv_obj_t * ui_numRed;
extern lv_obj_t * ui_numGray;
void ui_event_BTNpower(lv_event_t * e);
extern lv_obj_t * ui_BTNpower;
extern lv_obj_t * ui_Button2;
extern lv_obj_t * ui_Button3;
void ui_event_Slider1(lv_event_t * e);
extern lv_obj_t * ui_Slider1;
extern lv_obj_t * ui_LabelOnOff;
extern lv_obj_t * ui_Label10;
extern lv_obj_t * ui_Label11;
extern lv_obj_t * ui_Label12;
extern lv_obj_t * ui_speed;
extern lv_obj_t * ui____initial_actions0;


LV_IMG_DECLARE(ui_img_eight_32_png);    // assets/eight_32.png
LV_IMG_DECLARE(ui_img_eight_64_png);    // assets/eight_64.png
LV_IMG_DECLARE(ui_img_five_64_png);    // assets/five_64.png
LV_IMG_DECLARE(ui_img_four_64_png);    // assets/four_64.png
LV_IMG_DECLARE(ui_img_nine_64_png);    // assets/nine_64.png
LV_IMG_DECLARE(ui_img_one_64_png);    // assets/one_64.png
LV_IMG_DECLARE(ui_img_pause_32_png);    // assets/pause_32.png
LV_IMG_DECLARE(ui_img_seven_64_png);    // assets/seven_64.png
LV_IMG_DECLARE(ui_img_six_64_png);    // assets/six_64.png
LV_IMG_DECLARE(ui_img_three_64_png);    // assets/three_64.png
LV_IMG_DECLARE(ui_img_two_64_png);    // assets/two_64.png
LV_IMG_DECLARE(ui_img_zero_64_png);    // assets/zero_64.png



LV_FONT_DECLARE(ui_font_Font1);
LV_FONT_DECLARE(ui_font_Font2);
LV_FONT_DECLARE(ui_font_Font24);



void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
