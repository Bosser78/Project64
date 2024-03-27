// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.6
// Project name: ios-ui

#ifndef _IOS_UI_UI_H
#define _IOS_UI_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl/lvgl.h"

#include "ui_helpers.h"
#include "ui_events.h"

void hideToday_Animation(lv_obj_t * TargetObject, int delay);
void islandExpand_Animation(lv_obj_t * TargetObject, int delay);
void islandExpandDown_Animation(lv_obj_t * TargetObject, int delay);
void unlock_Animation(lv_obj_t * TargetObject, int delay);
void lock_Animation(lv_obj_t * TargetObject, int delay);
void showToday_Animation(lv_obj_t * TargetObject, int delay);
void showControl_Animation(lv_obj_t * TargetObject, int delay);
void closeControl_Animation(lv_obj_t * TargetObject, int delay);
void hideToRight_Animation(lv_obj_t * TargetObject, int delay);
void hideToLeft_Animation(lv_obj_t * TargetObject, int delay);
void revealFromRight_Animation(lv_obj_t * TargetObject, int delay);
void revealFromLeft_Animation(lv_obj_t * TargetObject, int delay);
void opacityShow_Animation(lv_obj_t * TargetObject, int delay);
void opacityHide_Animation(lv_obj_t * TargetObject, int delay);
void homeUp_Animation(lv_obj_t * TargetObject, int delay);
// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
extern lv_obj_t * ui_Screen1;
extern lv_obj_t * ui_Panel2;
extern lv_obj_t * ui_Panel1;
extern lv_obj_t * ui_Panel3;
extern lv_obj_t * ui_Label1;
extern lv_obj_t * ui_Slider3;
extern lv_obj_t * ui_Label2;
extern lv_obj_t * ui_Label3;
extern lv_obj_t * ui_Button3;
extern lv_obj_t * ui_Button1;
extern lv_obj_t * ui_Button2;
extern lv_obj_t * ui_Label4;
extern lv_obj_t * ui_Label5;
extern lv_obj_t * ui_Label6;
extern lv_obj_t * ui_Label7;
extern lv_obj_t * ui_Label8;
extern lv_obj_t * ui_Label9;
extern lv_obj_t * ui_Label10;
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



LV_FONT_DECLARE(ui_font_Font2);



void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif