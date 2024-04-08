#include "ui.h"

int onOffStage;
int speed;	   // รับค่าความเร็วปัจจุบันจากสไลด์เดอร์
int mappspeed = 50; // แม็ปค่าความเร็วจากช่วง 0-100 เป็นช่วง 0-255
int map(int x, int in_min, int in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void toggleLED(lv_event_t *e)
{
	// digitalWrite(33, 1); // sets the digital pin 13 on
	// delay(1000);         // waits for a second
	// digitalWrite(33, 0); // sets the digital pin 13 off
	// delay(1000);
}

void setOnOff(lv_event_t *e)
{
	if (lv_obj_has_state(ui_BTNpower, LV_STATE_CHECKED))
	{
		onOffStage = 1;									 // ถ้าปุ่มถูกเปิด กำหนดค่าเป็น 1
		// lv_slider_set_value(ui_Slider1, 50, LV_ANIM_ON); // ตั้งค่าสไลด์ให้มีค่าเป็น 50 เมื่อเปิดระบบใหม่
		// mappspeed = 50;
		
	}
	else
	{
		onOffStage = 0; // ถ้าปุ่มถูกปิด กำหนดค่าเป็น 0
		// lv_slider_set_value(ui_Slider1, 0, LV_ANIM_ON); // ตั้งค่าสไลด์เป็น 0 (ปิด) เมื่อปิดไฟ
		 analogWrite(12, 0); // ปิดไฟ
	}
}

void setSpeed(lv_event_t *e)
{
 // แม็ปค่าความเร็วจากช่วง 0-100 เป็นช่วง 0-255
	   // รับค่าความเร็วปัจจุบันจากสไลด์เดอร์


			speed = lv_slider_get_value(ui_Slider1);

		mappspeed = map(speed, 0, 100, 0, 255);						   // ถ้าไฟเปิดอยู่
		
	

}
