/*
   MIT License

  Copyright (c) 2022 Felix Biego

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#define LGFX_USE_V1

#include <Arduino.h>
#include <lvgl.h>
#include "ui.h"
#include <LovyanGFX.hpp>
#include <ESP32Time.h>
#include <tcs3200.h>
#include <vector>

#ifdef PLUS
#define SCR 30
class LGFX : public lgfx::LGFX_Device
{

  lgfx::Panel_ST7796 _panel_instance;

  lgfx::Bus_Parallel8 _bus_instance;

  lgfx::Light_PWM _light_instance;

  lgfx::Touch_FT5x06 _touch_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();

      cfg.port = 0;
      cfg.freq_write = 40000000;
      cfg.pin_wr = 47; // pin number connecting WR
      cfg.pin_rd = -1; // pin number connecting RD
      cfg.pin_rs = 0;  // Pin number connecting RS(D/C)
      cfg.pin_d0 = 9;  // pin number connecting D0
      cfg.pin_d1 = 46; // pin number connecting D1
      cfg.pin_d2 = 3;  // pin number connecting D2
      cfg.pin_d3 = 8;  // pin number connecting D3
      cfg.pin_d4 = 18; // pin number connecting D4
      cfg.pin_d5 = 17; // pin number connecting D5
      cfg.pin_d6 = 16; // pin number connecting D6
      cfg.pin_d7 = 15; // pin number connecting D7

      _bus_instance.config(cfg);              // Apply the settings to the bus.
      _panel_instance.setBus(&_bus_instance); // Sets the bus to the panel.
    }

    {                                      // Set display panel control.
      auto cfg = _panel_instance.config(); // Get the structure for display panel settings.

      cfg.pin_cs = -1;   // Pin number to which CS is connected (-1 = disable)
      cfg.pin_rst = 4;   // pin number where RST is connected (-1 = disable)
      cfg.pin_busy = -1; // pin number to which BUSY is connected (-1 = disable)

      // * The following setting values ​​are set to general default values ​​for each panel, and the pin number (-1 = disable) to which BUSY is connected, so please try commenting out any unknown items.

      cfg.memory_width = 320;  // Maximum width supported by driver IC
      cfg.memory_height = 480; // Maximum height supported by driver IC
      cfg.panel_width = 320;   // actual displayable width
      cfg.panel_height = 480;  // actual displayable height
      cfg.offset_x = 0;        // Panel offset in X direction
      cfg.offset_y = 0;        // Panel offset in Y direction
      cfg.offset_rotation = 2;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = false;
      cfg.invert = true;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;

      _panel_instance.config(cfg);
    }

    {                                      // Set backlight control. (delete if not necessary)
      auto cfg = _light_instance.config(); // Get the structure for backlight configuration.

      cfg.pin_bl = 45;     // pin number to which the backlight is connected
      cfg.invert = false;  // true to invert backlight brightness
      cfg.freq = 44100;    // backlight PWM frequency
      cfg.pwm_channel = 0; // PWM channel number to use

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance); // Sets the backlight to the panel.
    }

    { // Configure settings for touch screen control. (delete if not necessary)
      auto cfg = _touch_instance.config();

      cfg.x_min = 0;   // Minimum X value (raw value) obtained from the touchscreen
      cfg.x_max = 319; // Maximum X value (raw value) obtained from the touchscreen
      cfg.y_min = 0;   // Minimum Y value obtained from touchscreen (raw value)
      cfg.y_max = 479; // Maximum Y value (raw value) obtained from the touchscreen
      cfg.pin_int = 7; // pin number to which INT is connected
      cfg.bus_shared = false;
      cfg.offset_rotation = 0;

      // For I2C connection
      cfg.i2c_port = 0;    // Select I2C to use (0 or 1)
      cfg.i2c_addr = 0x38; // I2C device address number
      cfg.pin_sda = 6;     // pin number where SDA is connected
      cfg.pin_scl = 5;     // pin number to which SCL is connected
      cfg.freq = 400000;   // set I2C clock

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance); // Set the touchscreen to the panel.
    }

    setPanel(&_panel_instance); // Sets the panel to use.
  }
};

#else
#define SCR 8
class LGFX : public lgfx::LGFX_Device
{

  lgfx::Panel_ST7796 _panel_instance;

  lgfx::Bus_SPI _bus_instance;

  lgfx::Light_PWM _light_instance;

  lgfx::Touch_FT5x06 _touch_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config(); // Get the structure for bus configuration.

      // SPI bus settings
      cfg.spi_host = VSPI_HOST; // Select the SPI to use ESP32-S2,C3 : SPI2_HOST or SPI3_HOST / ESP32 : VSPI_HOST or HSPI_HOST
      // * With the ESP-IDF version upgrade, VSPI_HOST and HSPI_HOST descriptions are deprecated, so if an error occurs, use SPI2_HOST and SPI3_HOST instead.
      cfg.spi_mode = 3;                  // Set SPI communication mode (0 ~ 3)
      cfg.freq_write = 27000000;         // SPI clock when sending (up to 80MHz, rounded to 80MHz divided by an integer)
      cfg.freq_read = 6000000;           // SPI clock when receiving
      cfg.spi_3wire = false;             // set to true if receiving on MOSI pin
      cfg.use_lock = true;               // set to true to use transaction lock
      cfg.dma_channel = SPI_DMA_CH_AUTO; // Set the DMA channel to use (0=not use DMA / 1=1ch / 2=ch / SPI_DMA_CH_AUTO=auto setting)
      // * With the ESP-IDF version upgrade, SPI_DMA_CH_AUTO (automatic setting) is recommended for the DMA channel. Specifying 1ch and 2ch is deprecated.
      cfg.pin_sclk = 14; // set SPI SCLK pin number
      cfg.pin_mosi = 13; // Set MOSI pin number for SPI
      cfg.pin_miso = -1; // set SPI MISO pin number (-1 = disable)
      cfg.pin_dc = 21;   // Set SPI D/C pin number (-1 = disable)

      _bus_instance.config(cfg);              // Apply the settings to the bus.
      _panel_instance.setBus(&_bus_instance); // Sets the bus to the panel.
    }

    {                                      // Set display panel control.
      auto cfg = _panel_instance.config(); // Get the structure for display panel settings.

      cfg.pin_cs = 15;   // Pin number to which CS is connected (-1 = disable)
      cfg.pin_rst = 22;  // pin number where RST is connected (-1 = disable)
      cfg.pin_busy = -1; // pin number to which BUSY is connected (-1 = disable)

      // * The following setting values ​​are set to general default values ​​for each panel, and the pin number (-1 = disable) to which BUSY is connected, so please try commenting out any unknown items.

      cfg.memory_width = 320;  // Maximum width supported by driver IC
      cfg.memory_height = 480; // Maximum height supported by driver IC
      cfg.panel_width = 320;   // actual displayable width
      cfg.panel_height = 480;  // actual displayable height
      cfg.offset_x = 0;        // Panel offset in X direction
      cfg.offset_y = 0;        // Panel offset in Y direction
      cfg.offset_rotation = 1;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = false;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;

      _panel_instance.config(cfg);
    }

    {                                      // Set backlight control. (delete if not necessary)
      auto cfg = _light_instance.config(); // Get the structure for backlight configuration.

      cfg.pin_bl = 23;     // pin number to which the backlight is connected
      cfg.invert = false;  // true to invert backlight brightness
      cfg.freq = 44100;    // backlight PWM frequency
      cfg.pwm_channel = 1; // PWM channel number to use

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance); // Sets the backlight to the panel.
    }

    { // Configure settings for touch screen control. (delete if not necessary)
      auto cfg = _touch_instance.config();

      cfg.x_min = 0;    // Minimum X value (raw value) obtained from the touchscreen
      cfg.x_max = 319;  // Maximum X value (raw value) obtained from the touchscreen
      cfg.y_min = 0;    // Minimum Y value obtained from touchscreen (raw value)
      cfg.y_max = 479;  // Maximum Y value (raw value) obtained from the touchscreen
      cfg.pin_int = 39; // pin number to which INT is connected
      cfg.bus_shared = false;
      cfg.offset_rotation = 0;

      // For I2C connection
      cfg.i2c_port = 1;    // Select I2C to use (0 or 1)
      cfg.i2c_addr = 0x38; // I2C device address number
      cfg.pin_sda = 18;    // pin number where SDA is connected
      cfg.pin_scl = 19;    // pin number to which SCL is connected
      cfg.freq = 400000;   // set I2C clock

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance); // Set the touchscreen to the panel.
    }

    setPanel(&_panel_instance); // Sets the panel to use.
  }
};

#endif

// Create an instance of the prepared class.
LGFX tft;

/* Change to your screen resolution */
static const uint32_t screenWidth = 480;
static const uint32_t screenHeight = 320;

static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;

static lv_color_t disp_draw_buf[screenWidth * SCR];
static lv_color_t disp_draw_buf2[screenWidth * SCR];

// LV_IMG_DECLARE(ui_img_four_64_png);
// LV_IMG_DECLARE(ui_img_six_64_png);
// LV_IMG_DECLARE(ui_img_eight_64_png);
// LV_IMG_DECLARE(ui_img_nine_64_png);

// lv_img_dsc_t digits[10] = {ui_img_zero_64_png, ui_img_one_64_png, ui_img_two_64_png, ui_img_three_64_png, ui_img_four_64_png,
//                            ui_img_five_64_png, ui_img_six_64_png, ui_img_seven_64_png, ui_img_eight_64_png, ui_img_nine_64_png};

ESP32Time rtc;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  if (tft.getStartCount() == 0)
  {
    tft.endWrite();
  }

  tft.pushImageDMA(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, (lgfx::swap565_t *)&color_p->full);

  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  uint16_t touchX, touchY;

  bool touched = tft.getTouch(&touchX, &touchY);

  if (!touched)
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;
  }
}

void onBrightnessChange(lv_event_t *e)
{
  lv_obj_t *slider = lv_event_get_target(e);
  int brightness = (int)lv_slider_get_value(slider);
  tft.setBrightness(brightness);
}
tcs3200 tcs(5, 33, 32, 27, 25);
// tcs3200 tcs(2, 4, 33, 32, 27);  (S0, S1, S2, S3, output pin)
// Servo myservo;

std::vector<int> chiliVector = {NULL};
#define RED_CHILI 1
#define GREEN_CHILI 2
#define EMPTY 3
void setup()
{
  pinMode(35, INPUT_PULLUP);

  Serial.begin(115200);

  tft.init();

  tft.initDMA();
  tft.startWrite();

  lv_init();

  Serial.print("Width: ");
  Serial.print(screenWidth);
  Serial.print("\tHeight: ");
  Serial.println(screenHeight);

  if (!disp_draw_buf)
  {
    Serial.println("LVGL disp_draw_buf allocate failed!");
  }
  else
  {

    Serial.print("Display buffer size: ");

    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, disp_draw_buf2, screenWidth * SCR);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;

    lv_disp_drv_register(&disp_drv);
    /* Initialize the input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;

    lv_indev_drv_register(&indev_drv);

    ui_init();

    Serial.println("Setup done");
  }
  pinMode(12, OUTPUT);
}
extern int mappspeed1, mappspeed2, speed;
extern int onOffStage, reset, st;
double red, green, blue;
double h = 0;     // Initialize H value
double h_sum = 0; // Initialize sum of H values
int h_count = 0;  // Initialize count of H values
// int pwm_count = 0; // ต้องทำค่าให้เข้ากับความเร็ว
int input;
double h_avg;
int output;
int servoposition = 0;
void rgb_to_hsv(double r, double g, double b);

int red2 = 0;
int green2 = 0;
int numGray2 = 0;

bool objdetac = false;
bool rgbcheck = true;
int avgR = 0;
int avgG = 0;
int avgB = 0;

int sumR = 0;
int sumG = 0;
int sumB = 0;
int count = 0;

int blackR, blackG, blackB;
int whiteR, whiteG, whiteB;

void readtsc()
{
  red = tcs.colorRead('r');   // reads color value for red
  green = tcs.colorRead('g'); // reads color value for green
  blue = tcs.colorRead('b');  // reads color value for blue

  // Normalize against white calibration values
  //  red = map(red, 0, 40 - 16, 0, 255);
  //  green = map(green, 0, 43 - 16, 0, 255);
  //  blue = map(blue, 0, 33 - 18, 0, 255);
  // if (rgbcheck)
  // {
  //   analogWrite(2, 150); // ตั้งค่าความเร็ว
  //   analogWrite(4, 0);   // ตั้งค่าความเร็ว

  //   sumR += red;
  //   sumG += green;
  //   sumB += blue;
  //   count++;

  //   if (count >= 400)
  //   {
  //     // คำนวณค่าเฉลี่ย
  //     avgR = sumR / count;
  //     avgG = sumG / count;
  //     avgB = sumB / count;

  //     // แสดงค่าเฉลี่ยสี RGB
  //     Serial.print("Average R: ");
  //     Serial.println(avgR);
  //     Serial.print("Average G: ");
  //     Serial.println(avgG);
  //     Serial.print("Average B: ");
  //     Serial.println(avgB);

  //     // หยุดการเก็บค่าเฉลี่ยหลังจากคำนวณแล้ว
  //     rgbcheck = false;
  //     delay(500);
  //   }
  // }

  // // ลบค่าเฉลี่ยออกจากค่าที่อ่านได้
  // if (!rgbcheck) // คำนวณหลังจากที่ค่าเฉลี่ยได้แล้ว
  // {
  //   red = red - avgR;
  //   green = green - avgG ;// -1 เพราะ สีเขียวเป็นไรไม่รู้ค่าเพิ่มมา 1
  //   blue = blue - avgB;

  //   Serial.print("Red after subtraction: ");
  //   Serial.println(red);
  //   Serial.print("Green after subtraction: ");
  //   Serial.println(green);
  //   Serial.print("Blue after subtraction: ");
  //   Serial.println(blue);
  // }
  Serial.print("Red after subtraction: ");
  Serial.println(red);
  Serial.print("Green after subtraction: ");
  Serial.println(green);
  Serial.print("Blue after subtraction: ");
  Serial.println(blue);

  // Check if the color is white

    // Check if the RGB values fall within the specified ranges for red and green
    // if ()
    // {
    //   Serial.println("Color: Red");
    // }
    // else if (red >= 20 && red <= 50 && green >= 120 && green <= 180 && blue >= 30 && blue <= 50)
    // {
    //   Serial.println("Color: Green");
    // }
    // else
    // {
    //   Serial.println("Color: Unknown");
    // }
  

  delay(1000);
}

unsigned long lastTimeChecked = 0;
unsigned long startTimeChili = 0;
unsigned long captureDuration = 500;  // 0.5 วินาที
unsigned long captureDuration1 = 200; // 0.2 วินาที
bool capturing = false;

unsigned long startCaptureTime = 0; // ตัวแปรเก็บเวลาเริ่มต้นการตรวจจับพริก

void checkchii111() // gpt
{
  if (capturing)
  {
    analogWrite(2, 120);        // ตั้งค่าความเร็วของไฟฟ้า
    analogWrite(4, mappspeed2); // ตั้งค่าความเร็วของไฟฟ้า

    // เก็บค่าทุกค่าในช่วงที่ตรวจจับพริกอยู่
    h_sum += h;
    h_count++;
    if ((h > 260 || h < 170))
    {
      lastTimeChecked = millis(); // อัพเดทเวลาเมื่อเจอค่าที่ตรงกับเงื่อนไข
    }
    // หากไม่เจอพริกในช่วงเวลาเกิน 0.4 วินาที
    if (millis() - lastTimeChecked > 200)
    {
      Serial.println("time out");
      capturing = false; // หยุดเก็บค่าถ้าเวลาเกิน 0.4 วินาที
    }
  }
  else
  {
    analogWrite(2, mappspeed1); // ตั้งค่าความเร็วของไฟฟ้า
    analogWrite(4, mappspeed2); // ตั้งค่าความเร็วของไฟฟ้า
    Serial.println("not obj");

    // ตรวจจับพริก
    if ((h > 260 || h < 170))
    {
      if (startCaptureTime == 0)
      {
        startCaptureTime = millis(); // บันทึกเวลาเริ่มต้นเมื่อเจอพริกครั้งแรก
      }
      else if (millis() - startCaptureTime > 50) // ตรวจสอบเวลาหากผ่านไปมากกว่า 0.5 วินาที
      {
        capturing = true;
        lastTimeChecked = millis(); // เริ่มเก็บค่าใหม่
      }
    }
    else
    {
      startCaptureTime = 0; // รีเซ็ตเวลาเริ่มต้นหากค่าไม่ตรงเงื่อนไข
    }
  }

  if (!capturing && h_count > 0)
  {
    float h_avg = (float)h_sum / h_count;

    if (h_avg >= 260 && h_avg <= 355)
    {
      Serial.println("Red");

      red2++;
      _ui_label_set_property(ui_numRed, _UI_LABEL_PROPERTY_TEXT, std::to_string(red2).c_str());

      chiliVector.insert(chiliVector.begin(), 1);
    }
    else if (h_avg >= 12 && h_avg <= 50)
    {
      Serial.println("Green");
      green2++;
      _ui_label_set_property(ui_numGreen, _UI_LABEL_PROPERTY_TEXT, std::to_string(green2).c_str());

      chiliVector.insert(chiliVector.begin(), 2);
    }
    else
    {
      Serial.println("Not");
      numGray2++;
      _ui_label_set_property(ui_numGray, _UI_LABEL_PROPERTY_TEXT, std::to_string(numGray2).c_str());

      chiliVector.insert(chiliVector.begin(), 3);
    }

    // รีเซ็ตค่าเพื่อเก็บค่าใหม่ในครั้งต่อไป
    h_sum = 0;
    h_count = 0;
  }
}
const int sensorPin = 2;                 // Pin ที่เชื่อมต่อกับเซนเซอร์
bool chiliDetected = false;              // สถานะว่าพริกเข้ามาหรือยัง
unsigned long detectionTime = 0;         // เวลาที่ตรวจจับพริก
const unsigned long debounceDelay = 500; // เวลาหน่วงเพื่อป้องกันการตรวจจับซ้ำ
bool chili = false;
void readobj()
{

  input = digitalRead(35);

  if (input == 0)
  {
    // ตรวจพบพริก
    if (!chiliDetected)
    {
      chiliDetected = true;
      detectionTime = millis(); // บันทึกเวลาที่พริกเข้ามา
      Serial.println("Chili detected!");
    }
  }
  else
  {
    // พริกออกไปแล้ว
    if (chiliDetected && (millis() - detectionTime > debounceDelay))
    {
      chiliDetected = false;
      // ส่งค่าไปประมวลผล
      chili = true;
    }
  }
}
void setServoPosition(int angle)
{
  // คำนวณความกว้างของพัลส์ที่ต้องการ (1ms ถึง 2ms)
  int pulseWidth = map(angle, 0, 180, 544, 2400); // 544us ถึง 2400us
  // ส่งพัลส์ PWM ไปที่พินของเซอร์โว
  digitalWrite(12, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(12, LOW);
  delay(20 - pulseWidth / 1000);
}
bool hasRemoved = false;
void servoslite()
{
  if (chiliVector.back() == 0)
  {
    chiliVector.pop_back();
  }
  if (!chiliVector.empty())
  {

    if (chili)
    {

      if (chiliVector.back() == RED_CHILI)
      {

        setServoPosition(0);
        servoposition = 0;
        chiliVector.pop_back();
        hasRemoved = true;
      }
      else if (chiliVector.back() == GREEN_CHILI)
      {

        setServoPosition(90);
        servoposition = 90;
        chiliVector.pop_back();
        chili = false;
      }
      else if (chiliVector.back() == EMPTY)
      {

        setServoPosition(45);
        servoposition = 45;
        chiliVector.pop_back();
        chili = false;
      }
    }
    else
    {
      hasRemoved = false;

      setServoPosition(servoposition);
    }
  }
}


// void rgbcalibrate()
// {

//   analogWrite(2, 150); // ตั้งค่าความเร็ว

//   sumR += red;
//   sumG += green;
//   sumB += blue;
//   count++;

//   if (count >= 400)
//   {
//     // คำนวณค่าเฉลี่ย
//     int avgR = sumR / count;
//     int avgG = sumG / count;
//     int avgB = sumB / count;

//     // แสดงค่าเฉลี่ยสี RGB
//     Serial.print("Average R: ");
//     Serial.println(avgR);
//     Serial.print("Average G: ");
//     Serial.println(avgG);
//     Serial.print("Average B: ");
//     Serial.println(avgB);
//     delay(5000);
//   }
// }

void loop()
{

  lv_timer_handler(); /* let the GUI do its work */
  if (onOffStage == 1)
  {

    if (st == 0)
    {
      // _ui_label_set_property(ui_numGreen, _UI_LABEL_PROPERTY_TEXT, "10");
      // _ui_label_set_property(ui_numRed, _UI_LABEL_PROPERTY_TEXT, "01");
      // _ui_label_set_property(ui_numGray, _UI_LABEL_PROPERTY_TEXT, "10");
      // readtsc();

      // checkchii111();
      // readobj();

      // servoslite();
      // delay(5);

      // Serial.print("input =");
      // Serial.print(input);
      // Serial.print("H= ");
      // Serial.print(h);
      // Serial.print(st);
      // Serial.print("red =");
      // Serial.println(red2);
      // Serial.print("green = ");
      // Serial.println(green2);
      // Serial.print("blue = ");
      // Serial.println(blue);
      // Serial.print(st);

      // Serial.println("Havg= ");

      // Serial.println("Chili vector: ");

      // for (int i = 0; i < chiliVector.size(); ++i)
      // {
      //   Serial.print(chiliVector[i]);
      // }
    }
    else
    {
      analogWrite(2, 0); // ตั้งค่าความเร็วของไฟฟ้า
      analogWrite(4, 0);
      red2 = 100;

      _ui_label_set_property(ui_numGreen, _UI_LABEL_PROPERTY_TEXT, std::to_string(red2).c_str());
      _ui_label_set_property(ui_numRed, _UI_LABEL_PROPERTY_TEXT, "-");
      _ui_label_set_property(ui_numGray, _UI_LABEL_PROPERTY_TEXT, "-");
    }
  }
  if (reset == 1)
  {
    // green2 = 0;
    // numGray2 = 0;
    red2 = 0;
    _ui_label_set_property(ui_numGreen, _UI_LABEL_PROPERTY_TEXT, std::to_string(red2).c_str());
    _ui_label_set_property(ui_numRed, _UI_LABEL_PROPERTY_TEXT, "0");
    _ui_label_set_property(ui_numGray, _UI_LABEL_PROPERTY_TEXT, "0");

    delay(1000);
  }
}