#include <Arduino.h>
#include <Wire.h>  // Include Wire if you're using I2C
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library

#ifdef _VARIANT_WAVESHARE_BLE400_
  #include "angry_cookie_128x64.h"
  #define OLED_WIDTH 128
  #define OLED_HEIGHT 64
  MicroOLED oled;
#else
  #include "angry_cookie_64x32.h"
  #define OLED_WIDTH 64
  #define OLED_HEIGHT 32
  MicroOLED oled(OLED_RST, OLED_DC, OLED_CS);
#endif

uint32_t tButton;
#define debounceTime_ms 200

bool on_off_state = false;
bool wake_sleep_state = false;

void setup()
{
  Serial.begin(9600);
  Serial.println(__FILE__);
  
  #ifdef _VARIANT_WAVESHARE_BLE400_
  pinMode(PIN_BUTTON1, INPUT);
  #else
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  #endif
  pinMode(PIN_BUTTON2, INPUT);
  
  oled.setScreenSize(OLED_WIDTH, OLED_HEIGHT);
  oled.begin();
  oled.drawBitmap(angry_cookie);
  oled.display();
  oled.setFontType(0);  // Set font to type 0
  oled.setCursor(0, 0);
  oled.print("github.com/");
  oled.setCursor(0, 10);
  oled.print("micooke");
  oled.setCursor(0, 20);
  oled.print(__TIME__);
  oled.rect(0, 0, OLED_WIDTH, OLED_HEIGHT);
  
  oled.display();

  tButton = millis();
}

void loop()
{
  if (millis() - tButton > debounceTime_ms)
  {
    if (!digitalRead(PIN_BUTTON1))
    {
      if (wake_sleep_state)
      {
        Serial.println("sleep");
        oled.sleep();
      }
      else
      {
        Serial.println("wake");
        oled.wake();
      }
      wake_sleep_state = !wake_sleep_state;

      tButton = millis();
    }
    #ifdef _VARIANT_WAVESHARE_BLE400_
    if (!digitalRead(PIN_BUTTON2))
    #else
    if (digitalRead(PIN_BUTTON2))
    #endif
    {
      if (on_off_state)
      {
        Serial.println("off");
        oled.displayOff();
      }
      else
      {
        Serial.println("on");
        oled.displayOn();
      }
      on_off_state = !on_off_state;

      tButton = millis();
    }
  }
  
  yield();
}