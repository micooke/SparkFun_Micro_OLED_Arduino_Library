#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define SH1107_EXTERNAL_CONTROL
#if defined(_VARIANT_T28_) | defined(SH1107_EXTERNAL_CONTROL)
  #define SFEOLED_SH1107
#endif
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library

#if defined(SH1107_EXTERNAL_CONTROL)
  #define OLED_WIDTH 64
  #define OLED_HEIGHT 128
  MicroOLED oled(8, 9, 10);
#elif defined(_VARIANT_T28_)
  //#include "angry_cookie_64x32.h"
  #define OLED_WIDTH 64
  #define OLED_HEIGHT 128
  MicroOLED oled(OLED_RST, OLED_DC, OLED_CS);
#elif defined (_VARIANT_IDO003_) | defined (_VARIANT_ID100HR_) | defined(_VARIANT_ID107HR_)
  #include "angry_cookie_64x32.h"
  #define OLED_WIDTH 64
  #define OLED_HEIGHT 32
  MicroOLED oled(OLED_RST, OLED_DC, OLED_CS);
#else
  #include "angry_cookie_128x64.h"
  #define OLED_WIDTH 128
  #define OLED_HEIGHT 64
  MicroOLED oled;
#endif

uint32_t tButton;
#define debounceTime_ms 200

bool on_off_state = false;
bool wake_sleep_state = false;

void setup()
{
  #if defined(_VARIANT_T28_)
  pinMode(PIN_VIBRATE, OUTPUT);
  digitalWrite(PIN_VIBRATE, LOW);
  pinMode(PIN_HR_ON, OUTPUT);
  digitalWrite(PIN_HR_ON, HIGH);

  pinMode(PIN_OLED_VPP, OUTPUT);
  digitalWrite(PIN_OLED_VPP, HIGH);

  //pinMode(PIN_OLED_SW, OUTPUT);
  //digitalWrite(PIN_OLED_SW, HIGH);
  #endif
  
  Serial.begin(9600);
  Serial.println(__FILE__);
  #if defined(PIN_BUTTON1)
  #if defined(_VARIANT_WAVESHARE_BLE400_) | defined(_VARIANT_STCT_NRF52_minidev_)
  pinMode(PIN_BUTTON1, INPUT);
  #else
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  #endif
  #endif
  
  #if defined(PIN_BUTTON2)
  #if !(defined(_VARIANT_STCT_NRF52_minidev_) & defined(CONFIG_GPIO_AS_PINRESET))
  #if defined(_VARIANT_T28_)
  pinMode(PIN_BUTTON2, INPUT_PULLUP);
  #else
  pinMode(PIN_BUTTON2, INPUT);
  #endif
  #endif
  #endif
  
  oled.setScreenSize(OLED_WIDTH, OLED_HEIGHT);
  oled.begin();
  //oled.drawBitmap(angry_cookie);
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
  #if defined(PIN_BUTTON1)
  if (millis() - tButton > debounceTime_ms)
  {
    #if defined(_VARIANT_STCT_NRF52_minidev_)
    if (digitalRead(PIN_BUTTON1))
    #else
    if (!digitalRead(PIN_BUTTON1))
    #endif
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
    #if !(defined(_VARIANT_STCT_NRF52_minidev_) & defined(CONFIG_GPIO_AS_PINRESET))
      #if defined(_VARIANT_WAVESHARE_BLE400_) | defined(_VARIANT_T28_)
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
    #endif
  }
  #endif 
  yield();
}
