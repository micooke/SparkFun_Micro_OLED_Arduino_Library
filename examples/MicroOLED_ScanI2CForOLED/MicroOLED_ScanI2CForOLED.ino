#define SFE_MicroOLED_SoftwareI2C
#ifdef SFE_MicroOLED_SoftwareI2C
#include <i2c_device_list.h> //adds 7952 bytes, so comment it out if you dont need it
#endif
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library

#ifdef SFE_MicroOLED_SoftwareI2C
MicroOLED oled(24,25); // SoftwareI2C Example - sda,scl = 24,25
#else
MicroOLED oled; // I2C Example (No RST pin, i2c address  = 0x3C)
#endif

uint32_t t;

void setup()
{
  Serial.begin(9600);
  Serial.println(__FILE__);
  Serial.println(__TIME__);

  digitalWrite(13, HIGH);  
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  oled.getI2CAddress(); // scans i2c for a 0x3C or 0x3D device
  oled.setScreenSize(128, 64);
  oled.begin();     // Initialize the OLED
  oled.clear(ALL);  // Clear the library's display buffer
  oled.setFontType(0); // set font type 0, please see declaration in SFE_MicroOLED.cpp
  oled.setCursor(0, 0); // points cursor to x=27 y=0
  #ifdef SFE_MicroOLED_SoftwareI2C
  oled.print("SoftwareI2C");
  #else
  oled.print("Wire");
  #endif
  oled.setCursor(0, 10); // points cursor to x=27 y=0
  oled.print(__TIME__);
  oled.display(); // display the memory buffer drawn

  t = millis();
}

void loop()
{
  if (millis() - t > 1000) // output serial data (heartbeat) every second
  {
    Serial.print(".");
    t = millis();
  }
  yield();
}
