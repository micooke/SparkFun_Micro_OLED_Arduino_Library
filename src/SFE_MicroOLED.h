/****************************************************************************** 
SFE_MicroOLED.h
Header file for the MicroOLED Arduino Library

Jim Lindblom @ SparkFun Electronics
October 26, 2014
https://github.com/sparkfun/Micro_OLED_Breakout/tree/master/Firmware/Arduino/libraries/SFE_MicroOLED

Modified by:
Emil Varughese @ Edwin Robotics Pvt. Ltd.
July 27, 2015
https://github.com/emil01/SparkFun_Micro_OLED_Arduino_Library/

Modified by:
Mark Cooke
Sep 18, 2017
https://github.com/micooke/sparkfun_OLED

This file defines the hardware interface(s) for the Micro OLED Breakout. Those
interfaces include SPI, I2C and a parallel bus.

Development environment specifics:
Arduino 1.0.5
Arduino Pro 3.3V
Micro OLED Breakout v1.0

This code was heavily based around the MicroView library, written by GeekAmmo
(https://github.com/geekammo/MicroView-Arduino-Library), and released under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later 
version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#ifndef SFE_MICROOLED_H
#define SFE_MICROOLED_H

#include <stdio.h>
#include <Arduino.h>

#if defined(__AVR__) || defined(__arm__)
	#include <avr/pgmspace.h>
#else
	#include <pgmspace.h>
#endif

#define swap(a, b) { uint8_t t = a; a = b; b = t; }

#define I2C_ADDRESS_SA0_0 0b0111100 // 0x3C
#define I2C_ADDRESS_SA0_1 0b0111101 // 0x3D
#define I2C_COMMAND 0x00
#define I2C_DATA 0x40

#define SFEOLED_BLACK 0
#define SFEOLED_WHITE 1

#define SFEOLED_LCDWIDTH				64
#define SFEOLED_LCDHEIGHT				48
//#define LCD_BUFFER_SIZE (LCDHEIGHT*LCDWIDTH/8) // 384 - Default Value

#define SFEOLED_FONTHEADERSIZE		6

#define SFEOLED_NORM						0
#define SFEOLED_XOR						1

#define SFEOLED_PAGE						0
#define SFEOLED_ALL						1

#ifndef PAGE
#define PAGE SFEOLED_PAGE
#endif

#ifndef ALL
#define ALL SFEOLED_ALL
#endif

#define SFEOLED_SETCONTRAST 			0x81
#define SFEOLED_DISPLAYALLONRESUME 	0xA4
#define SFEOLED_DISPLAYALLON 			0xA5
#define SFEOLED_NORMALDISPLAY 		0xA6
#define SFEOLED_INVERTDISPLAY 		0xA7
#define SFEOLED_DISPLAYOFF 			0xAE
#define SFEOLED_DISPLAYON 				0xAF
#define SFEOLED_SETDISPLAYOFFSET 	0xD3
#define SFEOLED_SETCOMPINS 			0xDA
#define SFEOLED_SETVCOMDESELECT		0xDB
#define SFEOLED_SETDISPLAYCLOCKDIV 	0xD5
#define SFEOLED_SETPRECHARGE 			0xD9
#define SFEOLED_SETMULTIPLEX 			0xA8
#define SFEOLED_SETLOWCOLUMN 			0x00
#define SFEOLED_SETHIGHCOLUMN 		0x10
#define SFEOLED_SETSTARTLINE 			0x40
#define SFEOLED_COMSCANINC 			0xC0
#define SFEOLED_COMSCANDEC 			0xC8
#define SFEOLED_SEGREMAP 				0xA0
#define SFEOLED_CHARGEPUMP 			0x8D
#define SFEOLED_EXTERNALVCC 			0x01
#define SFEOLED_SWITCHCAPVCC 			0x02

#define SFEOLED_MEMORYMODE 			0x20
#define SFEOLED_COLUMNADDR   			0x21
#define SFEOLED_PAGEADDR   			0x22

// Scroll
#define SFEOLED_ACTIVATESCROLL 						0x2F
#define SFEOLED_DEACTIVATESCROLL 					0x2E
#define SFEOLED_SETVERTICALSCROLLAREA 				0xA3
#define SFEOLED_RIGHTHORIZONTALSCROLL 				0x26
#define SFEOLED_LEFT_HORIZONTALSCROLL 				0x27
#define SFEOLED_VERTICALRIGHTHORIZONTALSCROLL	0x29
#define SFEOLED_VERTICALLEFTHORIZONTALSCROLL		0x2A

typedef enum CMD {
	CMD_CLEAR,			//0
	CMD_INVERT,			//1
	CMD_CONTRAST,		//2
	CMD_DISPLAY,		//3
	CMD_SETCURSOR,		//4
	CMD_PIXEL,			//5
	CMD_LINE,			//6
	CMD_LINEH,			//7
	CMD_LINEV,			//8
	CMD_RECT,			//9
	CMD_RECTFILL,		//10
	CMD_CIRCLE,			//11
	CMD_CIRCLEFILL,		//12
	CMD_DRAWCHAR,		//13
	CMD_DRAWBITMAP,		//14
	CMD_GETLCDWIDTH,	//15
	CMD_GETLCDHEIGHT,	//16
	CMD_SETCOLOR,		//17
	CMD_SETDRAWMODE		//18
} commCommand_t;

typedef enum COMM_MODE{
	COMM_MODE_SPI,
	COMM_MODE_I2C,
	COMM_MODE_PARALLEL
} micro_oled_mode;

class MicroOLED : public Print{
public:
	// Constructor(s)
	MicroOLED(uint8_t rst = -1, uint8_t dc = 0); // I2C
	MicroOLED(uint8_t rst, uint8_t dc, uint8_t cs); // SPI
	MicroOLED(uint8_t rst, uint8_t dc, uint8_t cs, uint8_t wr, uint8_t rd, 
			  uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, 
			  uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7); // Parallel
	~MicroOLED() { free(screenmemory); }
	void begin(void);
	virtual size_t write(uint8_t);

	// RAW LCD functions
	void command(uint8_t c);
	void data(uint8_t c);
	void dataBlock(uint8_t c, uint16_t len);
	void dataBlock(uint8_t * c, uint16_t startIdx, uint16_t len);
	void setColumnAddress(uint8_t add);
	void setPageAddress(uint8_t add);
	
	// LCD Draw functions
	void clear(uint8_t mode, uint8_t c = 0);
	void invert(boolean inv);
	void contrast(uint8_t contrast);
	void display(void);
	void setCursor(uint8_t x, uint8_t y);
	void pixel(uint8_t x, uint8_t y);
	void pixel(uint8_t x, uint8_t y, uint8_t color, uint8_t mode);
	void line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
	void line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color, uint8_t mode);
	void lineH(uint8_t x, uint8_t y, uint8_t width);
	void lineH(uint8_t x, uint8_t y, uint8_t width, uint8_t color, uint8_t mode);
	void lineV(uint8_t x, uint8_t y, uint8_t height);
	void lineV(uint8_t x, uint8_t y, uint8_t height, uint8_t color, uint8_t mode);
	void rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	void rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color , uint8_t mode);
	void rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	void rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color , uint8_t mode);
	void circle(uint8_t x, uint8_t y, uint8_t radius);
	void circle(uint8_t x, uint8_t y, uint8_t radius, uint8_t color, uint8_t mode);
	void circleFill(uint8_t x0, uint8_t y0, uint8_t radius);
	void circleFill(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t color, uint8_t mode);
	void drawChar(uint8_t x, uint8_t y, uint8_t c);
	void drawChar(uint8_t x, uint8_t y, uint8_t c, uint8_t color, uint8_t mode);
	void drawBitmap(uint8_t * bitArray);
	void drawBitmap(const uint8_t bitArray[]);
	uint8_t getLCDWidth(void);
	uint8_t getLCDHeight(void);
	void setColor(uint8_t color);
	void setDrawMode(uint8_t mode);
	uint8_t *getScreenBuffer(void);

	// Font functions
	uint8_t getFontWidth(void);
	uint8_t getFontHeight(void);
	uint8_t getTotalFonts(void);
	uint8_t getFontType(void);
	uint8_t setFontType(uint8_t type);
	uint8_t getFontStartChar(void);
	uint8_t getFontTotalChar(void);

	// LCD Rotate Scroll functions	
	void scrollRight(uint8_t start, uint8_t stop);
	void scrollLeft(uint8_t start, uint8_t stop);
	void scrollVertRight(uint8_t start, uint8_t stop);
	void scrollVertLeft(uint8_t start, uint8_t stop);
	void scrollStop(void);
	void flipVertical(boolean flip);
	void flipHorizontal(boolean flip);
	
	void setScreenSize(uint8_t lcdWidth, uint8_t lcdHeight);

	void displayOff();
	void displayOn();

	void sleep();
	void wake();

	void printf(const char *format, ...);
	
private:
	uint8_t * screenmemory = NULL;
	uint8_t _lcdHeight, _lcdWidth;
	uint16_t _displayBufferSize;
	uint8_t csPin, dcPin;
	int16_t rstPin;
	uint8_t wrPin, rdPin, dPins[8];
	volatile uint8_t *wrport, *wrreg, *rdport, *rdreg;
	uint8_t wrpinmask, rdpinmask;
	micro_oled_mode interface;
	uint8_t i2c_address;
	volatile uint8_t *ssport, *dcport, *ssreg, *dcreg;	// use volatile because these are fixed location port address
	uint8_t mosipinmask, sckpinmask, sspinmask, dcpinmask;
	uint8_t foreColor,drawMode,fontWidth, fontHeight, fontType, fontStartChar, fontTotalChar, cursorX, cursorY;
	uint16_t fontMapWidth;
	static const unsigned char *fontsPointer[];
	
	// Communication
	void spiTransfer(uint8_t data, uint8_t dc);
	void spiBlockTransfer(uint8_t data, uint16_t len);
	void spiBlockTransfer(uint8_t * data, uint16_t startIdx, uint16_t len);
	void spiSetup();
	void i2cSetup();
	void i2cTransfer(uint8_t data, uint8_t control);
	void i2cBlockTransfer(uint8_t data, uint16_t len);
	void i2cBlockTransfer(uint8_t * data, uint16_t startIdx, uint16_t len);
	void parallelSetup();
	void parallelTransfer(uint8_t data, uint8_t dc);
	void parallelBlockTransfer(uint8_t data, uint16_t len);
	void parallelBlockTransfer(uint8_t * data, uint16_t startIdx, uint16_t len);
};

#endif