/******************************************************************************
SFE_MicroOLED.cpp
Main source code for the MicroOLED Arduino Library

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

#include <Arduino.h>
#include <stdarg.h>
#if defined(__AVR__) || defined(__arm__)
	#include <avr/pgmspace.h>
#else
	#include <pgmspace.h>
#endif
#include <SFE_MicroOLED.h>

#ifndef _BV
#define _BV(x) (1<<x)
#endif

#include <Wire.h>
#include <SPI.h>

// The 31x48 font is handy, but uses a big chunk of flash memory - about 7k.
// If you want to use font 4 in your sketch, uncomment out the line below:
//#define INCLUDE_LARGE_FONTS

// This fixed ugly GCC warning "only initialized variables can be placed into program memory area"
#if defined(__AVR__)
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))
#endif

// Add header of the fonts here.  Remove as many as possible to conserve FLASH memory.
#include "util/font5x7.h"
#include "util/font8x16.h"
#include "util/7segment.h"
#ifdef INCLUDE_LARGE_FONTS
#include "util/fontlargenumber.h"
#include "util/fontlargeletter31x48.h"
#endif

#define SFEOLED_NO_LOGO // uncomment to save 384 bytes of flash memory
#ifndef SFEOLED_NO_LOGO
#include "util/sfe_logo.h"
#endif

// Change the total fonts included
#ifdef INCLUDE_LARGE_FONTS
#define SFEOLED_TOTALFONTS		5
#else
#define SFEOLED_TOTALFONTS		3
#endif

// Add the font name as declared in the header file.  Remove as many as possible to conserve FLASH memory.
const unsigned char *MicroOLED::fontsPointer[]={
	font5x7
	,font8x16
	,sevensegment
#ifdef INCLUDE_LARGE_FONTS
	,fontlargenumber
	,fontlargeletter31x48
#endif
};

void MicroOLED::setScreenSize(uint8_t lcdWidth, uint8_t lcdHeight)
{
	_lcdHeight = lcdHeight;
	_lcdWidth = lcdWidth;
	_displayBufferSize = _lcdHeight*_lcdWidth/8;
	
	screenmemory = (uint8_t*)calloc(_displayBufferSize, sizeof(uint8_t));
#ifndef SFEOLED_NO_LOGO	
	if (_displayBufferSize == 384) // Initial size is 384
	{
		// I havent tried this so it probably wont work...
		memcpy_P(screenmemory, sfe_logo, _displayBufferSize*sizeof(uint8_t));
	}
#endif
}

void MicroOLED::displayOff()
{
	command(SFEOLED_DISPLAYOFF);			// 0xAE
}

void MicroOLED::displayOn()
{
	command(SFEOLED_DISPLAYON);	
}

void MicroOLED::sleep()
{
	command(SFEOLED_DISPLAYOFF);
	command(SFEOLED_CHARGEPUMP);			// turn off charge pump
	command(0x10);
	//delay(100); 					// power stabilisation delay
	// power down Vbat
	// delay(50);
	// power down Vdd
}

void MicroOLED::wake()
{
	command(SFEOLED_CHARGEPUMP);			// 0x8D - enable charge pump
	command(0x14);	
	command(SFEOLED_DISPLAYON);
	//delay(100);

	display();
}

void MicroOLED::printf(const char *format, ...)
{
    static char buffer[128]; // assume max 128px wide screen
    
    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);
    
    char *c = (char *)&buffer;
    while (*c != 0)
    {
        write(*c++);//putc2(*c++);
    }
}

/** \brief MicroOLED Constructor -- SPI Mode

	Setup the MicroOLED class, configure the display to be controlled via a
	SPI interface.
*/
MicroOLED::MicroOLED(uint8_t rst, uint8_t dc, uint8_t cs)
{
	setScreenSize(SFEOLED_LCDWIDTH, SFEOLED_LCDHEIGHT);
	
	// Assign each of the parameters to a private class variable.
	rstPin = rst;
	dcPin = dc;
	csPin = cs;
	interface = COMM_MODE_SPI;	// Set interface mode to SPI
}

/** \brief MicroOLED Constructor -- I2C Mode

	Setup the MicroOLED class, configure the display to be controlled via a
	I2C interface.
*/
MicroOLED::MicroOLED(uint8_t rst, uint8_t dc)
{
	setScreenSize(SFEOLED_LCDWIDTH, SFEOLED_LCDHEIGHT);
	
	rstPin = rst;	// Assign reset pin to private class variable
	interface = COMM_MODE_I2C;	// Set interface to I2C
	// Set the I2C Address based on whether DC is high (1) or low (0).
	// The pin is pulled low by default, so if it's not explicitly set to
	// 1, just default to 0.
	if (dc == 1)
		i2c_address = I2C_ADDRESS_SA0_1;
	else
		i2c_address = I2C_ADDRESS_SA0_0;
}

/** \brief MicroOLED Constructor -- Parallel Mode

	Setup the MicroOLED class, configure the display to be controlled via a
	parallel interface.
*/
MicroOLED::MicroOLED(uint8_t rst, uint8_t dc, uint8_t cs, uint8_t wr, uint8_t rd,
					uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
					uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
	setScreenSize(SFEOLED_LCDWIDTH, SFEOLED_LCDHEIGHT);

	interface = COMM_MODE_PARALLEL;	// Set to parallel mode
	// Assign pin parameters to private class variables.
	rstPin = rst;
	dcPin = dc;
	csPin = cs;
	wrPin = wr;
	rdPin = rd;
	dPins[0] = d0; dPins[1] = d1; dPins[2] = d2; dPins[3] = d3;
	dPins[4] = d4; dPins[5] = d5; dPins[6] = d6; dPins[7] = d7;
}

/** \brief Initialisation of MicroOLED Library.

    Setup IO pins for SPI port then send initialisation commands to the SSD1306 controller inside the OLED.
*/
void MicroOLED::begin()
{
	// default 5x7 font
	setFontType(0);
	setColor(SFEOLED_WHITE);
	setDrawMode(SFEOLED_NORM);
	setCursor(0,0);

	// Set up the selected interface:
	if (interface == COMM_MODE_SPI)
		spiSetup();
	else if (interface == COMM_MODE_I2C)
		i2cSetup();
	else if (interface == COMM_MODE_PARALLEL)
		parallelSetup();

	// Display reset routine
	if (rstPin >= 0)
	{
		pinMode(rstPin, OUTPUT);	// Set RST pin as OUTPUT
		digitalWrite(rstPin, HIGH);	// Initially set RST HIGH
		delay(5);	// VDD (3.3V) goes high at start, lets just chill for 5 ms
		digitalWrite(rstPin, LOW);	// Bring RST low, reset the display
		delay(10);	// wait 10ms
		digitalWrite(rstPin, HIGH);	// Set RST HIGH, bring out of reset
	}

	// Display Init sequence for 64x48 OLED module
	command(SFEOLED_DISPLAYOFF);			// 0xAE

	command(SFEOLED_SETDISPLAYCLOCKDIV);	// 0xD5
	command(0x80);					// the suggested ratio 0x80

	command(SFEOLED_SETMULTIPLEX);			// 0xA8
	command(_lcdHeight - 1); 		// duty cycle = 1/(_lcdHeight - 1)

	command(SFEOLED_SETDISPLAYOFFSET);		// 0xD3
	command(0x00);					// no offset

	command(SFEOLED_SETSTARTLINE | 0x0);	// 0x40 - line 0

	command(SFEOLED_CHARGEPUMP);			// 0x8D - enable charge pump
	command(0x14);

	command(SFEOLED_MEMORYMODE);            // 0x20
  	command(0x00);                  // 0x0 act like ks0108

	command(SFEOLED_SEGREMAP | 0x1);		// 0xA0 + 1
	
	command(SFEOLED_COMSCANDEC);			// 0xC8

	command(SFEOLED_SETCOMPINS);			// 0xDA - alter this if screen is interleaved or printed on the wrong lines
	command(0x12); // 128x32=0x02, 128x64=0x12, 96x16=0x02

	command(SFEOLED_SETCONTRAST);			// 0x81
	command(0x8F);					// 0x4F in ER-OLED0.49.1_Series_Datasheet.pdf

	command(SFEOLED_SETPRECHARGE);			// 0xD9
	command(0x1F);//F1);					// 0x1F in ER-OLED0.49.1_Series_Datasheet.pdf

	command(SFEOLED_SETVCOMDESELECT);		// 0xDB
	command(0x40);

	command(SFEOLED_DISPLAYALLONRESUME);	// 0xA4

	command(SFEOLED_NORMALDISPLAY);			// 0xA6 - optional
	command(SFEOLED_DEACTIVATESCROLL);		// 0x2E - optional

	clear(SFEOLED_ALL);

	command(SFEOLED_DISPLAYON);				//--turn on oled panel
	
	delay(100);
}

/** \brief Send the display a command byte

    Send a command via SPI, I2C or parallel	to SSD1306 controller.
	For SPI we set the DC and CS pins here, and call spiTransfer(byte)
	to send the data. For I2C and Parallel we use the write functions
	defined in hardware.cpp to send the data.
*/
void MicroOLED::command(uint8_t c) {

	if (interface == COMM_MODE_SPI)
	{
		spiTransfer(c, LOW);	// DC pin LOW for a command
	}
	else if (interface == COMM_MODE_I2C)
	{
		// Write to our address, make sure it knows we're sending a
		// command:
		i2cTransfer(c, I2C_COMMAND);
	}
	else if (interface == COMM_MODE_PARALLEL)
	{
		// Write the byte to our parallel interface. Set DC LOW.
		parallelTransfer(c, LOW);
	}
}

/** \brief Send the display a data byte

    Send a data byte via SPI, I2C or parallel to SSD1306 controller.
	For SPI we set the DC and CS pins here, and call spiTransfer(byte)
	to send the data. For I2C and Parallel we use the write functions
	defined in hardware.cpp to send the data.
*/
void MicroOLED::data(uint8_t c) {

	if (interface == COMM_MODE_SPI)
	{
		spiTransfer(c, HIGH);	// DC pin HIGH for a data byte
	}
	else if (interface == COMM_MODE_I2C)
	{
		// Write to our address, make sure it knows we're sending a
		// data byte:
		i2cTransfer(c, I2C_DATA);
	}
	else if (interface == COMM_MODE_PARALLEL)
	{
		// Write the byte to our parallel interface. Set DC HIGH.
		parallelTransfer(c, HIGH);
	}
}

void MicroOLED::dataBlock(uint8_t c, uint16_t len)
{
	switch(interface)
	{
		case COMM_MODE_SPI:
			spiBlockTransfer(c, len);
			break;
		case COMM_MODE_I2C:
			i2cBlockTransfer(c, len);
			break;
		case COMM_MODE_PARALLEL:
			parallelBlockTransfer(c, len);
			break;
	}
}

void MicroOLED::dataBlock(uint8_t * c, uint16_t startIdx, uint16_t len)
{
	switch(interface)
	{
		case COMM_MODE_SPI:
			spiBlockTransfer(c, startIdx, len);
			break;
		case COMM_MODE_I2C:
			i2cBlockTransfer(c, startIdx, len);
			break;
		case COMM_MODE_PARALLEL:
			parallelBlockTransfer(c, startIdx, len);
			break;
	}
}

/** \brief Set SSD1306 page address.

    Send page address command and address to the SSD1306 OLED controller.
*/
void MicroOLED::setPageAddress(uint8_t add)
{
	add=0xb0|add;
	command(add);
}

/** \brief Set SSD1306 column address.

    Send column address command and address to the SSD1306 OLED controller.
*/
void MicroOLED::setColumnAddress(uint8_t add)
{
	command((0x10|(add>>4))+0x02);
	command((0x0f&add));
}

/** \brief Clear screen buffer or SSD1306's memory.

    To clear GDRAM inside the LCD controller, pass in the variable mode = ALL with c character and to clear screen page buffer, pass in the variable mode = PAGE with c character (default == 0).
*/
void MicroOLED::clear(uint8_t mode, uint8_t c) {
	//	uint8_t page=6, col=0x40;
	if (mode==SFEOLED_ALL) {
		if (_lcdWidth > 64)
		{
			command(SFEOLED_COLUMNADDR);
			command(0x00); // start page Address
			command(_lcdWidth-1); // end page Address

			command(SFEOLED_PAGEADDR);
			command(0x00); // start page Address
			command((_lcdHeight/8)-1); // end page Address

			if (interface == COMM_MODE_I2C)
			{
				uint16_t BLOCK_SIZE = 16;
				
				for (uint16_t i=0; i<_displayBufferSize; i+=BLOCK_SIZE)
				{
					dataBlock(c, BLOCK_SIZE);
				}
			}
			else
			{
				dataBlock(c, _displayBufferSize);
			}
		}
		else
		{
			for (uint16_t i=0;i<_lcdHeight/8; i++)
			{
				setPageAddress(i);
				setColumnAddress(0);
				dataBlock(c, _lcdWidth);
			}
		}
	}
	else
	{
		memset(screenmemory,c,_displayBufferSize);
	}
}

/** \brief Invert display.

    The WHITE color of the display will turn to BLACK and the BLACK will turn to WHITE.
*/
void MicroOLED::invert(boolean inv) {
	if (inv)
	command(SFEOLED_INVERTDISPLAY); // 0xA7
	else
	command(SFEOLED_NORMALDISPLAY); // 0xA6
}

/** \brief Set contrast.

    OLED contract value from 0 to 255. Note: Contrast level is not very obvious.
*/
void MicroOLED::contrast(uint8_t contrast) {
	command(SFEOLED_SETCONTRAST);			// 0x81
	command(contrast);
}

/** \brief Transfer display memory.

    Bulk move the screen buffer to the SSD1306 controller's memory so that images/graphics drawn on the screen buffer will be displayed on the OLED.
*/
void MicroOLED::display(void) {
	if (_lcdWidth > 64)
	{
		command(SFEOLED_COLUMNADDR);
		command(0x00); // start page Address
		command(_lcdWidth-1); // end page Address

		command(SFEOLED_PAGEADDR);
		command(0x00); // start page Address
		command((_lcdHeight/8)-1); // end page Address

		if (interface == COMM_MODE_I2C)
		{
			const uint16_t BLOCK_SIZE = 16;
			
			for (uint16_t i=0; i<_displayBufferSize; i+=BLOCK_SIZE)
			{
				dataBlock(screenmemory, i, BLOCK_SIZE);
			}
		}
		else
		{
			dataBlock(screenmemory, 0, _displayBufferSize);
		}
	}
	else
	{
		if (interface == COMM_MODE_I2C)
		{
			for (uint8_t r=0; r<_lcdHeight/8; r++)
			{
				setPageAddress(r);
				setColumnAddress(0);
				
				const uint16_t BLOCK_SIZE = 16;
				
				for (uint16_t c=r*_lcdWidth; c<(r+1)*_lcdWidth; c+=BLOCK_SIZE)
				{
					dataBlock(screenmemory, c, BLOCK_SIZE);
				}
				yield();
			}
		}
		else
		{
			for (uint8_t r=0; r<_lcdHeight/8; r++)
			{
				setPageAddress(r);
				setColumnAddress(0);

				dataBlock(screenmemory, r*_lcdWidth, _lcdWidth);
			}
		}
	}
}

/** \brief Override Arduino's Print.

    Arduino's print overridden so that we can use uView.print().
*/
size_t MicroOLED::write(uint8_t c) {
	if (c == '\n') {
		cursorY += fontHeight;
		cursorX  = 0;
	} else if (c == '\r') {
		// skip
	} else {
		drawChar(cursorX, cursorY, c, foreColor, drawMode);
		cursorX += fontWidth+1;
		if ((cursorX > (_lcdWidth - fontWidth))) {
			cursorY += fontHeight;
			cursorX = 0;
		}
	}

	return 1;
}

/** \brief Set cursor position.

MicroOLED's cursor position to x,y.
*/
void MicroOLED::setCursor(uint8_t x, uint8_t y) {
	cursorX=x;
	cursorY=y;
}

/** \brief Draw pixel.

Draw pixel using the current fore color and current draw mode in the screen buffer's x,y position.
*/
void MicroOLED::pixel(uint8_t x, uint8_t y) {
	pixel(x,y,foreColor,drawMode);
}

/** \brief Draw pixel with color and mode.

Draw color pixel in the screen buffer's x,y position with NORM or XOR draw mode.
*/
void MicroOLED::pixel(uint8_t x, uint8_t y, uint8_t color, uint8_t mode) {
	if ((x<0) ||  (x>=_lcdWidth) || (y<0) || (y>=_lcdHeight))
	return;

	if (mode==SFEOLED_XOR) {
		if (color==SFEOLED_WHITE)
		screenmemory[x+ (y/8)*_lcdWidth] ^= _BV((y%8));
	}
	else {
		if (color==SFEOLED_WHITE)
		screenmemory[x+ (y/8)*_lcdWidth] |= _BV((y%8));
		else
		screenmemory[x+ (y/8)*_lcdWidth] &= ~_BV((y%8));
	}
}

/** \brief Draw line.

Draw line using current fore color and current draw mode from x0,y0 to x1,y1 of the screen buffer.
*/
void MicroOLED::line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
	line(x0,y0,x1,y1,foreColor,drawMode);
}

/** \brief Draw line with color and mode.

Draw line using color and mode from x0,y0 to x1,y1 of the screen buffer.
*/
void MicroOLED::line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color, uint8_t mode) {
	uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	uint8_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int8_t err = dx / 2;
	int8_t ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;}

	for (; x0<x1; x0++) {
		if (steep) {
			pixel(y0, x0, color, mode);
		} else {
			pixel(x0, y0, color, mode);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}

/** \brief Draw horizontal line.

Draw horizontal line using current fore color and current draw mode from x,y to x+width,y of the screen buffer.
*/
void MicroOLED::lineH(uint8_t x, uint8_t y, uint8_t width) {
	line(x,y,x+width,y,foreColor,drawMode);
}

/** \brief Draw horizontal line with color and mode.

Draw horizontal line using color and mode from x,y to x+width,y of the screen buffer.
*/
void MicroOLED::lineH(uint8_t x, uint8_t y, uint8_t width, uint8_t color, uint8_t mode) {
	line(x,y,x+width,y,color,mode);
}

/** \brief Draw vertical line.

Draw vertical line using current fore color and current draw mode from x,y to x,y+height of the screen buffer.
*/
void MicroOLED::lineV(uint8_t x, uint8_t y, uint8_t height) {
	line(x,y,x,y+height,foreColor,drawMode);
}

/** \brief Draw vertical line with color and mode.

Draw vertical line using color and mode from x,y to x,y+height of the screen buffer.
*/
void MicroOLED::lineV(uint8_t x, uint8_t y, uint8_t height, uint8_t color, uint8_t mode) {
	line(x,y,x,y+height,color,mode);
}

/** \brief Draw rectangle.

Draw rectangle using current fore color and current draw mode from x,y to x+width,y+height of the screen buffer.
*/
void MicroOLED::rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height) {
	rect(x,y,width,height,foreColor,drawMode);
}

/** \brief Draw rectangle with color and mode.

Draw rectangle using color and mode from x,y to x+width,y+height of the screen buffer.
*/
void MicroOLED::rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color , uint8_t mode) {
	uint8_t tempHeight;

	lineH(x,y, width, color, mode);
	lineH(x,y+height-1, width, color, mode);

	tempHeight=height-2;

	// skip drawing vertical lines to avoid overlapping of pixel that will
	// affect XOR plot if no pixel in between horizontal lines
	if (tempHeight<1) return;

	lineV(x,y+1, tempHeight, color, mode);
	lineV(x+width-1, y+1, tempHeight, color, mode);
}

/** \brief Draw filled rectangle.

Draw filled rectangle using current fore color and current draw mode from x,y to x+width,y+height of the screen buffer.
*/
void MicroOLED::rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height) {
	rectFill(x,y,width,height,foreColor,drawMode);
}

/** \brief Draw filled rectangle with color and mode.

Draw filled rectangle using color and mode from x,y to x+width,y+height of the screen buffer.
*/
void MicroOLED::rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color , uint8_t mode) {
	// TODO - need to optimise the memory map draw so that this function will not call pixel one by one
	for (uint16_t i=x; i<x+width;i++) {
		lineV(i,y, height, color, mode);
	}
}

/** \brief Draw circle.

    Draw circle with radius using current fore color and current draw mode at x,y of the screen buffer.
*/
void MicroOLED::circle(uint8_t x0, uint8_t y0, uint8_t radius) {
	circle(x0,y0,radius,foreColor,drawMode);
}

/** \brief Draw circle with color and mode.

Draw circle with radius using color and mode at x,y of the screen buffer.
*/
void MicroOLED::circle(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t color, uint8_t mode) {
	//TODO - find a way to check for no overlapping of pixels so that XOR draw mode will work perfectly
	int8_t f = 1 - radius;
	int8_t ddF_x = 1;
	int8_t ddF_y = -2 * radius;
	int8_t x = 0;
	int8_t y = radius;

	pixel(x0, y0+radius, color, mode);
	pixel(x0, y0-radius, color, mode);
	pixel(x0+radius, y0, color, mode);
	pixel(x0-radius, y0, color, mode);

	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		pixel(x0 + x, y0 + y, color, mode);
		pixel(x0 - x, y0 + y, color, mode);
		pixel(x0 + x, y0 - y, color, mode);
		pixel(x0 - x, y0 - y, color, mode);

		pixel(x0 + y, y0 + x, color, mode);
		pixel(x0 - y, y0 + x, color, mode);
		pixel(x0 + y, y0 - x, color, mode);
		pixel(x0 - y, y0 - x, color, mode);

	}
}

/** \brief Draw filled circle.

    Draw filled circle with radius using current fore color and current draw mode at x,y of the screen buffer.
*/
void MicroOLED::circleFill(uint8_t x0, uint8_t y0, uint8_t radius) {
	circleFill(x0,y0,radius,foreColor,drawMode);
}

/** \brief Draw filled circle with color and mode.

    Draw filled circle with radius using color and mode at x,y of the screen buffer.
*/
void MicroOLED::circleFill(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t color, uint8_t mode) {
	// TODO - - find a way to check for no overlapping of pixels so that XOR draw mode will work perfectly
	int8_t f = 1 - radius;
	int8_t ddF_x = 1;
	int8_t ddF_y = -2 * radius;
	int8_t x = 0;
	int8_t y = radius;

	// Temporary disable fill circle for XOR mode.
	if (mode==SFEOLED_XOR) return;

	for (uint8_t i=y0-radius; i<=y0+radius; i++) {
		pixel(x0, i, color, mode);
	}

	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		for (uint8_t i=y0-y; i<=y0+y; i++) {
			pixel(x0+x, i, color, mode);
			pixel(x0-x, i, color, mode);
		}
		for (uint8_t i=y0-x; i<=y0+x; i++) {
			pixel(x0+y, i, color, mode);
			pixel(x0-y, i, color, mode);
		}
	}
}

/** \brief Get LCD height.

    The height of the LCD return as byte.
*/
uint8_t MicroOLED::getLCDHeight(void) {
	return _lcdHeight;
}

/** \brief Get LCD width.

    The width of the LCD return as byte.
*/
uint8_t MicroOLED::getLCDWidth(void) {
	return _lcdWidth;
}

/** \brief Get font width.

    The cucrrent font's width return as byte.
*/
uint8_t MicroOLED::getFontWidth(void) {
	return fontWidth;
}

/** \brief Get font height.

    The current font's height return as byte.
*/
uint8_t MicroOLED::getFontHeight(void) {
	return fontHeight;
}

/** \brief Get font starting character.

    Return the starting ASCII character of the currnet font, not all fonts start with ASCII character 0. Custom fonts can start from any ASCII character.
*/
uint8_t MicroOLED::getFontStartChar(void) {
	return fontStartChar;
}

/** \brief Get font total characters.

    Return the total characters of the current font.
*/
uint8_t MicroOLED::getFontTotalChar(void) {
	return fontTotalChar;
}

/** \brief Get total fonts.

    Return the total number of fonts loaded into the MicroOLED's flash memory.
*/
uint8_t MicroOLED::getTotalFonts(void) {
	return SFEOLED_TOTALFONTS;
}

/** \brief Get font type.

    Return the font type number of the current font.
*/
uint8_t MicroOLED::getFontType(void) {
	return fontType;
}

/** \brief Set font type.

    Set the current font type number, ie changing to different fonts base on the type provided.
*/
uint8_t MicroOLED::setFontType(uint8_t type) {
	if ((type>=SFEOLED_TOTALFONTS) || (type<0))
	return false;

	fontType=type;
	fontWidth=pgm_read_byte(fontsPointer[fontType]+0);
	fontHeight=pgm_read_byte(fontsPointer[fontType]+1);
	fontStartChar=pgm_read_byte(fontsPointer[fontType]+2);
	fontTotalChar=pgm_read_byte(fontsPointer[fontType]+3);
	fontMapWidth=(pgm_read_byte(fontsPointer[fontType]+4)*100)+pgm_read_byte(fontsPointer[fontType]+5); // two bytes values into integer 16
	return true;
}

/** \brief Set color.

    Set the current draw's color. Only WHITE and BLACK available.
*/
void MicroOLED::setColor(uint8_t color) {
	foreColor=color;
}

/** \brief Set draw mode.

    Set current draw mode with NORM or XOR.
*/
void MicroOLED::setDrawMode(uint8_t mode) {
	drawMode=mode;
}

/** \brief Draw character.

    Draw character c using current color and current draw mode at x,y.
*/
void  MicroOLED::drawChar(uint8_t x, uint8_t y, uint8_t c) {
	drawChar(x,y,c,foreColor,drawMode);
}

/** \brief Draw character with color and mode.

    Draw character c using color and draw mode at x,y.
*/
void  MicroOLED::drawChar(uint8_t x, uint8_t y, uint8_t c, uint8_t color, uint8_t mode) {
	// TODO - New routine to take font of any height, at the moment limited to font height in multiple of 8 pixels

	uint8_t rowsToDraw,row, tempC;
	uint8_t i,j,temp;
	uint16_t charPerBitmapRow,charColPositionOnBitmap,charRowPositionOnBitmap,charBitmapStartPosition;

	if ((c<fontStartChar) || (c>(fontStartChar+fontTotalChar-1)))		// no bitmap for the required c
	return;

	tempC=c-fontStartChar;

	// each row (in datasheet is call page) is 8 bits high, 16 bit high character will have 2 rows to be drawn
	rowsToDraw=fontHeight/8;	// 8 is LCD's page size, see SSD1306 datasheet
	if (rowsToDraw<=1) rowsToDraw=1;

	// the following draw function can draw anywhere on the screen, but SLOW pixel by pixel draw
	if (rowsToDraw==1) {
		for  (i=0;i<fontWidth+1;i++) {
			if (i==fontWidth) // this is done in a weird way because for 5x7 font, there is no margin, this code add a margin after col 5
			temp=0;
			else
			temp=pgm_read_byte(fontsPointer[fontType]+SFEOLED_FONTHEADERSIZE+(tempC*fontWidth)+i);

			for (j=0;j<8;j++) {			// 8 is the LCD's page height (see datasheet for explanation)
				if (temp & 0x1) {
					pixel(x+i, y+j, color,mode);
				}
				else {
					pixel(x+i, y+j, !color,mode);
				}

				temp >>=1;
			}
		}
		return;
	}

	// font height over 8 bit
	// take character "0" ASCII 48 as example
	charPerBitmapRow=fontMapWidth/fontWidth;  // 256/8 =32 char per row
	charColPositionOnBitmap=tempC % charPerBitmapRow;  // =16
	charRowPositionOnBitmap=int(tempC/charPerBitmapRow); // =1
	charBitmapStartPosition=(charRowPositionOnBitmap * fontMapWidth * (fontHeight/8)) + (charColPositionOnBitmap * fontWidth) ;

	// each row on LCD is 8 bit height (see datasheet for explanation)
	for(row=0;row<rowsToDraw;row++) {
		for (i=0; i<fontWidth;i++) {
			temp=pgm_read_byte(fontsPointer[fontType]+SFEOLED_FONTHEADERSIZE+(charBitmapStartPosition+i+(row*fontMapWidth)));
			for (j=0;j<8;j++) {			// 8 is the LCD's page height (see datasheet for explanation)
				if (temp & 0x1) {
					pixel(x+i,y+j+(row*8), color, mode);
				}
				else {
					pixel(x+i,y+j+(row*8), !color, mode);
				}
				temp >>=1;
			}
		}
	}

}

/** \brief Stop scrolling.

    Stop the scrolling of graphics on the OLED.
*/
void MicroOLED::scrollStop(void){
	command(SFEOLED_DEACTIVATESCROLL);
}

/** \brief Right scrolling.

Set row start to row stop on the OLED to scroll right. Refer to http://learn.microview.io/intro/general-overview-of-microview.html for explanation of the rows.
*/
void MicroOLED::scrollRight(uint8_t start, uint8_t stop){
	if (stop<start)		// stop must be larger or equal to start
	return;
	scrollStop();		// need to disable scrolling before starting to avoid memory corrupt
	command(SFEOLED_RIGHTHORIZONTALSCROLL);
	command(0x00);
	command(start);
	command(0x7);		// scroll speed frames , TODO
	command(stop);
	command(0x00);
	command(0xFF);
	command(SFEOLED_ACTIVATESCROLL);
}

/** \brief Vertical flip.

Flip the graphics on the OLED vertically.
*/
void MicroOLED::flipVertical(boolean flip) {
	if (flip) {
		command(SFEOLED_COMSCANINC);
	}
	else {
		command(SFEOLED_COMSCANDEC);
	}
}

/** \brief Horizontal flip.

    Flip the graphics on the OLED horizontally.
*/
void MicroOLED::flipHorizontal(boolean flip) {
	if (flip) {
		command(SFEOLED_SEGREMAP | 0x0);
	}
	else {
		command(SFEOLED_SEGREMAP | 0x1);
	}
}

/*
	Return a pointer to the start of the RAM screen buffer for direct access.
*/
uint8_t *MicroOLED::getScreenBuffer(void) {
	return screenmemory;
}

/*
	Draw Bitmap image on screen. The array for the bitmap can be stored in the Arduino file, so user don't have to mess with the library files.
	To use, create uint8_t array that is 64x48 pixels (384 bytes). Then call .drawBitmap and pass it the array.
*/
void MicroOLED::drawBitmap(uint8_t * bitArray)
{
	memcpy(screenmemory, bitArray, _displayBufferSize*sizeof(uint8_t));
}


/*
	Draw Bitmap image on screen. The array for the bitmap can be stored in the Arduino file, so user don't have to mess with the library files.
	To use, create uint8_t array that is 64x48 pixels (384 bytes). Then call .drawBitmap and pass it the array.
*/
void MicroOLED::drawBitmap(const uint8_t bitArray[])
{
	memcpy(screenmemory, bitArray, _displayBufferSize*sizeof(uint8_t));
}