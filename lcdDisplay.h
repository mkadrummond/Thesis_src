/*
 * lcdDiplay.h
 *
 *  Created on: 22 Apr 2016
 *      Author: mkadrummond
 */

#ifndef LCDDISPLAY_H_
#define LCDDISPLAY_H_

#include <string>

// LCD Address
#define ADDRESS 0x27

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT  0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0b00000100 // Enable bit
#define Rw 0b00000010 // Read/Write bit
#define Rs 0b00000001 // Register select bit

// might use these for the cursor...?
#define LINE1	0x80
#define LINE2	0xC0
#define LINE3	0x94
#define LINE4 	0xD4

// Functions for the display
void initI2C();
void initLCD();
void lcd_strobe(int);
void lcd_write_four_bits(int);
void lcd_write(int, int);
void write_cmd(int);
void lcd_display_string(std::string, int);
void lcd_display_int(int, int);
void lcd_display_float(float, int);
void clearDisplay();




#endif /* LCDDISPLAY_H_ */
