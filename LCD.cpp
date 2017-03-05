

#include "LCD.h"
#include <unistd.h>



LCD::LCD()
	:	r(0),
		fd(0)
		{
		}

// clocks EN to latch command
void LCD::lcd_strobe(int data) {
	write_cmd(data | En | LCD_BACKLIGHT);
	usleep(500);
	write_cmd(((data & ~En) | LCD_BACKLIGHT));
	usleep(100);
}

void LCD::lcd_write_four_bits(int data) {
	write_cmd(data | LCD_BACKLIGHT);
    lcd_strobe(data);
}

// write a command to lcd
void LCD::lcd_write(int cmd, int mode) {
	lcd_write_four_bits(mode | (cmd & 0xF0));
    lcd_write_four_bits(mode | ((cmd << 4) & 0xF0));
}

// Write a single command
void LCD::write_cmd(int cmd) {
	r = write(fd, &cmd, 1);
    usleep(100);
}

// put string function
void LCD::lcd_display_string(std::string string, int line) {
	if(line == 1)
		lcd_write(0x80, 0);
	if (line == 2)
		lcd_write(0xC0, 0);
	if (line == 3)
    	lcd_write(0x94, 0);
	if (line == 4)
		lcd_write(0xD4, 0);

	for (int i = 0; i < string.length(); i++) {
		char x = string.at(i);
        lcd_write(x, Rs);
	}
}

// put int function
// the additional lines (5 to 8) are for placing the
// cursor along the right hand side of the display
void LCD::lcd_display_int(int var, int line) {
	if (line == 1)
    	lcd_write(0x80, 0);
	if (line == 2)
    	lcd_write(0xC0, 0);
	if (line == 3)
		lcd_write(0x94, 0);
	if (line == 4)
		lcd_write(0xD4, 0);
	if (line == 5)
    	lcd_write(0x90, 0);
	if (line == 6)
    	lcd_write(0xD0, 0);
	if (line == 7)
    	lcd_write(0xA4, 0);
	if (line == 8)
    	lcd_write(0xE4, 0);

	char buffer[10];

	// These if statements arrange the integer
	// nicely against the edge of the display
	if (var > 999 && var < 10000) {
		snprintf(buffer, 10, "%d", var);
	}
	if (var > 99 && var < 1000) {
		snprintf(buffer, 10, " %d", var);
	}
	if (var > 9 && var < 100) {
		snprintf(buffer, 10, "  %d", var);
	} if (var > -1 && var < 10) {
		snprintf(buffer, 10, "   %d", var);
	}

	lcd_display_string(buffer, line);

}

// put float function
// the additional lines (5 to 8) are for placing the
// cursor along the right hand side of the display
void LCD::lcd_display_float(float var, int line) {
	if (line == 1)
    	lcd_write(0x80, 0);
	if (line == 2)
    	lcd_write(0xC0, 0);
	if (line == 3)
		lcd_write(0x94, 0);
	if (line == 4)
		lcd_write(0xD4, 0);
	if (line == 5)
    	lcd_write(0x90, 0);
	if (line == 6)
    	lcd_write(0xD0, 0);
	if (line == 7)
    	lcd_write(0xA4, 0);
	if (line == 8)
    	lcd_write(0xE4, 0);

	char buffer[10];

	snprintf(buffer, 10, "%.2f", var);

	lcd_display_string(buffer, line);
}

void LCD::initLCD() {

	// initialise LCD
	this->lcd_write(0x03, 0);
	this->lcd_write(0x03, 0);
	this->lcd_write(0x03, 0);
	this->lcd_write(0x02, 0);

	this->lcd_write(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE, 0);
	this->lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);  // | LCD_BLINKON, 0
	this->lcd_write(LCD_CLEARDISPLAY, 0);
	this->lcd_write(LCD_ENTRYMODESET | LCD_ENTRYLEFT, 0);
	usleep(200000);

}

void LCD::clearDisplay() {
	lcd_write(LCD_CLEARDISPLAY, 0);
}

void LCD::displayCursor(bool flag) {
	if(flag) {
		lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);
	} else {
		lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSOROFF, 0);
	}
}
