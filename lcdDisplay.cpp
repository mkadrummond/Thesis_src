#include "lcdDisplay.h"

// Includes for the display and I2C interface
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string>

int r;
int fd;

// Setup i2c
void initI2C() {
	char *dev = "/dev/i2c-2";
	int addr = 0x27;

	fd = open(dev, O_RDWR );

	if(fd < 0) {
		perror("Opening i2c device node\n");
		// return 1;
	}

	r = ioctl(fd, I2C_SLAVE, addr);
	if(r < 0) {
		perror("Selecting i2c device\n");
	}
}

// initialises LCD
void initLCD() {
	lcd_write(0x03, 0);
	lcd_write(0x03, 0);
	lcd_write(0x03, 0);
	lcd_write(0x02, 0);

	lcd_write(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE, 0);
	lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON |LCD_CURSORON, 0);  // | LCD_BLINKON, 0
	lcd_write(LCD_CLEARDISPLAY, 0);
	lcd_write(LCD_ENTRYMODESET | LCD_ENTRYLEFT, 0);
	usleep(200000);
}

// clocks EN to latch command
void lcd_strobe(int data) {
	write_cmd(data | En | LCD_BACKLIGHT);
	usleep(500);
	write_cmd(((data & ~En) | LCD_BACKLIGHT));
	usleep(100);
}

void lcd_write_four_bits(int data) {
	write_cmd(data | LCD_BACKLIGHT);
    lcd_strobe(data);
}

// write a command to lcd
void lcd_write(int cmd, int mode) {
	lcd_write_four_bits(mode | (cmd & 0xF0));
    lcd_write_four_bits(mode | ((cmd << 4) & 0xF0));
}

// Write a single command
void write_cmd(int cmd) {
	r = write(fd, &cmd, 1);
    usleep(100);
}

// put string function
void lcd_display_string(std::string string, int line) {
	if(line == 1)
		lcd_write(0x81, 0);
	if (line == 2)
		lcd_write(0xC1, 0);
	if (line == 3)
    	lcd_write(0x95, 0);
	if (line == 4)
		lcd_write(0xD5, 0);

	for (int i = 0; i < string.length(); i++) {
		char x = string.at(i);
        lcd_write(x, Rs);
	}
}

// display int
// the additional lines (5 to 8) are for placing the
// cursor along the right hand side of the display
void lcd_display_int(int var, int line) {
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

	snprintf(buffer, 10, "%d  ", var);

	lcd_display_string(buffer, line);
}

// display float
// the additional lines (5 to 8) are for placing the
// cursor along the right hand side of the display
void lcd_display_float(float var, int line) {
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

void clearDisplay() {
	lcd_write(LCD_CLEARDISPLAY, 0);
}
