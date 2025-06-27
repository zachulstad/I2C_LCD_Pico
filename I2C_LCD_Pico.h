/**
 * @file I2C_LCD_Pico.h
 * @brief C adaptation of arduino LiquidCrystal_I2C C++ library, for use with Pico
 * 
 * @author Zach Ulstad
 * @date June 26, 2025
 * @version 1.0.0
 */

 #ifndef I2C_LCD_PICO_H
 #define I2C_LCD_PICO_H

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include "pico/stdlib.h"

/**
 * @brief Baud rate for I2C connection
 */
#define BAUD_RATE 100*1000

/**
 * @brief Commands
 */
#define CLEAR_DISPLAY 0x01
#define RETURN_HOME 0x02
#define ENTRY_MODE 0x04
#define DISPLAY_CONTROL 0x08
#define CURSOR_SHIFT 0x10
#define FUNCTION_SET 0x20
#define CGRAM_ADDR 0x40
#define DDRAM_ADDR 0x80

/**
 * @brief Display mode entry flags
 */
#define ENTRY_RIGHT 0x00
#define ENTRY_LEFT 0x02
#define ENTRY_INCREMENT 0x01
#define ENTRY_DECREMENT 0x00

/**
 * @brief Display control flags
 */
#define DISPLAY_ON 0x04
#define DISPLAY_OFF 0x00
#define CURSOR_ON 0x02
#define CURSOR_OFF 0x00
#define BLINK_ON 0x01
#define BLINK_OFF 0x00

/**
 * @brief Cursor/display shift flags
 */
#define DISPLAY_MOVE 0x08
#define CURSOR_MOVE 0x00
#define MOVE_RIGHT 0x04
#define MOVE_LEFT 0x00

/**
 * @brief Function set flags
 */
#define BIT_MODE_8 0x10
#define BIT_MODE_4 0x00
#define LINE_MODE_2 0x08
#define LINE_MODE_1 0x00
#define DOTS_5x10 0x04
#define DOTS_5x8 0x00

/**
 * @brief Backlight control flags
 */
#define BACKLIGHT 0x08
#define NO_BACKLIGHT 0x00

// bit definitions
#define EN 0b00000100 // Enable bit
#define RW 0b00000010 // Read/Write bit
#define RS 0b00000001 // Register select bit

/******************* Mid Level Commands */

/**
 * @brief Sends a command via I2C
 * @param value Command byte
 */
void command(uint8_t value);

/**
 * @brief Helper function for lcd_init()
 * @param cols Number of columns on display
 * @param rows Number of rows on display
 * @param dotsize Dot dimensions for each character
 */
void begin(uint8_t cols, uint8_t rows, uint8_t dotsize);

/******************* High Level Commands */

/**
 * @brief Helper function to set up I2C and LCD information if using different pins or displays
 * @param addr I2C address for display
 * @param num_rows Number of rows on LCD
 * @param num_cols Number of cols on LCD
 * @param i2c_instance I2C instance used on Pico (i2c1 or i2c0)
 * @param sda_pin Pin used for SDA
 * @param scl_pin Pin used for SCL
 */
void setup_lcd(uint8_t addr, uint8_t num_rows, uint8_t num_cols, i2c_inst_t *i2c_instance, uint8_t sda_pin, uint8_t scl_pin);

/**
 * @brief Initializes the display
 */
void lcd_init(void);

/**
 * @brief Clears the display
 */
void lcd_clear(void);

/**
 * @brief Returns the cursor to default position
 */
void lcd_home(void);

/**
 * @brief Turns the display off
 */
void no_display(void);

/**
 * @brief Turns the display on
 */
void display(void);

/**
 * @brief Turns off the cursor blink
 */
void no_blink(void);

/**
 * @brief Turns on the cursor blink
 */
void blink(void);

/**
 * @brief Turns off underline cursor
 */
void no_cursor(void);

/**
 * @brief Turns on underline cursor
 */
void cursor(void);

/**
 * @brief Scrolls display to the left
 */
void scroll_display_left(void);

/**
 * @brief Scrolls display to the right
 */
void scroll_display_right(void);

/**
 * @brief Sets text flow from left to right
 */
void left_to_right(void);

/**
 * @brief Sets text flow from right to left
 */
void right_to_left(void);

/**
 * @brief Turns the backlight off
 */
void no_backlight(void);

/**
 * @brief Turns the backlight on
 */
void backlight(void);

/**
 * @brief This "right justifies" text from the cursor
 */
void autoscroll(void);

/**
 * @brief This "left justifies" text from the cursor
 */
void no_autoscroll(void);

/**
 * @brief Sets cursor location
 * @param col Destination column for cursor
 * @param row Destination row for cursor
 */
void set_cursor(uint8_t col, uint8_t row);

/**
 * @brief Prints a character to the display
 * @param character Character to print to display
 */
void print_char(char character);

/**
 * @brief Prints a string to the display
 * @param character String to print to display
 */
void print_string(const char * str);
 #endif // I2C_LCD_PICO_H