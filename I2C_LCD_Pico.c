#include "I2C_LCD_Pico.h"

/**
 * @brief Necessary I2C variables. Can be modified using setup_i2c() helper function.
 */
static uint8_t address = 0x27;
static uint8_t rows = 2;
static uint8_t cols = 16;
static i2c_inst_t *I2C = i2c1;
static uint8_t SDA = 18;
static uint8_t SCL = 19;

/**
 * @brief Helper variables, used to condense various display commands
 */
static uint8_t display_function;
static uint8_t display_control;
static uint8_t display_mode;
static uint8_t num_lines;
static uint8_t backlight_val = BACKLIGHT;

/******************* Low Level Commands */
/**
 * @brief Sends a byte of data to the PCF8574 over I2C
 * @param data Byte to be sent to PCF8574
 */
static void i2c_write(uint8_t data) {
    uint8_t byte;

    byte =  data | backlight_val;
    i2c_write_blocking(I2C, address, &byte, 1, false);
}

/**
 * @brief Sends an enable pulse to the LCD
 * @param data Byte to be sent to PCF8574
 */
static void pulse_enable(uint8_t data) {
    i2c_write(data | EN); // EN = 1
    sleep_us(1);
    i2c_write(data & ~EN); // EN = 0
    sleep_us(50);
}

/**
 * @brief Sends a four bit value to the PCF8574
 * @param value 4 bits to be sent to the PCF8574
 */
static void write_4_bits(uint8_t value) {
    i2c_write(value);
    pulse_enable(value);
}

/**
 * @brief Splits a byte into two four bit nibbles and sends them to the LCD
 * @param value Byte to be sent to PCF8574
 * @param mode Byte containing register select (RS) bit (0 for command, RS for data)
 */
static void send(uint8_t value, uint8_t mode) {
    uint8_t high_nibble = value & 0xF0;
    uint8_t low_nibble = (value << 4) & 0xF0;
    write_4_bits(high_nibble | mode);
    write_4_bits(low_nibble | mode);
}

/******************* Mid Level Commands */
void command(uint8_t value) {
    send(value, 0);
}

void begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
    // I2C Initialisation. Using it at 100Khz.
    i2c_init(I2C, 100*1000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SDA);
    gpio_pull_up(SCL);
    
    if (lines > 1) {
        display_function |= LINE_MODE_2;
    }

    num_lines = lines;

    if ((dotsize != 0) && (lines == 1)) {
        display_function |= DOTS_5x10;
    }

    // wait >40us before beginning commands
    sleep_ms(50);

    // pull RS and R/W low to begin commands
    i2c_write(backlight_val);
    sleep_ms(1000);

    // put LCD into 4 bit mode
    write_4_bits(0x03 << 4);
    sleep_us(4500);
    write_4_bits(0x03 << 4);
    sleep_us(4500);
    write_4_bits(0x03 << 4);
    sleep_us(150);
    write_4_bits(0x02 << 4);

    // set # lines, font size, etc.
    command(FUNCTION_SET | display_function);

    // turn on display with no cursor or blinking
    display_control = DISPLAY_ON | CURSOR_OFF | BLINK_OFF;
    display();

    // clear display
    lcd_clear();

    // set default text direction
    display_mode = ENTRY_LEFT | ENTRY_DECREMENT;

    // set entry mode
    command(ENTRY_MODE | display_mode);
    lcd_home();
}

/******************* High Level Commands */

void setup_lcd(uint8_t addr, uint8_t num_rows, uint8_t num_cols, i2c_inst_t *i2c_instance, uint8_t sda_pin, uint8_t scl_pin) {
    address = addr;
    rows = num_rows;
    cols = num_cols;
    I2C = i2c_instance;
    SDA = sda_pin;
    SCL = scl_pin;
}

void lcd_init(void) {
    display_function = BIT_MODE_4 | LINE_MODE_1 | DOTS_5x8;
    begin(cols, rows, DOTS_5x8);
}

void lcd_clear(void) {
    command(CLEAR_DISPLAY);
    sleep_us(2000);
}

void lcd_home(void) {
    command(RETURN_HOME);
    sleep_us(2000);
}

void set_cursor(uint8_t col, uint8_t row) {
    int row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row > rows) {
        row = rows - 1;
    }
    command(DDRAM_ADDR | (col + row_offsets[row]));
}

void no_display(void) {
    display_control &= ~DISPLAY_ON;
    command(DISPLAY_CONTROL | display_control);
}

void display(void) {
    display_control |= DISPLAY_ON;
    command(DISPLAY_CONTROL | display_control);
}

void no_cursor(void) {
    display_control &= ~CURSOR_ON;
    command(DISPLAY_CONTROL | display_control);
}

void cursor(void) {
    display_control |= CURSOR_ON;
    command(DISPLAY_CONTROL | display_control);
}

void no_blink(void) {
    display_control &= ~BLINK_ON;
    command(DISPLAY_CONTROL | display_control);
}

void blink(void) {
    display_control |= BLINK_ON;
    command(DISPLAY_CONTROL | display_control);
}

void scroll_display_left(void) {
    command(CURSOR_SHIFT | DISPLAY_MOVE | MOVE_LEFT);
}

void scroll_display_right(void) {
    command(CURSOR_SHIFT | DISPLAY_MOVE | MOVE_RIGHT);
}

void left_to_right(void) {
    display_mode |= ENTRY_LEFT;
    command(ENTRY_MODE | display_mode);
}

void right_to_left(void) {
    display_mode &= ~ENTRY_LEFT;
    command(ENTRY_MODE | display_mode);
}

void autoscroll(void) {
    display_mode |= ENTRY_INCREMENT;
    command(ENTRY_MODE | display_mode);
}

void no_autoscroll(void) {
    display_mode &= ~ENTRY_INCREMENT;
    command(ENTRY_MODE | display_mode);
}

void no_backlight(void) {
    backlight_val = NO_BACKLIGHT;
    i2c_write(0);
}

void backlight(void) {
    backlight_val = BACKLIGHT;
    i2c_write(0);
}

void print_char(char character) {
    send(character, RS);
}

void print_string(const char * str) {
    int row = 0;
    int col = 0;
    while (*str != '\0') {
        print_char(*str);
        col++;

        if (col >= cols) {
            col = 0;
            row++;
            
            if (row >= rows) {
                row = 0;
                lcd_home();
            } else {
                set_cursor(col, row);
            }
        }
        str++;
    }
}



