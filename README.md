# I2C_LCD_Pico
A simple C library for controlling 16x2 character LCD displays via I2C using a PCF8574 I/O expander. Designed for use with Pico family microcontrollers.

## Features
- Supports 16x2 LCD displays over I2C
- Adapted to C from the Arduino LiquidCrystal_I2C C++ library
- User customizable I2C setup to support different pins, or displays with nonstandard I2C addresses
- Simple high level functions for printing text and display control

## Requirements
- Raspberry Pi Pico
- C/C++ SDK for Raspberry Pi Pico
- 16x2 LCD with I2C backpack (ex. PCF8574 I/O expander)

## Installation
1. Clone/copy repo into project directory or download .c/.h files
2. Include "I2C_LCD_Pico.h" in your project
3. Add source file (I2C_LCD_Pico.c) to the "add_executable" line in your CMakeLists.txt file
4. Ensure CMakeLists has required libraries (hardware_i2c and pico_stdlib) in the target_link_libraries line

## Configuration
By default, the library is set with the following parameters:
- I2C address: 0x27
- I2C instance: i2c1
- SDA pin: GPIO 18
- SCL pin: GPIO 19
- Rows: 2
- Columns: 16

These settings can be overridden using the setup_lcd() function.

## TODO
- Add custom character support