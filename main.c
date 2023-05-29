//  STM32F103-CMSIS-I2C-LCD-AHT10

//  Target Microcontroller: STM32F103 (Blue Pill)
//  Mike Shegedin, 05/2023
//
//  Use and I2C LCD controller to drive a 16x2 LCD display module and AHT10
//  temperature and humidity sensor..
//
//  Target I2C devices:
//    I2C 16x2 LCD Driver Module based on the PCF8574.
//    AHT10 Temperature and Humidity Module
//
//  HARDWARE SETUP
//  ==============
//
//             I2C LCD Driver Module
//     ======================================
//     1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
//     ======================================
//                16x2 LCD Module
//
//
//     AHT10 Sensor Module  Blue Pill  RESISTOR  POWER  I2C LCD Driver Module
//     ===================  =========  ========  =====  =====================
//                             GND -------------- GND
//                              5V --------------  5V
//
//         |  VIN ------------------------------- 3.3V
//    AHT10|  GND ------------------------------- GND
//         |  SCL ------------ B11 ---- [10K] --- 3.3V  
//         |  SDA ------------ B10 ---- [10K] --- 3.3V
//
//
//                         |   B6 --------------------------- SCL
//                         |   B7 --------------------------- SDA
//                         |                      5V -------- VCC
//    I2C LCD Driver Module|                     GND -------- GND
//                         |
//                         |                                  LED Jumper -- [1K] --\
//                         |                                                       |
//                         |                                  LED Jumper ----------/
//
 
#include "stm32f103x8.h"                  // Primary CMSIS header file
#include "STM32F103-pause-lib.c"          // pause and delay_us library

I2C_TypeDef *LCD_I2C;                     // Needed for below I2C LCD driver library
#include "STM32F103-CMSIS-I2C-LCD-lib.c"  // I2C LCD driver library


// ============================================================================
// main
// ============================================================================
int
main()
{

  I2C_LCD_init( I2C1 );              // Set the LCD interface to I2C1 and inialize it
  I2C_LCD_cmd( LCD_4B_58F_2L );
  I2C_LCD_cmd( LCD_CLEAR );
  I2C_LCD_cmd( LCD_HOME );
  I2C_LCD_cmd( LCD_ON_BLINK_CURSOR );

  I2C_LCD_puts( "AHT10 Sensor" );
  delay_us( 1E6 ); 
  I2C_LCD_cmd( LCD_2ND_LINE );
  I2C_LCD_puts( "TMP:    HUM:" );



  I2C_LCD_cmd( 0x38 );
  return 1;
}  
