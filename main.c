//  STM32F103-CMSIS-I2C-LCD-AHT10

//  Target Microcontroller: STM32F103 (Blue Pill)
//  Mike Shegedin, 05/2023
//
//  Use and I2C LCD controller to drive a 16x2 LCD display module to
//  display temperature and humidity taken from an AHT10 temperature
//  and humidity sensor, which is connected to another I2C interface.
//
//  Target I2C devices:
//    I2C1: 16x2 LCD Driver Module based on the PCF8574.
//    I2C2: AHT10 Temperature and Humidity Module
//
// ================================================================== 
//     ***  HARDWARE SETUP  ***
//
//             I2C LCD Driver Module
//     ======================================
//     1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
//     ======================================
//                16x2 LCD Module
//
//
//     AHT10 Module  Blue Pill  POWER  I2C LCD Driver Module
//     ============  =========  =====  =====================
//                      GND ---- GND
//                       5V ----  5V
//
//         |  VIN -------------- 3.3V
//    AHT10|  GND -------------- GND
//         |  SCL ----- B11
//         |  SDA ----- B10
//
//
//                  |   B6 ----------------- SCL
//                  |   B7 ----------------- SDA
//                  |             5V ------- VCC
//    I2C LCD Module|            GND ------- GND
//                  |
//                  |                        LED Jumper -- [1K] --,
//                  |                                             |
//                  |                        LED Jumper ----------'
//
// ================================================================== 


#include <string.h>
#include <stdlib.h>
#include "stm32f103x8.h"                  // Primary CMSIS header file
#include "STM32F103-pause-lib.c"          // Library for pause and delay_us functions

I2C_TypeDef *LCD_I2C;                     // Needed for below I2C LCD driver library
#include "STM32F103-CMSIS-I2C-LCD-lib.c"  // I2C LCD driver library

I2C_TypeDef *AHT10_I2C;                   // Needed for below AHT10 driver library
#include "STM32F103-CMSIS-AHT10-lib.c"    // AHT10 sensor library


// ============================================================================
// main
// ============================================================================


int
main()
{
  uint8_t  gotData[8];              // Return data from sensor
  char     myString[16];            // Will hold printable strings
  uint32_t tempV, humidV;           // Will hold raw temp and humidity values from sensor
  int16_t  realV;                   // Used in conversion from raw to real data
  
  I2C_LCD_init( I2C1 );             // Set the LCD interface to I2C1 and initialize it
  I2C_LCD_cmd( LCD_4B_58F_2L );     // Get LCD into 4-bit mode
  I2C_LCD_cmd( LCD_ON_NO_CURSOR );  // LCD ON, Cursor OFF
  I2C_LCD_cmd( LCD_CLEAR );         // Clear the LCD screen
  I2C_LCD_cmd( LCD_HOME );          // Set the LCD to the home position

  AHT10_init( I2C2 );               // Initialize AHT10 sensor, set sensor to I2C2

  I2C_LCD_puts( "AHT10" );          // Display title text

  while ( 1 )                              // Repeat this block forever
  {

    AHT10_readSensorData( gotData );       // Get data from sensor
    
    // Display Heartbeat Character
    I2C_LCD_cmd( LCD_1ST_LINE + 15 );       // Position LCD to the end of the 1st line
    if( gotData[0] == 0x19 )                // Display heartbeat
      I2C_LCD_putc( 0xA5 );                 // Display center dot to indicate normal operation 
    else
      I2C_LCD_putc( 'E' );                  // Display E to indicate error
      
    delay_us( 400e3 );                      // Show heartbeat sign for 400 ms
    I2C_LCD_putc( 0x08 );                   // Backspace to clear heartbeat character

    // Separate out humidity and temperature data
    humidV = (gotData[1]<<16            | gotData[2]<<8 | gotData[3] ) >> 4;
    tempV  = ( gotData[3] & 0x0F ) <<16 | gotData[4]<<8 | gotData[5] ;

    realV = (tempV*625) / 32768 -5000;      // Calculate temperature x 100.
                                            // This is done to avoid the overhead of floating math routines.
                                            // tempC = (( tempV * 200 ) / 2^20 ) - 50
                                            // 100 x tempC = tempV  * 20000 ) / 2^20 ) - 5000
                                            // This reduces to: ( tempV * 625 ) / 32768 ) - 5000
    i100toa( realV, myString );             // Convert x100 value to readable string with one decimal place.
    
    I2C_LCD_cmd( LCD_1ST_LINE + 6 );        // Position LCD to display temperature and humidity
    I2C_LCD_puts( myString );               // Display rounded single-decimal-place value
    I2C_LCD_putc( 0xDF );                   // Display the degree character
    I2C_LCD_putc( ' ' );

    realV = humidV/10486;                   // Calculate and display humidity
    itoa( realV, myString, 10 );            // humidity % = ( humidV / 2^20 ) * 100
    I2C_LCD_puts( myString );               // which is equiv. to ( humidV / 10486 ).
    I2C_LCD_puts( "%   " );                 // Display % character and spaces to ensure old display is cleared

    delay_us( 9600e3 );                     // Pause approx. 1:0 s between measurements. Excessive measurements
                                            // can lead to self-heating of the sensor.
//  I2C_LCD_cmd( 0x38 );
  
  }
  return 1;
}  
