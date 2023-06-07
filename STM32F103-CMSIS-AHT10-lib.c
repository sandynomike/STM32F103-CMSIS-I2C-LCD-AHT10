//  STM32F103-CMSIS-I2C-LCD-AHT10-lib.c
//
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
//  ================================================================= 
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
//  ================================================================= 
//
//
//  ===============================================================================================
//  Routines in this Library
//  ===============================================================================================
//
//  ===============================================================================================
//  void
//  AHT10_init( I2C_TypeDef *this I2C )
//  -----------------------------------------------------------------------------------------------
//  Initialize the specified I2C interface and associate that interface with the subsequent
//  AHT10_ routines. Then initializes the AHT10 unit to its default calibrated values..
//
//  ===============================================================================================
//  void
//  AHT10_readSensorData( uint8_t *data )
//  -----------------------------------------------------------------------------------------------
//  Called with a pointer to an array of at least 6 uint8_t ints.
//  Sends command to trigger a measurement. Then reads in the measured data after 75 ms.
//  The status register is contained in the first byte in the array. The subsequent 5 bytes
//  contain the raw humidity and temperature values.
//  The status register should have a value of 0x19 if a normal temperature/humidity conversion
//  occurred. A status value of 0x99 indicates that there was not sufficient time to complete the
//  measurement.
//  Note that, after powering up the sensor, the AHT10_init routine must be called one time before
//  calling this routine for the first time. Subsequent calls to this routine do not require
//  additional calls to AHT10_init();
//
//  ===============================================================================================
//  void
//  i100toa( int16_t realV, char *thisString )
//  -----------------------------------------------------------------------------------------------
//  i100toa takes a number with 2 decimal places multiplied by 100, and returns a string
//  of the original decimal number rounded to 1 decimal place. For example, if the number
//  in question is 12.36, then 1236 is passed via realV. The resulting string is 12.4,
//  because 12.36 rounds up to 12.4. Negative numbers and more complex rounding work as
//  expected. For example, -2.35, passed as -235, returns "-2.4", and 19.96, passed as 1996,
//  "w" = -2 and "d" = 4. Also, 19.96 would return "w" = 20 and "d" = 0. Note that "w" and "d"
//  are passed by reference.
//
//  ===============================================================================================

#ifndef __STM32F103_CMSIS_AHT10_LIB_C
#define __STM32F103_CMSIS_AHT10_LIB_C

#include <string.h>
#include <stdlib.h>

#include "stm32f103x8.h"              // Primary CMSIS header file
#include "STM32F103-CMSIS-I2C-lib.c"  // I2C library
#include "STM32F103-pause-lib.c"      // pause and delay_us library

I2C_TypeDef *AHT10_I2C;               // Global variable to point to the I2C interface used for
                                      // the I2C AHT10 routines. 

//  Useful constants used with AHT10 sensor routines
#define AHT10_ADD       0x38  // I2C address of AHT10 sensor
#define AHT10_INIT      0xE1  // Initialization command byte
#define AHT10_INIT_D0   0x08  // Initialization 2nd byte to turn on calibration
#define AHT10_INIT_D1   0x00  // Initialization 3rd byte
#define AHT10_TRIG_MEAS 0xAC  // 1st byte to trigger measurement
#define AHT10_TRIG_D0   0x33  // 2nd byte to trigger measurement
#define AHT10_TRIG_D1   0x00  // 3rd byte to trigger measurement
#define AHT10_CHAR_DEG  0xDF  // Degree symbol character
#define AHT10_CHAR_DOT  0xA5  // Center dot Character


//  void
//  AHT10_init( I2C_TypeDef *this I2C )
//  Initialize the specified I2C interface and associate that interface with the subsequent
//  AHT10_ routines. Then initializes the AHT10 unit to its default calibrated values..
void
AHT10_init( I2C_TypeDef *thisI2C )
{
  AHT10_I2C = thisI2C;                       // Associate AHT10_ routines with this I2C interface

  I2C_init(    AHT10_I2C );                  // Initialize this I2C2 interface
  I2C_start(   AHT10_I2C );                  // I2C start
  I2C_address( AHT10_I2C, AHT10_ADD, 0  );   // Send address as a "write" command
  I2C_write(   AHT10_I2C, AHT10_INIT    );   // 0xE1: Init command
  I2C_write(   AHT10_I2C, AHT10_INIT_D0 );   // 0x08: 2nd init byte to set CAL bit
  I2C_write(   AHT10_I2C, AHT10_INIT_D1 );   // 0x00: Finish command with 0-byte
  I2C_stop(    AHT10_I2C );                  // I2C stop
delay_us(40);
}


//  void
//  AHT10_readSensorData( uint8_t *data )
//  Called with a pointer to an array of at least 6 uint8_t ints.
//  Sends command to trigger a measurement. Then reads in the measured data after 75 ms.
//  The status register is contained in the first byte in the array. The subsequent 5 bytes
//  contain the raw humidity and temperature values.
//  The status register should have a value of 0x19 if a normal temperature/humidity conversion
//  occurred. A status value of 0x99 indicates that there was not sufficient time to complete the
//  measurement.
//  Note that, after powering up the sensor, the AHT10_init routine must be called one time before
//  calling this routine for the first time. Subsequent calls to this routine do not require
//  additional calls to AHT10_init();
void
AHT10_readSensorData( uint8_t *data )
{
  I2C_start( AHT10_I2C );                   // Send command to trigger measurement
  I2C_address( AHT10_I2C, AHT10_ADD, 0 );   // Send the address of the sensor and the write command
  I2C_write( AHT10_I2C, AHT10_TRIG_MEAS );  // 0xAC: 1st measurement trigger command byte
  I2C_write( AHT10_I2C, AHT10_TRIG_D0   );  // 0x33: 2nd measurement trigger command byte
  I2C_write( AHT10_I2C, AHT10_TRIG_D1   );  // 0x00: 3rd measurement trigger command byte 
  I2C_stop( AHT10_I2C );                    // Stop this transaction
  
  delay_us( 75e3 );                         // Wait for measurement to complete

  I2C_start( AHT10_I2C );                   // Send command to read data registers
  I2C_address( AHT10_I2C, AHT10_ADD, 1 );   // Send the address of the sensor and the read command
  data[0] = I2C_read( AHT10_I2C, 1 );       // Read Status register, send ACK
  data[1] = I2C_read( AHT10_I2C, 1 );       // Read Humidity [19:12], send ACK
  data[2] = I2C_read( AHT10_I2C, 1 );       // Read Humidity [11:4], send ACK
  data[3] = I2C_read( AHT10_I2C, 1 );       // Read Humidity [3:0] / Temperature [19:16], send ACK
  data[4] = I2C_read( AHT10_I2C, 1 );       // Read Temperature [15:8], send ACK
  data[5] = I2C_read( AHT10_I2C, 0 );       // Read Temperature [7:0], send NAK
  I2C_stop( AHT10_I2C );                    // STop this transaction
delay_us(420);
}


//  void
//  i100toa( int16_t realV, char *thisString )
//  i100toa takes a number with 2 decimal places multiplied by 100, and returns a string
//  of the original decimal number rounded to 1 decimal place. For example, if the number
//  in question is 12.36, then 1236 is passed via realV. The resulting string is 12.4,
//  because 12.36 rounds up to 12.4. Negative numbers and more complex rounding work as
//  expected. For example, -2.35, passed as -235, returns "-2.4", and 19.96, passed as 1996,
//  "w" = -2 and "d" = 4. Also, 19.96 would return "w" = 20 and "d" = 0. Note that "w" and "d"
//  are passed by reference.
void
i100toa( int16_t realV, char *thisString )
{
  char tmpString[2];              // Hold decimal digit and EOL character

  int16_t  x = abs( realV );      // realV = 2596 (target: 26.0C) -> x = 2596
  int16_t  w = x / 100;           // w (whole part) = 25
  int16_t  f = x - ( w * 100 );   // f (fraction)   = 2596 - 2500 = 96
  int16_t  d = f / 10;            // d (decimal)    = 9
  int16_t  r = f - ( d * 10 );    // r (remainder)  = 96 - 90 = 6

  if( r>=5 )                      // Round up if needed
  {
    d = d + 1;                    // d gets rounded up to 10
    if( d >= 10 )                 // If d gets rounded up,
    {
      d = 0;                      // d = 0    zero out d and
      w = w + 1;                  // w = 26   bump up w. 
    }
  }   
  if( realV < 0 )                 // Restore negative sign if original value was negative
    w = 0 - w;
    
  itoa( w, thisString, 10 );        // Make whole part into string
  strcat( thisString, "." );        // Add the decimal point
  itoa( d, tmpString, 10 );         // Make a string for the single decimal place
  strcat( thisString, tmpString );  // Tack on the decimal digit to the rest of the string
}  
 
#endif /* __STM32F103_CMSIS_AHT10-LIB_C */