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
// ============================================================================ 
//     ***  HARDWARE SETUP  ***
//
//             I2C LCD Driver Module
//     ======================================
//     1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
//     ======================================
//                16x2 LCD Module
//
//
//     AHT10 Module  Blue Pill  RESISTOR  POWER  I2C LCD Driver Module
//     ============  =========  ========  =====  =====================
//                      GND -------------- GND
//                       5V --------------  5V
//
//         |  VIN ------------------------ 3.3V
//    AHT10|  GND ------------------------ GND
//         |  SCL ----- B11 ---- [10K] --- 3.3V  
//         |  SDA ----- B10 ---- [10K] --- 3.3V
//
//
//                  |   B6 --------------------------- SCL
//                  |   B7 --------------------------- SDA
//                  |                      5V -------- VCC
//    I2C LCD Module|                     GND -------- GND
//                  |
//                  |                                  LED Jumper -- [1K] --\
//                  |                                                       |
//                  |                                  LED Jumper ----------/
//
// ============================================================================ 


#include <string.h>
#include "stm32f103x8.h"                  // Primary CMSIS header file
#include "STM32F103-pause-lib.c"          // pause and delay_us library

I2C_TypeDef *LCD_I2C;                     // Needed for below I2C LCD driver library
I2C_TypeDef *AHT10_I2C;                   // Needed for below AHT10 driver library
#include "STM32F103-CMSIS-I2C-LCD-lib.c"  // I2C LCD driver library

#define AHT10_ADD       0x38
#define AHT10_INIT      0b11100001
#define AHT10_TRIG_MEAS 0b10101100
#define AHT10_SOFT_RES  0b10111010
#define AHT10_NOP       0b00000000
#define AHT10_NORM_CMD  0b10101000
#define AHT10_DATA_MEAS 0b00110011
#define AHT10_STAT_REG  0b01110001
#define AHT10_STAT_REG_BSY (1<<7)
#define AHT10_STAT_REG_NOR (0b00<<5)
#define AHT10_STAT_REG_CYC (0b01<<5)
#define AHT10_STAT_REG_CMD (0b1<<6)
#define AHT10_STAT_REG_CAL (1<<3)



uint8_t
AHT10_getStatusReg( void )
{
  uint8_t gotData;                      /// This might not be right. May have to send command to request status reg!!!

  delay_us( 10e3 );

  I2C_start(   I2C2 );
  I2C_address( I2C2, AHT10_ADD, 1 );          // Command to read value
  
  while( !( I2C2->SR1 & (I2C_SR1_RXNE) )) ;   // Wait for receive data
  
  I2C_stop( I2C2 );
  gotData = I2C2->DR;
  I2C2->CR1 &= ~(I2C_CR1_ACK);                    // Send NAK
  
  return gotData;
}


void
AHT10_softReset( void )
{
  I2C_start(   I2C2 );                        // Start
  I2C_address( I2C2, AHT10_ADD, 0 );          // Command to get stat reg
  I2C_write(   I2C2, AHT10_SOFT_RES );
  I2C_stop(    I2C2 );
  delay_us( 20E3 );
}


void
AHT10_setStatusReg( uint8_t setV )
{
  delay_us( 10E3 );  
  I2C_start(   I2C2 );                // I2C start
  I2C_address( I2C2, AHT10_ADD, 0 );  // Send address as a "write" command
  I2C_write(   I2C2, AHT10_INIT );    // Send Init command
  I2C_write(   I2C2, setV );          // Send desired status byte
  I2C_write(   I2C2, AHT10_NOP );     // Finish with NOP command
  I2C_stop(    I2C2 );                // I2C stop
}


//  void
//  AHT10_init( I2C_TypeDef *this I2C )
//  Initialize the specified I2C interface and associate that interface with the subsequent
//  AHT10_ routines. Then initialize the AHT10 unit.
void
AHT10_init( I2C_TypeDef *thisI2C )
{
  AHT10_I2C = thisI2C;                    // Associate this I2C interface with AHT10 routines

  I2C_init( AHT10_I2C );                  // Initialize this I2C2 interface
  I2C_start( AHT10_I2C );                 // I2C start
  I2C_address( AHT10_I2C, AHT10_ADD, 0 ); // Send address as a "write" command
  I2C_write( AHT10_I2C, AHT10_INIT );     // 0xE1: Init command
  I2C_write( AHT10_I2C, 0x08 );           // As part of init, send command to set CAL bit
  I2C_write( AHT10_I2C, AHT10_NOP );      // Finish command with NOP command
  I2C_stop( AHT10_I2C );                  // I2C stop
  delay_us( 100 );                        // Allow 200 us for init to finish.
}


//  void
//  AHT10_readSensorData( uint8_t *data )
//  Called with a pointer to an array of at least 6 uint8-t ints.
//  Sends command to trigger a measurement. Then reads in the measured data after 75 ms.
//  The status register is contained in the first byte in the array. The subsequent 5 bytes
//  contain the raw humidity and temperature values.
//  The status register should have a value of 0x19 if normal. A status value of 0x99 indicates
//  that there was not sufficient time to complete the measurement.
//
//  Tempurature = raw-value
void
AHT10_readSensorData( uint8_t *data )
{
  I2C_start( AHT10_I2C );                   // Send command to trigger measurement
  I2C_address( AHT10_I2C, AHT10_ADD, 0 );
  I2C_write( AHT10_I2C, AHT10_TRIG_MEAS );  // 0xAC
  I2C_write( AHT10_I2C, AHT10_DATA_MEAS );  // 0x33
  I2C_write( AHT10_I2C, AHT10_NOP  );       // 0x00 
  I2C_stop( AHT10_I2C );
  
  delay_us( 75e3 );                         // Time for measurement to complete

  I2C_start( AHT10_I2C );                    // Send command to read data registers
  I2C_address( AHT10_I2C, AHT10_ADD, 1 );
  data[0] = I2C_read( AHT10_I2C, 1 );        // Status register
  data[1] = I2C_read( AHT10_I2C, 1 );        // Humidity [19:12]
  data[2] = I2C_read( AHT10_I2C, 1 );        // Humidity [11:4]
  data[3] = I2C_read( AHT10_I2C, 1 );        // Humidity [3:0] / Temperature [19:16]
  data[4] = I2C_read( AHT10_I2C, 1 );        // Temperature [15:8]
  data[5] = I2C_read( AHT10_I2C, 0 );        // Temperature [7:0]
  I2C_stop( AHT10_I2C );
}


//  void
//  i100toa( int16_t realV, int8_t *w, uint8_t *d, char *thisString )
//  i100toa takes a "times-100-integer and breaks itdown to an integer and one place after
//  the decimal, rounding as required. For example, if given 2445, which represents 24.45,
//  it would return 24 for the whole number part and 5 as the decimal (for 24.5).
void
i100toa( int16_t realV, int8_t *w, int8_t *d, char *thisString )
{
  char tmpString[2];            // Hold decimal digit and EOL character

  int16_t  x = abs( realV );    // realV = 2596 (26.0C) -> x = 2596
          *w = x/100;           // w = 25
  int8_t   f = x - (*w * 100);  // f = 2596 - 2500 = 96
          *d = f/10;            // d = 9
  int8_t   r = f - (*d * 10);   // r = 96 - 90 = 6

  if( r>=5 )                    // Round up if needed
  {
    *d = *d + 1;                // d = 10
    if( *d >= 10 )              
    {
      *d = 0;                   // d = 0
      *w = *w + 1;              // w = 26
    }
  }   
  if( realV < 0 )               // Restore negative value if original value was negative
    *w = 0 - *w;
  
  itoa( *w, thisString, 10 );   // Display whole part,
  strcat( thisString, "." );    // decimal point,
  itoa( *d, tmpString, 10 );    // single decimal place.
  strcat( thisString, tmpString );
}  
    

// ============================================================================
// main
// ============================================================================

#include <stdlib.h>


int
main()
{
  uint8_t gotData[8];
  uint8_t wp, dp;
  char myString[16];

  uint32_t tempV, humidV;
  int16_t realV;
  I2C_LCD_init( I2C1 );                     // Set the LCD interface to I2C1 and inialize it
  I2C_LCD_cmd( LCD_4B_58F_2L );
  I2C_LCD_cmd( LCD_ON_NO_CURSOR );
  I2C_LCD_cmd( LCD_CLEAR );
  I2C_LCD_cmd( LCD_HOME );
  I2C_LCD_puts( "AHT10  Status:" );

  AHT10_init( I2C2 );                       // Initialize AHT10 sensor

  while ( 1 )
  {
    I2C_LCD_cmd( LCD_1ST_LINE + 14 );       // Status byte display position

    AHT10_readSensorData( &gotData );       // Get data from sensor

    itoa( gotData[0], myString, 16 );       // Display Status Byte
    I2C_LCD_puts( myString );

                                            // Separate out humidity and temperature data
    humidV = (gotData[1]<<16            | gotData[2]<<8 | gotData[3] ) >> 4;
    tempV  = ( gotData[3] & 0x0F ) <<16 | gotData[4]<<8 | gotData[5] ;

    I2C_LCD_cmd( LCD_2ND_LINE );            // Position LCD to beginning of 2nd line

    realV = (tempV*625) / 32768 -5000;      // Calculate temperature x 100.
                                            // This is done to avoid the overhead of floating math routines.
                                            // tempC = (( tempV * 200 ) / 2^20 ) - 50
                                            // 100 x tempC = tempV  * 20000 ) / 2^20 ) - 5000
                                            // reduces to ( tempV * 625 ) / 32768 ) - 5000
    i100toa( realV, &wp, &dp, myString );   // Convert x100 value into whole (w) and decimal (d) parts
    I2C_LCD_puts( myString );               // Display value with suffix
    I2C_LCD_puts( "C  " );

    realV = humidV/10486;                   // Calculate and display humidity
    itoa( (uint32_t)realV, myString, 10 );  // humidity % = ( humidV / 2^20 ) * 100
    I2C_LCD_puts( myString );               // which is equiv. to ( humidV / 10486 ).
    I2C_LCD_puts( "%" );


    delay_us( 8e6 );                        // Pause 8 s between measurements. Excessive measurements may
                                            // lead to self-heating of the sensor.
//  I2C_LCD_cmd( 0x38 );
  
  }
  return 1;
}  
