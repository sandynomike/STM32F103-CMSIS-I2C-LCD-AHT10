// =====================================================================================
//  STM32F103-CMSIS-I2C-LCD-lib.c

//  Target Microcontroller: STM32F103 (Blue Pill)
//  Target I2C device: I2C 16x2 LCD Driver Module based on the PCF8574.
//
//  Mike Shegedin, 05/2023
//
//
//  HARDWARE SETUP
//  ==============
//  While a 5V 16x2 LCD module can be driven mostly with 3.3 logic levels,
//  and while the I2C display driver module can operate in a 3.3V system,
//  the combination probably requires that, if using a 3.3V microcontroller,
//  the I2C LCD driver module itself must be powered by 5 V in order to
//  properly drive a 5V LCD module.
//
//  The 16 pins of the I2C LCD driver module are connected to the corresponding
//  pins on the 16x2 LCD module. The I2C data lines (SCA and SCL) lines can be
//  directly connected to the I2C lines on the microcontroller. Pullup resistors
//  are built into the I2C LCD driver module, so additional pullup resistors on
//  the I2C lines are not needed.
//
//  If the display seems too bright with the backlight jumper installed, then the
//  jumper can be replaced with a resistor of hundreds of ohms or higher to dim
//  the display. Technically the backlight on the LCD could be driven via PWM, but
//  as it is being controlled via I2C on the I2C LCD driver module, this is not
//  practical to implement.
//  
//
//             I2C LCD Driver Module
//     ======================================
//     1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
//     ======================================
//                16x2 LCD Module
//
//
//     Blue Pill  I2C LCD Driver Module
//     =========  =====================
//        GND ----------- GND
//         5V ----------- VDD
//         B6 ----------- SCL
//         B7 ----------- SDA
//
//                        LED Jumper -- [1k ohm] --\
//                                                 |
//                        LED Jumper --------------/
//
//

#ifndef __STM32F103_CMSIS_I2C_LCD_LIB_C
#define __STM32F103_CMSIS_I2C_LCD_LIB_C


#include <stdlib.h>
#include "stm32f103x8.h"              // Primary CMSIS header file
#include "STM32F103-CMSIS-I2C-lib.c"


// Pin-Bit definitions between the LCD module pins and the
// I2C LCD driver module data-byte bits.

#define I2C_LCD_ADD 0x3F
#define I2C_LCD_RS  0b00000001
#define I2C_LCD_RW  0b00000010
#define I2C_LCD_EN  0b00000100
#define I2C_LCD_BL  0b00001000
#define I2C_LCD_D4  0b00010000
#define I2C_LCD_D5  0b00100000
#define I2C_LCD_D6  0b01000000
#define I2C_LCD_D7  0b10000000


//  LCD Command Codes
//  These command codes are used to put the LCD module into various modes.
#define LCD_CLEAR                0x01
#define LCD_HOME                 0x02
#define LCD_OFF                  0x08
#define LCD_ON_NO_CURSOR         0x0C
#define LCD_ON_BLINK_CURSOR      0x0D
#define LCD_ON_LINE_CURSOR       0x0E 
#define LCD_ON_LINE_BLINK_CURSOR 0x0F
#define LCD_4B_58F_2L            0x2B
#define LCD_1ST_LINE             0x80
#define LCD_2ND_LINE             0xC0

// Command used initially to get the LCD module into the 4-bit mode:
#define I2C_LCD_4B  0x02


// I2C_LCD_init
// Initialize the LCD display module. The first step is to initialize the
// associated I2C port. Then there is a 20 ms wait time to give the display
// module time to fully power up. Then the command is sent to set the LCD
// module into the 4-bit mode.
void
I2C_LCD_init( I2C_TypeDef *thisI2C )
{
  LCD_I2C = thisI2C;    // Set Global LCD_I2C interface to I2C1 or I2C2

  uint8_t LCD_data;


  I2C_init( LCD_I2C );
  delay_us( 20000 );

  // Send initial 4-bit command
  LCD_data = I2C_LCD_EN | I2C_LCD_BL | (I2C_LCD_4B << 4 );
  I2C_writeByte ( LCD_I2C, LCD_data, I2C_LCD_ADD );
  delay_us( 1000 );
  
  // Turn off EN after 1 ms
  LCD_data = I2C_LCD_BL;
  I2C_writeByte ( LCD_I2C, LCD_data, I2C_LCD_ADD );
  delay_us( 1000 );
}


// I2C_LCD_cmd
// Send command (not character) to LCD display. Commands are sent like data but
// with the RS pin set LOW
void
I2C_LCD_cmd( uint8_t data )
{
  uint8_t I2C_data;

  // Place upper nibble and EN and BL and WR
  I2C_data = (data & 0b11110000) | I2C_LCD_EN | I2C_LCD_BL;
  I2C_writeByte( LCD_I2C, I2C_data, I2C_LCD_ADD );
  
  // Clear EN bit
  I2C_data =   I2C_LCD_BL  ;
  I2C_writeByte( LCD_I2C, I2C_data, I2C_LCD_ADD );

  // Place lower nibble and EN and BL and WR
  I2C_data = (data << 4) | I2C_LCD_EN | I2C_LCD_BL;
  I2C_writeByte( LCD_I2C, I2C_data, I2C_LCD_ADD );
  
  // Clear EN bit
  I2C_data =  I2C_LCD_BL  ;
  I2C_writeByte( LCD_I2C, I2C_data, I2C_LCD_ADD );
}


// I2C_LCD_putc
// Writes a single character to the current position on the LCD display
void
I2C_LCD_putc( char data )
{
  char I2C_data;

  // Place upper nibble and set EN, RS, and BL bits 
  I2C_data = (data & 0b11110000) | I2C_LCD_EN | I2C_LCD_RS | I2C_LCD_BL;
  I2C_writeByte( LCD_I2C, I2C_data, I2C_LCD_ADD );
  
  // Clear EN bit
  I2C_data =  I2C_LCD_BL | I2C_LCD_RS ;
  I2C_writeByte( LCD_I2C, I2C_data, I2C_LCD_ADD );

  // Place lower nibble and set EN, RS, and BL bits
  I2C_data = (data << 4) | I2C_LCD_EN | I2C_LCD_RS | I2C_LCD_BL;
  I2C_writeByte( LCD_I2C, I2C_data, I2C_LCD_ADD );

  // Clear EN bit
  I2C_data =  I2C_LCD_BL | I2C_LCD_RS ;
  I2C_writeByte( LCD_I2C, I2C_data, I2C_LCD_ADD );
  delay_us( 2000 );
}


//  LCD_puts
//  Takes a pointer to a null-terminated string and displays that string
//  from the current LCD cursor position. Does not check for LCD line/string
//  overflow.
void
I2C_LCD_puts( char *data )
{
  uint8_t j=0;

    while( data[j] != 0 )
      {
        I2C_LCD_putc( data[j] );
        j++;
      }
}


#endif /* __STM32F103_CMSIS_I2C_LCD_LIB_C */
