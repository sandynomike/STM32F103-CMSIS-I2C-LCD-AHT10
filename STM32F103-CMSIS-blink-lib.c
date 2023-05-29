//  STM32F103-CMSIS-blink-lib.c

//  Target Microcontroller: STM32F103 (Blue Pill)
//  Mike Shegedin, 05/2023
//
//  Code to impliment simple routines to blink the LED built into the Blue Pill.
//  Relies on the STM32F103-pause-lib.c routines.
//
//  Implements the following routines:
//  ----------------------------------
//  void
//  BL_init( void )
//    Initializes GPIO port C for and pin C13 (built-in LED) as output.
//  ----------------------------------
//  void
//  BL_nBlink( uint8_t blCnt )
//    Blink the built-in LED a set number of times
//  ----------------------------------
//  void
//  BL_pBlink( uint32_t blPer )
//    Blink the built-in LED continuously at the period of blPer us. Use with large values
//    to test/set timing of the BL_nBlink routine.
//  ----------------------------------

#ifndef __STM32F103_CMSIS_BLINK_LIB_C
#define __STM32F103_CMSIS_BLINK_LIB_C

#include "stm32f103x8.h"  // Primary CMSIS header file
#include "STM32F103-pause-lib.c"


// BL_init
// Initialize blinking LED function. For Blue Pill, this means setting up
// GPIO port C pin 13 as an output with push-pull.
void
BL_init( void )
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;             // Enable GPIO C clock
  GPIOC->CRH &= ~( 0b00 << GPIO_CRH_CNF13_Pos );  // Clear CNF13 bits
  GPIOC->CRH |=  ( 0b10 << GPIO_CRH_MODE13_Pos ); // Set MODE13 bits to 0b10
}


// BL_nBlink
// Blink LED a set number of times
void
BL_nBlink( uint8_t blCnt )
{
  while( blCnt-- > 0 )
  {
    GPIOC->ODR &= ~GPIO_ODR_ODR13;  // Turn ON for approx. 0.25 s
    delay_us( 250000 );             
    GPIOC->ODR |= GPIO_ODR_ODR13;   // Turn OFF for approx. 0.25 s
    delay_us( 250000 );
  }  
}


// BL_pBlink
// Blink LED continuously at a set period in us.
void
BL_pBlink( uint32_t blPer )
{
  uint16_t x;

  while( 1 )
  {
      GPIOC->ODR ^= GPIO_ODR_ODR13;
      delay_us( blPer );
  }
}

#endif /* __STM32F103_CMSIS_BLINK_LIB_C */
