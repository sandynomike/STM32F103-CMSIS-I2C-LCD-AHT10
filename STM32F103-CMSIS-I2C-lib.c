// STM32F103-CMSIS-I2C-lib.c
//
// Target Microcontroller: STM32F103 (Blue Pill)
// Mike Shegedin, 05/2023
//
// Requires the STM32F103-pause-lib.c library.
//
// Code to implement the following routines.
// Note that the desired target I2C interface, I2C1 or I2C2, will be passed to *thisI2C.
// The interface name is passed as-is, like "I2C_init( I2C1 );".
// -----------------------------------------
// void
// I2C_init( I2C_TypeDef *thisI2C )
//    Initialize the specified I2C interface
// -----------------------------------------
// void
// I2C_start( I2C_TypeDef *thisI2C )
//    Set the start bit and wait for acknowledge that it was set.
// -----------------------------------------
// void
// I2C_address( I2C_TypeDef *thisI2C, uint8_t address, uint8_t readBit )
//    Send out the address of the desired I2C target and wait for the target
//    to acknowledge that the address was received. Note that the routine will
//    hang if no target device acknowledges the address.
// -----------------------------------------
// void
// I2C_write( I2C_TypeDef *thisI2C, uint8_t data )
//    Write a byte of data to the specified I2C interface after the DR register is
//    empty. Waits for acknowledgement that the byte was transferred before resuming.
//    Note that this routine does not set the start/stop bits nor poll the address.
//    Use the I2C_writeByte routine in the application to send a byte of data to a
//    specific I2C target device.
// -----------------------------------------
// void
// I2C_stop( I2C_TypeDef *thisI2C )
//    Send the I2C stop bit and wait 20 us to allow enough time for next I2C command
//    to occur properly.
// -----------------------------------------

// void
// I2C_writeByte( I2C_TypeDef *thisI2C, uint8_t data, uint8_t Address )
// -----------------------------------------

#ifndef __STM32F103_CMSIS_I2C_LIB_C
#define __STM32F103_CMSIS_I2C_LIB_C


#include "stm32f103x8.h"  // Primary CMSIS header file
#include "STM32F103-pause-lib.c"


// I2C_init
// Initialize GPIO port B pins 6 and 7 for I2C1 for pins 10 and 11 for I2C2, and set up required I2C clocks.
void
I2C_init( I2C_TypeDef *thisI2C )
{
  if( thisI2C == I2C1 )
  {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_AFIOEN;   // Enable GPIO PORT B and Alt. Func. Clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;   // Enable I2C1 Clock

    // Set CNF and MODE bits for B6/B7 to 0b11 for Alternate Function / High Speed
    GPIOB->CRL |= ( 0b11 << GPIO_CRL_CNF6_Pos  |
                    0b11 << GPIO_CRL_CNF7_Pos  |
                    0b11 << GPIO_CRL_MODE6_Pos |
                    0b11 << GPIO_CRL_MODE7_Pos );
  }
  else  // I2C2 Setup
  {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_AFIOEN;   // Enable GPIO PORT B and Alt. Func. Clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;   // Enable I2C2 Clock

    // Set CNF and MODE bits for B10/B11 to 0b11 for Alternate Function / High Speed
    GPIOB->CRH |= ( 0b11 << GPIO_CRH_CNF10_Pos  |
                    0b11 << GPIO_CRH_CNF11_Pos  |
                    0b11 << GPIO_CRH_MODE10_Pos |
                    0b11 << GPIO_CRH_MODE11_Pos );
  }

  thisI2C->TRISE |= 0x02;                // Set the TRISE time
  thisI2C->CR1   |=  I2C_CR1_SWRST;      // Set I2C reset bit
  thisI2C->CR1   &= ~I2C_CR1_SWRST;      // Clear I2C reset bit
  thisI2C->CR2   |= 0x08 << I2C_CR2_FREQ_Pos;  // Set to APB1 Peripheral Clock freq. in MHz (8 MHz = 8)
  thisI2C->TRISE  = 0x09;                // Set to APB1 Peripheral Clock Freq. in MHz + 1 (8+1 = 9)
  thisI2C->CCR    = 0x28;                // CCR = 5 us * APB1 Peripheral clock speed (8E6) = 40 (0x28)
                                                          // F/S and DUTY bits are 0 as not using "fast mode" i2c.
  thisI2C->OAR1   = 0x4000;              // Probably not needed when you are the master
  thisI2C->CR1   |= I2C_CR1_PE;          // Turn on I2C peripheral
  thisI2C->CR1   |= I2C_CR1_ACK;
}


// I2C_start
// Command for host to start I2C communication.
void
I2C_start( I2C_TypeDef *thisI2C )
{
  // Set START bit in CR1 which initializes I2C sequence and cause the I2C interface to enter master mode.
  thisI2C->CR1 |= I2C_CR1_START;
  
  // Wait for SB bit in SR1 to be set indicating start occurred [EV5:1]
  while( !( thisI2C->SR1 & I2C_SR1_SB )) ;
}


// I2C_address
// Command for host to send the I2C address of the desired target device. Waits for target to acknowledge.
// Note -- will hang if no target acknowledges the sent address.
void
I2C_address( I2C_TypeDef *thisI2C, uint8_t address, uint8_t readBit )
{
  // Load address into DR and send it out [EV5:2]
  // Note that the address is shifted 1 bit to the left, to make room for the read/write bit in the lsb position.
  thisI2C->DR = address<<1 | (0x1 & readBit );

  // Wait for ADDR bit in SR1 to be set indicating the address was transferred [EV5:3] [EV6:1].
  while( !( thisI2C->SR1 & I2C_SR1_ADDR )) ;

  // Read SR1 and SR2 to clear ADDR bit [EV6:2]
  uint8_t temp = thisI2C->SR1 | thisI2C->SR2;
}


// I2C_write
// Wait for TXE (transmit buffer empty) bit in SR1 to be set, send data to write, wait for BTF (byte-transfer-
// finished) flag to be set.
void
I2C_write( I2C_TypeDef *thisI2C, uint8_t data )
{ 
  // Wait for TXE bit in SR1 to be set indicating DR is empty [EV8_1:1]
  while( !( thisI2C->SR1 & I2C_SR1_TXE )) ;
  
  // Load data to write, and send it out [EV8_1:2]
  thisI2C->DR = data;                        
  
  // Wait for BTF bit in SR1 to be set indicating the byte was transferred [EV8_2:1]
  while( !( thisI2C->SR1 & I2C_SR1_BTF )) ;
}


// I2C_stop
// Command for host to STOP current I2C transfer.
void
I2C_stop( I2C_TypeDef *thisI2C )
{
  thisI2C->CR1 |= I2C_CR1_STOP;              // Set STOP bit in CR1 indicating end of I2C transmission and
                                          // return to slave mode [EV8_2:2]
  delay_us(20);                                          
}


/*
void
I2C_writeMulti( uint8_t *data, uint8_t size)
{
  while( !( I2C1->SR1 & I2C_SR1_TXE )) ;    // Wait for TXE bit to be set
  while( size )
  {
    while ( !(I2C1->SR1 & I2C_SR1_TXE )) ;  // Wait for TXE bit to be set
    I2C1->DR = (volatile uint32_t)*data++;  // Send data byte
    size--;
  }
  while( !( I2C1->SR1 & I2C_SR1_BTF )) ;
}
*/


// I2C_writeByte
// Command for host to send one byte of data to the target device at the specified address.
// Does a complete single byte send from starting I2C, sending address, sending the byte, then
// stopping the I2C transfer.
void
I2C_writeByte( I2C_TypeDef *thisI2C, uint8_t data, uint8_t Address )
{
    I2C_start(   thisI2C );
    I2C_address( thisI2C, Address, 0 );
    I2C_write(   thisI2C, data );
    I2C_stop(    thisI2C );
}


/*
// I2C_readByte
// Command for host to request one byte from the target device at the specified address.
void
I2C_readByte( uint8_t *data,  uint8_t address )
{
//  I2C_start();                            // Start I2C sequence
//  I2C_address( address, 1 );              // Poll address
//  I2C_writeByte( reg, address );          // Send byte command indicating what data to read
//  I2C_start();

  // I2C_read( Address, data );
  
  I2C1->CR1 |= I2C_CR1_START;             // Set the START bit
  while( !( I2C1->SR1 & I2C_SR1_SB )) ;   // Wait for SB to be set

  I2C1->DR = (address<<1) + 0x00;           // Send address and read-bit
  while( !( I2C1->SR1 & I2C_SR1_ADDR )) ; // Wait for ADDR bit to be set indicating the address was transferred
  I2C1->CR1 &= ~(I2C_CR1_ACK);              // Clear ACK bit
  uint8_t temp = I2C1->SR1 | I2C1->SR2;   // Read these regs to clear ADDR bit [EV6]
  I2C1->CR1 |= I2C_CR1_STOP;              // Stop I2C

  while( !( I2C1->SR1 & (I2C_SR1_RXNE) )) ; // Wait for data register to have data

  uint8_t keep = (I2C1->DR & 0xFF);              // Read received data
//  TIM1->CR1 = keep;
  I2C_stop( (uint32_t)I2C1 );
  *data = keep;
}
*/

#endif /* __STM32F103_CMSIS_I2C_LIB.C */
