/*
   ADXL345 Library

   This libary contains functions to interact with the ADXL345 Triple Axis Digital Accelerometer from Analog Devices written for the ATmega328p
   In order to use this libary, define the appropriate pins in the ADXL345.h file

   created 20 Aug 2009
   by Ryan Owens
   http://www.sparkfun.com

 */
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "main.h"
#include "ADXL345.h"

void adxl345_getxyz(int16_t *x, int16_t *y, int16_t *z)
{
  uint16_t high, low;

  //wait for a sample to be available
  while(!(adxl345_read(INT_SOURCE) & DATA_READY));

  high = adxl345_read(DATAX1);
  low = adxl345_read(DATAX0);
  *x = (high << 8) | low;

  high = adxl345_read(DATAY1);
  low = adxl345_read(DATAY0);
  *y = (high << 8) | low;

  high = adxl345_read(DATAZ1);
  low = adxl345_read(DATAZ0);
  *z = (high << 8) | low;
}

char adxl345_read(char register_address)
{
  char read_address=0x80 | register_address;
  char register_value=0;

  //clock high, chip select low, to start
  sbi(ACCEL_SCK_PORT, ACCEL_SCK);
  cbi(ACCEL_CS_PORT, ACCEL_CS);

  //write the register address
  for(int bit=7; bit>=0; bit--)
  {
    cbi(ACCEL_SCK_PORT, ACCEL_SCK);
    if(read_address & _BV(bit)) sbi(ACCEL_DI_PORT, ACCEL_DI);
    else                        cbi(ACCEL_DI_PORT, ACCEL_DI);
    _delay_us(1);
    sbi(ACCEL_SCK_PORT, ACCEL_SCK);
    _delay_us(1);
  }

  //read the register value
  for(int bit=7; bit>=0; bit--)
  {
    cbi(ACCEL_SCK_PORT, ACCEL_SCK);
    _delay_us(1);
    sbi(ACCEL_SCK_PORT, ACCEL_SCK);
    _delay_us(1);
    if(ACCEL_DO_PIN & _BV(ACCEL_DO))  sbi(register_value, bit);
    else                           cbi(register_value, bit);
  }

  //chip select high
  sbi(ACCEL_CS_PORT, ACCEL_CS);

  return register_value;
}

void adxl345_write(char register_address, char register_value)
{
  //clock high, chip select low, to start
  sbi(ACCEL_SCK_PORT, ACCEL_SCK);
  cbi(ACCEL_CS_PORT, ACCEL_CS);

  //write the register address
  for(int bit=7; bit>=0; bit--)
  {
    cbi(ACCEL_SCK_PORT, ACCEL_SCK);
    if(register_address & _BV(bit)) sbi(ACCEL_DI_PORT, ACCEL_DI);
    else                            cbi(ACCEL_DI_PORT, ACCEL_DI);
    _delay_us(1);
    sbi(ACCEL_SCK_PORT, ACCEL_SCK);
    _delay_us(1);
  }

  //write the register value
  for(int bit=7; bit>=0; bit--)
  {
    cbi(ACCEL_SCK_PORT, ACCEL_SCK);
    if(register_value & _BV(bit)) sbi(ACCEL_DI_PORT, ACCEL_DI);
    else                          cbi(ACCEL_DI_PORT, ACCEL_DI);
    _delay_us(1);
    sbi(ACCEL_SCK_PORT, ACCEL_SCK);
    _delay_us(1);
  }

  //chip select high
  sbi(ACCEL_CS_PORT, ACCEL_CS);
}
