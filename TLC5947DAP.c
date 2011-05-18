#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <ctype.h>
#include <string.h>

#include "main.h"

void spi_write(uint16_t bit)
{
  if(bit) sbi(PWM_SI_PORT, PWM_SI);
  else    cbi(PWM_SI_PORT, PWM_SI);

  //clock the data
  sbi(PWM_CL_PORT, PWM_CL);
  cbi(PWM_CL_PORT, PWM_CL);
}

void send_rgb(uint16_t red, uint16_t green, uint16_t blue)
{
  int i, j;

  red <<= 4;
  green <<= 4;
  blue <<= 4;

  //send 8 LEDs x 3 colors x 12 bits of data (blue, green, red)
  for(i = 0; i < 8; ++i)
  {
    for(j = 0; j < 12; ++j)
      spi_write(blue & _BV(11 - j));
    for(j = 0; j < 12; ++j)
      spi_write(green & _BV(11 - j));
    for(j = 0; j < 12; ++j)
      spi_write(red & _BV(11 - j));
  }

  //latch the data
  sbi(PWM_XL_PORT, PWM_XL);
  cbi(PWM_XL_PORT, PWM_XL);

  //clear the blank, if necessary
  cbi(PWM_BL_PORT, PWM_BL);
}

void send_hsb(uint16_t hue, uint16_t sat, uint16_t bri)
{
  /*-------8-Bit-PWM-|-Light-Emission-normalized------
  hue: 0 to 764
  sat: 0 to 128 (0 to 127 with small inaccuracy)
  bri: 0 to 255
  http://mbed.org/forum/mbed/topic/1251/?page=1#comment-6216
  */
  uint16_t red_val, green_val, blue_val;

  while (hue > 764) hue -= 765;

  if (hue < 255) {
    red_val = (10880 - sat * (hue - 170)) >> 7;
    green_val = (10880 - sat * (85 - hue)) >> 7;
    blue_val = (10880 - sat * 85) >> 7;
  }
  else if (hue < 510) {
    red_val = (10880 - sat * 85) >> 7;
    green_val = (10880 - sat * (hue - 425)) >> 7;
    blue_val = (10880 - sat * (340 - hue)) >> 7;
  }
  else {
    red_val = (10880 - sat * (595 - hue)) >> 7;
    green_val = (10880 - sat * 85) >> 7;
    blue_val = (10880 - sat * (hue - 680)) >> 7;
  }

  red_val = (uint16_t)((bri + 1) * red_val) >> 8;
  green_val = (uint16_t)((bri + 1) * green_val) >> 8;
  blue_val = (uint16_t)((bri + 1) * blue_val) >> 8;

  send_rgb(red_val, green_val, blue_val);
}
