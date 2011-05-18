#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <ctype.h>
#include <string.h>
#include <util/delay.h>

#include <math.h>

#include "main.h"
#include "ADXL345.h"
#include "TLC5947DAP.h"
#include "console.h"

void ioinit(void);
void serialinit(void);
void accelinit(void);

void read_mic(int16_t *buffer, uint8_t count);

uint8_t get_battery_percent();
void shutdown(bool forever);
void banner();
void lowbat();
void check_battery();

//FIXME: calibrate DELAY_MS and ITERS_PER_SECOND
#define DELAY_MS 1
#define ITERS_PER_SECOND 18
//check battery every 10 minutes
#define BATTERY_CHECK_ITERS (600 * ITERS_PER_SECOND)
//de-activate after a minute sitting idle
#define IDLE_ITERS (180 * ITERS_PER_SECOND)
#define IDLE_ITERS_ACTIVITY_THRESHOLD (60 * 1)
#define BRI_MAX 255
#define BRI_MIN 64
#define NUM_SAMPLES 128
#define HIST_SIZE (ITERS_PER_SECOND * 4)

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

int main(void)
{
  //LEDs
  uint16_t hue = 0, brightness = BRI_LIMIT - 1, lastBrightness = brightness;
  //motion
  int16_t oldaccel[3], accel[3] = {0};
  int32_t dotp[3];
  double flt_div, flt_acos;
  int8_t int_acos;
  //microphone
  int16_t capture[NUM_SAMPLES];
  uint8_t i;
  uint32_t histPower[HIST_SIZE] = {0}, histPowerIndex = 0;
  uint32_t currentPower, avgPower = 0;
  double scale;
  //power management
  uint16_t idleIters = 0, batteryCheckIters = 0, idleItersActivity = 0;

  // Initialize io ports
  ioinit();

  // Initialize serial
  serialinit();
  stdout = &mystdout;

  // Initialize accelerometer
  accelinit();

  // print banner/battery check
  banner();

  while(1)
  {
    //Read accelerometer, determine hue
    oldaccel[0] = accel[0]; oldaccel[1] = accel[1]; oldaccel[2] = accel[2];
    adxl345_getxyz(&accel[0], &accel[1], &accel[2]);
    //theta = acos(a.b / |a||b|)
    dotp[0] = accel[0] * oldaccel[0] + accel[1] * oldaccel[1] + accel[2] * oldaccel[2];
    dotp[1] = accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2];
    dotp[2] = oldaccel[0] * oldaccel[0] + oldaccel[1] * oldaccel[1] + oldaccel[2] * oldaccel[2];
    if(dotp[1] == 0) dotp[1] = 1;
    if(dotp[2] == 0) dotp[2] = 1;
    flt_div = dotp[0] / (sqrt(dotp[1]) * sqrt(dotp[2]));
    flt_acos = acos(flt_div);
    int_acos = (int8_t)(flt_acos * 180 / M_PI);

    //the amount of motion varies pretty widely; it seems like the best
    //aesthetic results are from motion-triggering
    if(int_acos > 15)
    {
      hue += 15;
      ++idleItersActivity;
    }
    if(hue >= HUE_LIMIT) hue -= HUE_LIMIT;

    //Read microphone FFT, determine intensity (sliding scale)
    read_mic(capture, NUM_SAMPLES);
    currentPower = 0;
    for(i = 0; i < NUM_SAMPLES; ++i)
      currentPower += (uint32_t)capture[i] * (uint32_t)capture[i];

    avgPower -= histPower[histPowerIndex];
    histPower[histPowerIndex] = currentPower;
    avgPower += histPower[histPowerIndex];
    histPowerIndex++;
    if(histPowerIndex >= HIST_SIZE) histPowerIndex = 0;

    //scale should vary between 0.5ish and 1.5ish... possibly throw a log in here?
    scale = (float)(currentPower * HIST_SIZE) / avgPower;
    scale -= 0.5;
    if(scale > 1.0) scale = 1.0;
    if(scale < 0.0) scale = 0.0;
    brightness = BRI_MIN + (BRI_MAX-BRI_MIN) * scale;
    if(brightness < lastBrightness)
      brightness = (brightness + lastBrightness) / 2;
    lastBrightness = brightness;

    //Update LED colors
    send_hsb(hue, SATURATION, brightness);

    //Handle console
    dispatch_console();

    //Power management
    if(batteryCheckIters >= BATTERY_CHECK_ITERS)
    {
      batteryCheckIters = 0;
      check_battery();
    }
    else
      ++batteryCheckIters;

    if(idleIters >= IDLE_ITERS)
    {
      if(idleItersActivity <= IDLE_ITERS_ACTIVITY_THRESHOLD)
        shutdown(false);
      else { idleIters = 0; idleItersActivity = 0; }
    }
    else
    {
      ++idleIters;
      if(adxl345_read(INT_SOURCE) & ACTIVITY)
        ++idleItersActivity;
    }

    //Loop delay
    _delay_ms(DELAY_MS);
  }
}

void ioinit(void)
{
  // Power latch
  PORTC |= _BV(PWR_EN_CPU); // PWR_EN_CPU should default to high
  DDRC |= (_BV(PWR_EN_CPU));

  // Accelerometer
  DDRC |= (_BV(ACCEL_SCK) | _BV(ACCEL_DI));
  PORTD |= _BV(ACCEL_CS); // ADXL_CS should default to high
  DDRD |= (_BV(ACCEL_CS));

  // LEDs
  DDRB |= (_BV(PWM_XL) | _BV(PWM_SI) | _BV(PWM_CL));
  PORTD |= _BV(PWM_BL); // LED_BLANK should default to high
  DDRD |= (_BV(PWM_BL));
}

void serialinit(void)
{
  // 115.2K, 8N1
  UBRR0 = 16;
  UCSR0A = (_BV(U2X0));
  UCSR0B = (_BV(RXEN0) | _BV(TXEN0));
  UCSR0C = (_BV(UCSZ01) | _BV(UCSZ00));
}

void accelinit(void)
{
  // Interrupts
  adxl345_write(THRESH_ACT, 10); // Threshold to detect activity
  adxl345_write(ACT_INACT_CTL, (_BV(4) | _BV(5) | _BV(6) | _BV(7))); // X, Y, Z participate in activity detection, AC coupled
  adxl345_write(DATA_FORMAT, INT_INVERT); // Needed for power control bootstrapping
  adxl345_write(INT_MAP, ~(ACTIVITY)); // Only activity interrupt on INT1
  adxl345_write(INT_ENABLE, ACTIVITY); // Enable the interrupts

  // Sampling
  adxl345_write(BW_RATE, RATE_25HZ); // Set sample rate to 25 Hz
  adxl345_write(POWER_CTL, MEASURE); // Start measuring
}

int uart_putchar(char c, FILE *stream)
{
  if(c == '\n') {
    uart_putchar('\r', stream);
  }
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
  return 0;
}

int uart_charwaiting(void)
{
  return !(UCSR0A & (1<<RXC0));
}

uint8_t uart_getchar(void)
{
  while( !(UCSR0A & (1<<RXC0)) );
  return UDR0;
}

//reads samples
void read_mic(int16_t *buffer, uint8_t count)
{
  ADMUX = MIC_ADC_MUX;
  do {
    ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADIF)|_BV(ADPS2)|_BV(ADPS1);
    while(bit_is_clear(ADCSRA, ADIF));
    *buffer++ = ADC - 512;
  } while(--count);
  ADCSRA = 0;
}

uint8_t get_battery_percent()
{
  ADMUX = BATT_ADC_MUX;

  //10x multisample
  uint16_t val = 0;
  for(uint8_t i = 0; i < 10; ++i) {
    ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADIF)|_BV(ADPS2)|_BV(ADPS1);
    while(bit_is_clear(ADCSRA, ADIF));
    val += ADC;
  };
  ADCSRA = 0;
  val /= 10;

  // Scale the voltage to percent
  // 500 => 1.8V
  // 800 => 3V
  if(val < 500) { val = 500; }
  if(val > 800) { val = 800; }
  val -= 500;
  val /= 3;

  return val;
}

//blink LEDs and turn off (in case of low battery)
void lowbat()
{
  send_rgb(100, 0, 0);
  _delay_ms(200);
  send_rgb(0, 0, 0);
  _delay_ms(200);
  send_rgb(100, 0, 0);
  _delay_ms(200);
  send_rgb(0, 0, 0);
  _delay_ms(200);
  send_rgb(100, 0, 0);
  _delay_ms(200);
  send_rgb(0, 0, 0);
  shutdown(true);
}

void banner()
{
  //console banner
  printf("Syzygryd Memorial Cube Firmware.  Hello!\n");

  //give the user a visual indication of battery health...
  uint8_t bat = get_battery_percent();
  if(bat == 0) {
    lowbat();
  } else {
    // Show the battery level
    send_rgb(100 - bat, bat, 0);
    _delay_ms(1000);
    send_rgb(0, 0, 0);
  }
}

void check_battery()
{
  //if the battery is too low, shut down
  uint8_t bat = get_battery_percent();
  if(bat == 0) {
    lowbat();
  }
}

void shutdown(bool forever)
{
  // Turn off the LEDs
  send_rgb(0,0,0);

  // Reset the ADXL threshold level
  adxl345_write(THRESH_ACT, 50); // Threshold to detect activity

  // If we don't want to turn back on turn off the accelerometer
  if(forever)
    adxl345_write(POWER_CTL, 0);

  // Loop until we die
  while(1)
  {
    // Read the ADXL interrupts to clear out PWR_EN_ADXL
    adxl345_read(INT_SOURCE);

    // Lower the power latch (PWR_EN_CPU)
    PORTC &= ~(_BV(PORTC4));

    _delay_ms(5000);
  }
}
