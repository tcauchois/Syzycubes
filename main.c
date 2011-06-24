#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <ctype.h>
#include <string.h>
#include <util/delay.h>
#include <util/atomic.h>

#include <math.h>

#include "main.h"
#include "ADXL345.h"
#include "TLC5947DAP.h"
#include "console.h"

void ioinit(void);
void timerinit(void);
void audioinit(void);
void serialinit(void);
void accelinit(void);

uint8_t get_battery_percent();
void shutdown(int forever);
void banner();
void lowbat();
uint16_t get_ticks();

//FIXME: calibrate DELAY_MS and ITERS_PER_SECOND
#define DELAY_MS 1
#define ITERS_PER_SECOND 18
#define ITERS_PER_MINUTE (ITERS_PER_SECOND * 60)

//check battery every 10 minutes
#define BATTERY_CHECK_ITERS (600 * ITERS_PER_SECOND)

// Idle config
#define IDLE_MINUTES_UNTIL_SHUTDOWN 10
#define IDLE_ACTIVITY_THRESHOLD 4
#define MINUTES_UNTIL_SHUTDOWN 15

#define BRI_MAX 255
#define BRI_MIN 32
#define NUM_SAMPLES 128
#define HIST_SIZE (ITERS_PER_SECOND * 4)

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
static volatile uint16_t ticks; // updated async
static volatile uint32_t curAudioPower; // updated async
static volatile uint8_t curAudioPowerReady; // updated async

int main(void)
{
  //LEDs
  uint16_t hue = 0, brightness = BRI_LIMIT - 1, lastBrightness = brightness;
  //motion 
  int16_t oldaccel[3], rawoldaccel[3], accel[3] = {0}, rawaccel[3] = {0};
  int32_t dotp[3];
  double flt_div, flt_acos;
  int8_t int_acos;
  //microphone
  uint32_t histPower[HIST_SIZE] = {0}, histPowerIndex = 0;
  uint32_t currentPower, avgPower = 0;
  double scale;
  //power management
  uint16_t batteryCheckIters = 0;
  //idle
  uint16_t idleIters = 0, idleActivity = 0;
  uint8_t idleMinutes = 0;

  // Initialize io ports
  ioinit();

  // Initialize serial
  serialinit();
  stdout = &mystdout;

  // print banner/battery check
  banner();

  // Initialize the timer
  timerinit();

  // Initialize audio capture
  audioinit();

  // Initialize accelerometer
  accelinit();

  // Enable interrupts
  sei();

  while(1)
  {
    // Wait until audio sample is ready
    while(!curAudioPowerReady) {}
    curAudioPowerReady = 0;

    // Grab the current audio power
    currentPower = curAudioPower;

    //Read accelerometer, determine hue
    oldaccel[0] = accel[0]; oldaccel[1] = accel[1]; oldaccel[2] = accel[2];
    rawoldaccel[0] = rawaccel[0]; rawoldaccel[1] = rawaccel[1]; rawoldaccel[2] = rawaccel[2];
    adxl345_getxyz(&rawaccel[0], &rawaccel[1], &rawaccel[2]);
    //(lowpass filter the accelerometer data)
    accel[0] = (rawoldaccel[0] + rawaccel[0]) / 2;
    accel[1] = (rawoldaccel[1] + rawaccel[1]) / 2;
    accel[2] = (rawoldaccel[2] + rawaccel[2]) / 2;
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
      idleActivity++;
    }
    if(hue >= HUE_LIMIT) hue -= HUE_LIMIT;

    //determine intensity (sliding scale)
    avgPower -= histPower[histPowerIndex];
    histPower[histPowerIndex] = currentPower;
    avgPower += histPower[histPowerIndex];
    histPowerIndex++;
    if(histPowerIndex >= HIST_SIZE) histPowerIndex = 0;

    //scale should vary between 0.5ish and 1.5ish... clamp to [0,1]
    scale = (float)(currentPower * HIST_SIZE) / avgPower;
    scale -= 0.5;
    if(scale > 1.0) scale = 1.0;
    if(scale < 0.0) scale = 0.0;
    //perception mapping: y=x^4 to give it a concave shape
    scale = scale*scale*scale*scale;
    brightness = BRI_MIN + (BRI_MAX-BRI_MIN) * scale;
    if(brightness < lastBrightness)
      brightness = (brightness + lastBrightness) / 2;
    lastBrightness = brightness;

    //Update LED colors
    send_hsb(hue, SATURATION, brightness);

    //Handle console
    dispatch_console();

    // Idle check
    idleIters++;
    if(idleIters >= ITERS_PER_MINUTE) {
      if(idleActivity >= IDLE_ACTIVITY_THRESHOLD) {
        // Activity!!!  Reset idle minutes
        idleMinutes = 0;
      } else {
        // Not enough activity, mark minute as idle
        idleMinutes++;
      }

      // Shutdown if idle for too long
      if(idleMinutes >= IDLE_MINUTES_UNTIL_SHUTDOWN) {
        shutdown(0);
      }

      // Reset iteration counts
      idleIters = 0;
      idleActivity = 0;
    }

    // Idle backstop check
    if(get_ticks() >= MINUTES_UNTIL_SHUTDOWN * 60) {
      shutdown(0);
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

void timerinit(void)
{
  // Set TIMER1_CAPT interrupt for 1s interval
  ICR1 = 15625;
  TIMSK1 |= (_BV(ICIE1));
  TCCR1B |= (_BV(WGM13) | _BV(WGM12) | _BV(CS12) | _BV(CS10)); // CTC, ICR1, CLK/1024
}

void audioinit(void)
{
  // Free running at 19230 samples/s
  ADMUX = (_BV(ADLAR) | MIC_ADC_MUX);
  ADCSRA = (_BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1));
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
  shutdown(1);
}

void banner()
{
  //console banner
  printf_P(PSTR("Syzygryd Memorial Cube Firmware.  Hello!\n"));

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

void shutdown(int forever)
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

uint16_t get_ticks()
{
  uint16_t result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    result = ticks;
  }
  return ticks;
}

ISR(TIMER1_CAPT_vect, ISR_BLOCK)
{
  ticks++;
}

ISR(ADC_vect, ISR_BLOCK)
{
  static uint16_t count;
  static uint32_t power;

  count++;
  if(count == 1000) {
    curAudioPower = power;
    curAudioPowerReady = 1;
    power = 0;
    count = 0;
  }

  int8_t sample = ADCH - 128;
  power += (sample * sample);
}
