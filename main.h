#ifndef MAIN_H
#define MAIN_H

//*******************************************************
//					GPIO Definitions
//*******************************************************

/* UART port definitions */
#define UART_RX_I			0		//Port D.0
#define UART_TX_O			1		//Port D.1

/* PWM LED controller definitions */
#define PWM_SI_PORT PORTB
#define PWM_SI        3   //Port B.3
#define PWM_CL_PORT PORTB
#define PWM_CL        5   //Port B.5
#define PWM_BL_PORT PORTD
#define PWM_BL        7   //Port D.7
#define PWM_XL_PORT PORTB
#define PWM_XL        0   //Port B.0

/* Accelerometer definitions */
#define ACCEL_CS_PORT PORTD
#define ACCEL_CS      4   //Port D.4
#define ACCEL_SCK_PORT PORTC
#define ACCEL_SCK     1   //Port C.1
#define ACCEL_DI_PORT PORTC
#define ACCEL_DI      2   //Port C.2
#define ACCEL_DO_PIN PINC
#define ACCEL_DO      3   //Port C.3
#define ACCEL_INT2_PIN PIND
#define ACCEL_INT2    3   //Port D.3

/* Microphone definitions */
#define MIC_ADC_MUX   0   //ADC0 (Port C.0)

/* Power latch */
#define PWR_EN_CPU    4   //Port C.4

/* Battery ADC */
#define BATT_ADC_MUX  5   //ADC5 (Port C.5)

//*******************************************************
//						Macros
//*******************************************************
#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

//*******************************************************
//					General Definitions
//*******************************************************
//#define MYUBRR 16	//Used to set the AVR Baud Rate TO 57600 (internal 8 MHz Oscillator)

//=======================================================
//					Function Definitions
//=======================================================
int uart_putchar(char c, FILE *stream);
uint8_t uart_getchar(void);
int uart_charwaiting(void);

#endif
