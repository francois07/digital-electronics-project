/* Defines -----------------------------------------------------------*/
#ifndef F_CPU
# define F_CPU 16000000  // CPU frequency in Hz required for UART_BAUD_SELECT
#endif
#define MOTOR1 PD0
#define __DELAY_BACKWARD_COMPATIBLE__ // Make it possible to use variables in delay functions


/* Includes ----------------------------------------------------------*/
#include <util/delay.h>     // Functions for busy-wait delay loops
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <stdlib.h>         // C library. Needed for conversion function
#include "uart.h"           // Peter Fleury's UART library
#include "twi.h"            // TWI library for AVR-GCC
#include "lcd.h"
#include "gpio.h"           // GPIO library for AVR-GCC

/* Variables ---------------------------------------------------------*/
typedef enum {              // FSM declaration
    STATE_IDLE = 1,
    STATE_HUMID,
    STATE_TEMP,
    STATE_CHECK
} state_t;

uint8_t customChar[24] = {
  0b00000,
  0b00000,
  0b10101,
  0b01110,
  0b11111,
  0b01110,
  0b10101,
  0b00000,

  0b00000,
  0b01110,
  0b11111,
  0b11111,
  0b10101,
  0b00100,
  0b10100,
  0b01100,
  
  0b00000,
  0b00111,
  0b00101,
  0b00111,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

/* Function definitions ----------------------------------------------*/

/**********************************************************************
 * Function:    Rotate Motor Function
 * Purpose:     Rotate motor according to its datasheet description
 * Input:       reg_name		Name of the motor's PWM port
                pin_num			Name of the motor's PWM pin
                period			How many times to cycle
				pulse			Pulse width in us
 * Returns:     none
 **********************************************************************/
void rotateMotor(volatile uint8_t *reg_name, uint8_t pin_num, uint8_t period, uint64_t pulse)
{
    GPIO_write_low(reg_name, pin_num);
	for(uint8_t i=0; i < period; i++){
		_delay_ms(20);
		GPIO_toggle(reg_name, pin_num);
		_delay_us(pulse);
		GPIO_toggle(reg_name, pin_num);
	}
}

/**********************************************************************
 * Function:    Display Sensor Function
 * Purpose:     Use LCD and sensor to display informations such as current 
 *              temperature, humidity, ...
 * Input:       title           Title to be display along the data
                slave_adress    Sensor slave adress
                reg_adress      Data register address on the sensor
 * Returns:     The displayed information
 **********************************************************************/
uint8_t displaySensor(char title[], uint8_t slave_adress, uint8_t reg_adress)
{
    uint8_t result = 1;
    char res_string[] = "000";
    
    lcd_gotoxy(1, 0);
    lcd_puts(title);
        
    twi_start((slave_adress<<1) + TWI_WRITE);
    twi_write(reg_adress);
    twi_stop();
        
    twi_start((slave_adress<<1) + TWI_READ);
    result = twi_read_ack();
        
    itoa(result, res_string, 10);
        
    lcd_gotoxy(0, 1);
    lcd_puts(res_string);
    
    return result;
}

/**********************************************************************
 * Function:    Main function where the program execution begins
 * Purpose:     Use Timer/Counter1 and send I2C (TWI) address every 33 ms.
 *              Send information about scanning process to UART.
 * Returns:     none
 **********************************************************************/
int main(void)
{
    // Initialize LCD Display
    lcd_init(LCD_DISP_ON);
    
	lcd_command(1 << LCD_CGRAM);
	for (uint8_t i = 0; i < 24; i++)
	{
    	// Store all new chars to memory line by line
    	lcd_data(customChar[i]);
	}
	// Set DDRAM address
	lcd_command(1 << LCD_DDRAM);

	// Welcome screen
    lcd_gotoxy(0, 0);
    lcd_puts("De2");
    lcd_gotoxy(0, 1);
    lcd_puts("Meteo Station");
    
    // Initialize I2C (TWI)
    twi_init();

    // Configure 16-bit Timer/Counter1 to update FSM
    // Set prescaler to 33 ms and enable interrupt
    TIM1_overflow_4s();
    TIM1_overflow_interrupt_enable();

    // Enables interrupts by setting the global interrupt mask
    sei();
    
    // Configure the motor at port D
    GPIO_config_output(&DDRD, MOTOR1);

    // Infinite loop
    while (1)
    {
		rotateMotor(&PORTD, MOTOR1, 20, 1500);
		rotateMotor(&PORTD, MOTOR1, 20, 2000);
		rotateMotor(&PORTD, MOTOR1, 20, 1000);
    }

    // Will never reach this
    return 0;
}

/* Interrupt service routines ----------------------------------------*/

/**********************************************************************
 * Function:	Timer/Counter1 overflow interrupt
 * Purpose:		Update Finite State Machine and get humidity, temperature,
 *				and checksum from DHT12 sensor.
 **********************************************************************/
ISR(TIMER1_OVF_vect)
{
    static state_t state = STATE_TEMP;  // Current state of the FSM
    static uint8_t addr = 0x5c;  // I2C slave address of DHT12
    static uint8_t res;
	
    lcd_clrscr();
	
    // FSM
    switch (state)
    {
    // Get humidity
    case STATE_HUMID:
        res = displaySensor("HUMIDITY", addr, 0x00);
        lcd_puts("% ");
        
        if (res >= 16)
        {
            lcd_putc(1);
        }

        state = STATE_TEMP;
        break;

    // Get temperature
    case STATE_TEMP:
        res = displaySensor("TEMPERATURE", addr, 0x02);
        lcd_putc(2);
        lcd_puts("C ");
        
        if (res >= 16)
        {
            lcd_putc(0);
        }

        state = STATE_HUMID;
        break;

    default:
        state = STATE_TEMP;
        break;
    }
}
