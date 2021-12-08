/***********************************************************************
 * 
 * The I2C bus scanner detects the addresses of the modules that are 
 * connected to the SDA and SCL signals. A simple description of FSM is 
 * used.
 * ATmega328P (Arduino Uno), 16 MHz, AVR 8-bit Toolchain 3.6.2
 *
 * Copyright (c) 2017-Present Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/

/* Defines -----------------------------------------------------------*/
#ifndef F_CPU
# define F_CPU 16000000  // CPU frequency in Hz required for UART_BAUD_SELECT
#endif
#define MOTOR1 PD0

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

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Use Timer/Counter1 and send I2C (TWI) address every 33 ms.
 *           Send information about scanning process to UART.
 * Returns:  none
 **********************************************************************/
int main(void)
{
    // Initialize LCD Display
    lcd_init(LCD_DISP_ON);
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
    
    // Configure the first motor at port A
    GPIO_config_output(&DDRD, MOTOR1);
    GPIO_write_low(&PORTD, MOTOR1);


    // Infinite loop
    while (1)
    {
        /* Empty loop. All subsequent operations are performed exclusively 
         * inside interrupt service routines ISRs */
        //GPIO_toggle(&PORTD, MOTOR1);
        //_delay_us(2000);
        //GPIO_toggle(&PORTD, MOTOR1);
        //_delay_us(1000);
        //
        //GPIO_toggle(&PORTD, MOTOR1);
        //_delay_us(1000);
        //GPIO_toggle(&PORTD, MOTOR1);
        //_delay_us(1000);    
    }

    // Will never reach this
    return 0;
}

void displaySensor(char title[], uint8_t slave_adress, uint8_t reg_adress)
{
    uint8_t result = 1;
    char uart_string[] = "000";
    
    lcd_gotoxy(1, 0);
    lcd_puts(title);
        
    twi_start((slave_adress<<1) + TWI_WRITE);
    twi_write(reg_adress);
    twi_stop();
        
    twi_start((slave_adress<<1) + TWI_READ);
    result = twi_read_ack();
        
    itoa(result, uart_string, 10);
        
    lcd_gotoxy(0, 1);
    lcd_puts(uart_string);
}

/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Update Finite State Machine and get humidity, temperature,
 *           and checksum from DHT12 sensor.
 **********************************************************************/
ISR(TIMER1_OVF_vect)
{
    static state_t state = STATE_TEMP;  // Current state of the FSM
    static uint8_t addr = 0x5c;  // I2C slave address of DHT12

    // FSM
    switch (state)
    {
    // Get humidity
    case STATE_HUMID:
        lcd_clrscr();

        displaySensor("HUMIDITY", addr, 0x00);

        state = STATE_TEMP;
        break;

    // Get temperature
    case STATE_TEMP:
        // WRITE YOUR CODE HERE
        lcd_clrscr();

        displaySensor("TEMPERATURE", addr, 0x02);

        state = STATE_HUMID;
        break;

    default:
        state = STATE_TEMP;
        break;
    }
}
