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

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <stdlib.h>         // C library. Needed for conversion function
#include "uart.h"           // Peter Fleury's UART library
#include "twi.h"            // TWI library for AVR-GCC
#include "lcd.h"

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
    //lcd_init(LCD_DISP_ON);
    //lcd_gotoxy(8,1);
    //lcd_puts("test");
    
    // Initialize I2C (TWI)
    twi_init();

    // Initialize UART to asynchronous, 8N1, 9600
    uart_init(UART_BAUD_SELECT(9600, F_CPU));

    // Configure 16-bit Timer/Counter1 to update FSM
    // Set prescaler to 33 ms and enable interrupt
    TIM1_overflow_33ms();
    TIM1_overflow_interrupt_enable();

    // Enables interrupts by setting the global interrupt mask
    sei();

    // Put strings to ringbuffer for transmitting via UART
    uart_puts("\r\nScan I2C-bus for devices:\r\n");

    // Infinite loop
    while (1)
    {
        /* Empty loop. All subsequent operations are performed exclusively 
         * inside interrupt service routines ISRs */
    }

    // Will never reach this
    return 0;
}

/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Update Finite State Machine and get humidity, temperature,
 *           and checksum from DHT12 sensor.
 **********************************************************************/
ISR(TIMER1_OVF_vect)
{
    static state_t state = STATE_IDLE;  // Current state of the FSM
    static uint8_t addr = 0x5c;  // I2C slave address of DHT12
    //uint8_t value;               // Data obtained from the I2C bus
    char uart_string[] = "000";  // String for converting numbers by itoa()
    uint8_t result = 1;

    // FSM
    switch (state)
    {
    // Do nothing
    case STATE_IDLE:
        lcd_gotoxy(1,1);
        //lcd_puts("      ");
        uart_puts("IDLE: \n");
        state = STATE_HUMID;
        break;
    
    // Get humidity
    case STATE_HUMID:
        lcd_gotoxy(1,1);
        //lcd_puts("      ");
        uart_puts("HUMID: \n");
        state = STATE_TEMP;
        break;

    // Get temperature
    case STATE_TEMP:
        // WRITE YOUR CODE HERE
        //lcd_gotoxy(1,1);
        //lcd_puts("      ");
        uart_puts("TEMP: \n");
        
        twi_start((addr<<1) + TWI_WRITE);
        twi_write(0x02);
        twi_stop();
        
        twi_start((addr<<1) + TWI_READ);
        result = twi_read_ack();
        
        itoa(result, uart_string, 10);
        
        //lcd_gotoxy(1, 2);
        //lcd_puts("      ");
        uart_puts(uart_string);
        // Move to the next state
        state = STATE_CHECK;
        break;

    // Get checksum
    case STATE_CHECK:
        //lcd_gotoxy(1,1);
        //lcd_puts("      ");
        uart_puts("CHECKSUM: \n");
        // Move to the next state
        state = STATE_IDLE;
        break;

    default:
        state = STATE_IDLE;
        break;
    }
}
