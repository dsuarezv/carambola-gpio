/*
 * Copyright (C) 2013 David Suarez
 * See LICENSE for more information.
 */
#ifndef __LIBGPIO_H__
#define __LIBGPIO_H__

#ifdef __cplusplus
extern "C" {
#endif


#include <sys/types.h>
#include <stdint.h>


// __ Constants _______________________________________________________________


#define SPI_MODE_GPIO  0x02
#define SPI_MODE_SPI   0x00

#define I2C_MODE_GPIO  0x01
#define I2C_MODE_I2C   0x00


/*
 *  GPIO bitmasks
 */
#define GPIO1  0x0002
#define GPIO2  0x0004
#define GPIO3  0x0008
#define GPIO4  0x0010
#define GPIO5  0x0020
#define GPIO6  0x0040
#define GPIO7  0x0080
#define GPIO8  0x0100
#define GPIO9  0x0200
#define GPIO10 0x0400
#define GPIO11 0x0800
#define GPIO12 0x1000
#define GPIO13 0x2000
#define GPIO14 0x4000
//#define ALL_GPIOS 0x7FFE
#define ALL_GPIOS 0xFFFFFF


// __ Arduino types and defines _______________________________________________

#define __AVR__ 1


typedef uint8_t boolean;
typedef uint8_t byte;
typedef uint16_t word;


// __ Registers _______________________________________________________________


/*
 * Direct access to the GPIO data and dir registers
 */
extern uint32_t *PORT;
extern uint8_t  *PORTA;
extern uint8_t  *PORTB;
extern uint8_t  *PORTC;

extern uint32_t *DPP;
extern uint8_t  *DPPA;
extern uint8_t  *DPPB;
extern uint8_t  *DPPC;


// __ API _____________________________________________________________________


void		gpio_init();
void		gpio_dispose();

void		gpio_set_mode(unsigned int mode);

void		gpio_setdir(uint32_t value);
uint32_t	gpio_getdir();

void 		gpio_set(uint32_t value);
uint32_t 	gpio_get();


// __ Arduino API  ____________________________________________________________


#define HIGH 1
#define LOW  0
#define OUTPUT 1
#define INPUT 0


// pins_arduino.h
uint32_t *digitalPinToPort(uint8_t pin);
uint32_t digitalPinToBitMask(uint8_t pin);
uint8_t *portModeRegister(uint8_t port);
uint8_t *portInputRegister(uint8_t port);


// wiring_private.h
void cbi(uint32_t *addr, uint32_t bitmask);
void sbi(uint32_t *addr, uint32_t bitmask);


//
void cli();
void sei();


// wiring.h
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
uint8_t digitalRead(uint8_t pin);
void delayMicroseconds(unsigned int delayMicroseconds);
void delay(unsigned int ms);
unsigned long millis();
unsigned long micros();

#define interrupts() sei()
#define noInterrupts() cli()


//
#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);



#ifdef __cplusplus
}
# endif

#endif //__LIBGPIO_H__
