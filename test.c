/*
 * Copyright (C) 2013 David Suarez
 * See LICENSE for more information.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "libgpio.h"


/*
 * To run the tests, set the analizer probes on GPIOs 4 and 10.
 * Use GPIO 4 as the trigger (raising edge).
 */

void test_gpio_set()
{
	// 10 pulses on all GPIO lines using gpio_set

	int i;

	for (i = 0; i < 10; ++i)
	{
		gpio_set(ALL_GPIOS);
		gpio_set(0);
	}
}


void test_cbi_sti()
{
	// 10 pulses on GPIO10 using cbi / sti

	int i;
	uint32_t bitmask = digitalPinToBitMask(10);

	for (i = 0; i < 10; ++i)
	{
		sbi(PORT, bitmask);
		cbi(PORT, bitmask);
	}
}


void test_digitalWrite()
{
	// 10 pulses on all GPIO lines using digitalWrite

	int i;

	for (i = 0; i < 10; ++i)
	{
		digitalWrite(10, 1);
		digitalWrite(10, 0);
	}
}


void test_digitalRead()
{
	// Triggers analyzer test sequence on GPIO10 by rising edge on GPIO4

	unsigned int testPin = 10;
	unsigned int triggerPin = 4;

	pinMode(testPin, INPUT);
	pinMode(triggerPin, OUTPUT);

	// Trigger on GPIO4
	digitalWrite(triggerPin, HIGH);
	digitalWrite(triggerPin, LOW);

	// And expect some data on GPIO10
	int numSamples = 2000000;
	uint8_t lastState = LOW;

	while (numSamples > 0)
	{
		uint8_t read = digitalRead(testPin);
		
		if (read != lastState)
			printf("read: %0X\n", read);

		lastState = read;

		--numSamples;
	}
}

void test_delay()
{
	unsigned int pin = 10;

	pinMode(pin, OUTPUT);

	digitalWrite(pin, HIGH);
	delay(100);
	digitalWrite(pin, LOW);

	delay(100);

	digitalWrite(pin, HIGH);
	delay(500);
	digitalWrite(pin, LOW);
}

void test_delayMicroseconds()
{
	unsigned int pin = 10;

	pinMode(pin, OUTPUT);

	digitalWrite(pin, HIGH);
	delayMicroseconds(100);
	digitalWrite(pin, LOW);

	delayMicroseconds(100);

	int i;

	// pulses with no delay
	for (i = 0; i < 10; ++i)
	{
		digitalWrite(pin, HIGH);
		digitalWrite(pin, LOW);
	}

	// pulses with 1us delay (expected ~3us in analyzer)
	for (i = 0; i < 10; ++i)
	{
		digitalWrite(pin, HIGH);
		delayMicroseconds(1);
		digitalWrite(pin, LOW);
		delayMicroseconds(1);
	}

	// pulses with varying delays: 10, 20, 30, ... , 100
	for (i = 1; i <= 10; ++i)
	{
		int d = i * 10;

		digitalWrite(pin, HIGH);
		delayMicroseconds(d);
		digitalWrite(pin, LOW);
		delayMicroseconds(d);
	}
}

void test_delayMicroseconds2(unsigned int microSeconds)
{
	unsigned long t1 = micros();
	delayMicroseconds(microSeconds);
	unsigned long t2 = micros();

	printf("Time: %luus\n", t2 - t1);
}

void test_pinToPort()
{
	gpio_setdir(ALL_GPIOS);

	int i;
	
	// pulse pin 10 using PORT pointer	
	printf("__ Using PORT ___________________________________________\n");
	*PORT = 0; printf("PORT: %0X\n", *PORT);
	
	for (i = 0; i < 10; ++i)
	{
		*PORT = 1 << 10;
		printf("PORT: %0X\n", *PORT);
		*PORT = 0;
		printf("PORT: %0X\n", *PORT);
	}

	// pulse pin 10 using portInputRegister
	volatile uint32_t *pin10port = portInputRegister(digitalPinToPort(10));
	uint32_t pin10mask = digitalPinToBitMask(10);
	
	printf("__ Using portInputRegister ______________________________\n");
	*PORT = 0; printf("PORT: %0X\n", *PORT);

	for (i = 0; i < 10; ++i)
	{
		*pin10port = pin10mask;
		printf("PORT: %0X, val: %0X\n", *PORT, *pin10port);
		*pin10port = 0;
		printf("PORT: %0X, val: %0X\n", *PORT, *pin10port);
	}
}


int main(int argc, char *argv[])
{
	// Initialize GPIO library. Needed before any GPIO pins are used. 
	gpio_init();

	// Set I2C and SPI shared pins to GPIO.
	// This makes 6 more GPIOs available to us (1,2 -> I2C, 3,4,5,6 -> SPI)
	gpio_set_mode(SPI_MODE_GPIO | I2C_MODE_GPIO);

	// Set direction of all GPIOs to OUTPUT
	gpio_setdir(ALL_GPIOS);  

	//test_gpio_set();
	//test_cbi_sti();
	//test_digitalWrite();
	//test_digitalRead();
	//test_delay();
	//test_delayMicroseconds();
	//test_delayMicroseconds2(atoi(argv[1]));
	test_pinToPort();
	
	// Cleanup GPIO library
	gpio_dispose();

	return 0;
}
