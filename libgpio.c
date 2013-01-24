/*
 * Copyright (C) 2013 David Suarez
 * See LICENSE for more information.
 */
#include <sys/mman.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>

#include "libgpio.h"


#define MMAP_DEVICE "/dev/mem"
#define GPIO_BASE_ADDRESS  0x10000000
#define GPIO_BASE_SIZE  0xFFF
#define GPIO_24_DATA_OFFSET  0x620
#define GPIO_24_DIR_OFFSET  0x624

#define GPIO_MODE_OFFSET 0x60
#define SPI_MODE_MASK 0x02
#define I2C_MODE_MASK 0x01


/*
 * Direct access to the GPIO data and dir registers
 */
uint32_t *PORT;
uint32_t *DPP;


struct mem_map
{
	unsigned long base_address;
	unsigned int size;

	void *mapped_memory;
	unsigned int mapped_offset;
	unsigned int mapped_size;
};

struct mem_map gpio_mem;


// __ Helpers _________________________________________________________________


static void debug(char *msg, ...)
{
	va_list argptr;
    va_start(argptr, msg);
    vfprintf(stderr, msg, argptr);
    va_end(argptr);
}


static void die(char *msg, ...)
{
	va_list argptr;
    va_start(argptr, msg);
    vfprintf(stderr, msg, argptr);
    va_end(argptr);
	exit(1);
}


static inline uint32_t readptr(void *ptr)
{
	uint32_t *data = ptr;
	return *data;
}

static inline void writeptr(void *ptr, uint32_t value)
{
	uint32_t *data = ptr;
	*data = value;
}



static void init_gpio_map()
{
	memset(&gpio_mem, 0, sizeof(gpio_mem));


    int npages = 0;
    int pagesize = getpagesize();

    gpio_mem.mapped_offset = GPIO_BASE_ADDRESS & (pagesize - 1);
    gpio_mem.base_address = GPIO_BASE_ADDRESS &  ~(pagesize - 1);

    npages += (GPIO_BASE_SIZE / pagesize);
    npages += 1;
    gpio_mem.mapped_size = npages * pagesize;

    //debug("[Base: %0lX, offset: %0lX, size: %i, pagesize: %i]\n", gpio_mem.base_address, gpio_mem.mapped_offset, gpio_mem.mapped_size, pagesize);
}


// __ Public API ______________________________________________________________


void gpio_set_mode(uint32_t mode)
{
	uint32_t *addr = gpio_mem.mapped_memory + gpio_mem.mapped_offset + GPIO_MODE_OFFSET;
	uint32_t modeMask = 0x03;	
	uint32_t xh = (*addr & ~modeMask) | mode;
	*addr = xh;

	//debug("GPIO_MODE: %0lX = %0lX\n", addr, *addr);
}

void gpio_init()
{
	init_gpio_map();

	int iofd = open(MMAP_DEVICE, O_RDWR);
	if (iofd < 0)
		die("open failed\n");

	gpio_mem.mapped_memory = mmap(NULL, gpio_mem.mapped_size,
			 PROT_READ|PROT_WRITE,
			 MAP_SHARED,
			 iofd, gpio_mem.base_address);

	if (gpio_mem.mapped_memory == MAP_FAILED)
		die("mmap failed for %0lX\n", gpio_mem.base_address);

	//debug("MMAP: %0lX mapped to: %0lX\n", gpio_mem.base_address, gpio_mem.mapped_memory);

	PORT = gpio_mem.mapped_memory + gpio_mem.mapped_offset + GPIO_24_DATA_OFFSET;
	DPP = gpio_mem.mapped_memory + gpio_mem.mapped_offset + GPIO_24_DIR_OFFSET;

	//debug("MMAP: PORT=%0lX, DPP=%0lX\n", PORT, DPP);

	close(iofd);
}

void gpio_dispose()
{
	if (munmap(gpio_mem.mapped_memory, gpio_mem.mapped_size))
		die("munmap failed for %lX\n", gpio_mem.base_address);
}


void gpio_setdir(uint32_t value)
{
	*DPP = value;
}

uint32_t gpio_getdir()
{
	return *DPP;
}


inline void gpio_set(uint32_t value)
{
	//debug("GPIO_SET: %0lX -> %0lX\n", PORT, value);
	*PORT = value;
}

inline uint32_t gpio_get()
{
	//debug("GPIO_GET: %0lX:   %0lX\n", PORT, *PORT);
	return *PORT;
}


// __ Arduino compatible API __________________________________________________


uint32_t *digitalPinToPort(uint8_t pin)
{
	return PORT;
}

uint32_t digitalPinToBitMask(uint8_t pin)
{
	return 1 << pin;
}

void cbi(uint32_t *addr, uint32_t bitmask)
{
	*addr &= ~bitmask;
}

void sbi(uint32_t *addr, uint32_t bitmask)
{
	*addr |= bitmask;
}

void pinMode(uint8_t pin, uint8_t mode)
{
	uint32_t mask = 1 << pin;
	uint32_t val = mode << pin;
	uint32_t newVal = (*DPP & ~mask) | val;
	*DPP = newVal;
}

void digitalWrite(uint8_t pin, uint8_t value)
{
	uint32_t mask = 1 << pin;
	uint32_t val = value << pin;
	uint32_t newVal = (*PORT & ~mask) | val;
	*PORT = newVal;
}

uint8_t digitalRead(uint8_t pin)
{
	uint32_t mask = 1 << pin;
	uint32_t result = *PORT & mask;
	return (uint8_t)(result >> pin);
}

void delayMicroseconds(unsigned int delayMicroseconds)
{
	unsigned long t2, t1 = micros();

	do
	{
		t2 =  micros();
	}
	while ((t2 - t1) < delayMicroseconds);
}

void delay(unsigned int ms)
{
	usleep(ms * 1000);
}

unsigned long micros()
{
	struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now.tv_sec * 1000000 + now.tv_nsec / 1000;
}


unsigned long millis()
{
	return micros() / 1000;
}

void cli()
{
	// clear interrupts. We don't implement that yet.
}

void sei()
{
	// set interrupts. We don't implement that yet. 
}
