Arduino-compatible GPIO library for Carambola
=============================================

This is a little library intended to make existing Arduino libraries compatible with [8devices Carambola](http://www.8devices.com/product/3/carambola). It uses GPIO registers for fast access to GPIO pins. 

The Carambola is a nice board that runs a linux distribution on top of MIPS CPU heavy oriented towards WIFI and network. This makes it ideal to connect some sensors and get the readings to the outside world through a Wifi connection. 

I made this library to reuse some of the code from existing Arduino libraries (mainly temperature sensors) and because I found no fast enough GPIO handling samples on the web. 

The available API is very basic, mainly the functions I needed for a couple of DHT22 temperature sensors, but it is fast. I've measured pulses to around 100ns.


Compiling
---------

Follow the instructions in the Carambola Wiki to get a cross-compile environment set up. There is a Makefile.dave file that defines the cross-compile tool chain. Modify the STAGING_DIR variable defined there to your toolchain installation (you can use the existing value as a hint). Typing 'make' should get you no errors or warnings and a 'testgpio' executable ready to be deployed in the Carambola. The library is compiled to libgpio.o.


Using the library
-----------------

To include the library in your programs, you can take compile it standalone and link to .o file, including WProgram.h and libgpio.h in your include path. 

Before you can use the gpio functions, call 

    gpio_init();

on your program. This maps the GPIO registers to the process memory and the fun begins. 

You can now call sbi/cbi, digitalRead/digitalWrite/pinMode or use the PORT pointer to directly write the GPIO register.


Configuring Carambola GPIO
--------------------------

I've included a function gpio_set_mode(mode) to control how the GPIO pins are shared with I2C and SPI. Call it with 

    gpio_set_mode(SPI_MODE_GPIO | I2C_MODE_GPIO);

to make all the pins available as GPIO lines.


Arduino available functions
---------------------------

I've implemented the functions I needed to port a couple simple temperature sensor libraries, but the basics are there for most of the applications. 


### Timing functions

* delay() and delayMicroseconds() are implemented and pretty accurate (3us measured error in the analyzer). 
* millis() and micros() are also implemented


### Pin manipulation

* sbi() / cbi() 
* pinMode() / digitalRead() / digitalWrite()
* digitalPinToPort() / digitalPinToBitMask() are implemented and work based on the PORT pointer (as described above).

