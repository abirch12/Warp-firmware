## Overview - Alice Birch adb82, Pembroke College
This version of the Warp firmware runs with the FRDM KL03 development board; the 96x64 OLED with SSD1331 driver and the MAXREFDES117 heart rate module with MAX30102. 
`boot.c` runs an algorithm to output the user's blood oxygen (SpO2) level and heart rate in bpm on the OLED display.

## Wiring
In order for the firmware to run as intended, connect your components to the FRDM KL03 as follows:

J2.2  - SSD1331 OC

J2.3  - MAXREFDES117 INT

J2.6  - SSD1331 R

J2.9  - MAXREFDES117 SDA

J2.10 - MAXREFDES117 SCL


J3.4  - MAXREFDES117 Vin

J3.5  - SSD1331 +

J3.7  - MAXREFDES117, SSD1331 GND


J4.1  - SSD1331 CK

J4.2  - SSD1331 SI

J4.3  - SSD1331 DC

## Changed Files
The following files were changed for this project: 

`CMakeLists-Warp.txt`

`devMAXREFDES117.*`

`devSSD1331.*`

`algorithm.*`

`config.h`

`gpio_pins.*`

`warp.h`

`boot.c`

## Source File Descriptions

##### `CMakeLists-Warp.txt`
This is the CMake configuration file. Edit this to change the default size of the stack and heap.

##### `SEGGER_RTT.*`
This is the implementation of the SEGGER Real-Time Terminal interface. Do not modify.

##### `SEGGER_RTT_Conf.h`
Configuration file for SEGGER Real-Time Terminal interface. You can increase the size of `BUFFER_SIZE_UP` to reduce text in the menu being trimmed.

##### `SEGGER_RTT_printf.c`
Implementation of the SEGGER Real-Time Terminal interface formatted I/O routines. Do not modify.

##### `devMAXREFDES117.*`
Driver for MAXREFDES117 (SpO2 and Heart Rate Monitor). Contains functions to read/write to the sensor registers, read from its FIFO, configure it, initialise it and check its interrupt flag via a GPIO input pin.  

##### `devSSD1331.*`
Driver for SSD1331 (OLED Driver). It can initialise and write commands to the screen over SPI. Functions have been added to write the characters "S, p, H, R, -, 0-9" so that the SpO2 and heart rate can be output.

##### `algorithm.*`
This file contains all the algorithms which find the heart rate and SpO2 from the raw data from the MAXREFDES117 FIFO. This involves peak finding and sorting for the heart rate, then looking at the ratio between AC and DC components of the two wavelengths for SpO2. 

##### `gpio_pins.c`
Definition of I/O pin configurations using the KSDK `gpio_output_pin_user_config_t` structure.

##### `gpio_pins.h`
Definition of I/O pin mappings and aliases for different I/O pins to symbolic names relevant to the Warp hardware design, via `GPIO_MAKE_PIN()`.

##### `startup_MKL03Z4.S`
Initialization assembler.

##### `boot.c`
The core of the implementation. The main loop of this file brings together functions in the device drivers and `algorithm.c` to run the program which outputs the user's heart rate and SpO2. 

##### `warp.h`
Constant and data structure definitions.
