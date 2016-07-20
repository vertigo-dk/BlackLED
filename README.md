# BlackLED
Software for the BlackLED controller

Introduction
------------
Code for the Teensy 3.1/3.2 running the BlackLED controller. Compatible with Arduino IDE.

Licence
-------
The code in this repository is available under the MIT License.

Installation
------------
- Install [Arduino (1.68)](https://www.arduino.cc/en/Main/OldSoftwareReleases#previous)
- Install [Teensyduino (1.28)](https://www.pjrc.com/teensy/td_download.html)
- Install [ArtNode](https://github.com/vertigo-dk/ArtNode)
- In ~/Arduino.app/Contents/Java/hardware/teensy/avr/boards.txt
	uncoment the line 55 "teensy31.menu.speed.120opt=120 MHz optimized (overclock)"
- In ~/Arduino.app/Contents/Java/hardware/teensy/avr/cores/teensy3/SPIFIFO.h
	to the #if F_BUS == 60000000 section add:
	"#define SPI_CLOCK_30MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR) //(60 / 2) * ((1+1)/2) = 30 MHz"
	to the #if F_BUS == 56000000 section add:
	"#define SPI_CLOCK_30MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR) //(60 / 2) * ((1+1)/2) = //(56 / 2) * ((1+1)/2) = 28 MHz"
- Open Arduino IDE, select Teensy 3.1/3.2 from Tools -> Board
	and 120 MHz optimized (overclock) from Tools -> CPU speed


Network setup
-----------
The host computer has to be on the 2.x.x.x network (for example 2.0.0.1), and the subnet must be 255.0.0.0.

The nodes get an calculated IP address from the MAC address accordingly to ArtNet 3 specifications in the 2.x.x.x space.

Dependencies
------------
- [ArtNode](https://github.com/vertigo-dk/ArtNode)
- [Ethernet](https://github.com/alex-Arc/Ethernet/tree/Selectable-socket-number)
	modified version with that uses only 1 socket at full memory size


Compatibility
------------
- teensy 3.1/3.2 with WIZ820io

Known issues
------------
at 18 DMX universes it will begin to drop packages

Version history
------------

### v0.8  Developer
