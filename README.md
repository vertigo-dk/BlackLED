# BlackLED - Ofelia edition
Software for the BlackLED controller  -> branch for the Wave installation at Ofelia plads

## Introduction
Code for the Teensy 3.1/3.2 running the BlackLED controller. Compatible with Arduino IDE.

## Licence
The code in this repository is available under the MIT License.

## Installation
1. Install [Arduino (1.6.11)](https://www.arduino.cc/en/Main/OldSoftwareReleases#previous)
- Install [Teensyduino (1.30)](https://www.pjrc.com/teensy/td_download.html)
- Install [ArtNode lib](https://github.com/vertigo-dk/ArtNode)
- Install modified [OctoWS2811 lib](https://github.com/alex-Arc/OctoWS2811)
- Install modified [Ethernet lib](https://github.com/alex-Arc/Ethernet/tree/Selectable-socket-number)
- In ```~/Arduino.app/Contents/Java/hardware/teensy/avr/boards.txt```

	uncoment line 430 ```teensy31.menu.speed.120opt=120 MHz optimize speed (overclock)```
- In ```~/Arduino.app/Contents/Java/hardware/teensy/avr/cores/teensy3/SPIFIFO.h```
 - add ```#define SPI_CLOCK_30MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR) //(60 / 2) * ((1+1)/2) = 30 MHz``` ```to the #if F_BUS == 60000000 section``` line 73
 - add ```#define SPI_CLOCK_30MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR) //(60 / 2) * ((1+1)/2) = //(56 / 2) * ((1+1)/2) = 28 MHz``` to the ```#if F_BUS == 56000000 section``` line 83

- Open Arduino IDE
 - In ```Tools -> Board```  ```Teensy 3.1/3.2```
 - in ```Tools -> CPU speed``` select
	and 120 MHz optimized speed (overclock)

## Network setup
The host computer has to be on the 2.x.x.x network (for example 2.0.0.1), and the subnet must be 255.0.0.0.

The nodes get an calculated IP address from the MAC address accordingly to ArtNet 3 specifications in the 2.x.x.x space.

## OSC setup
a message will be sent to 2.0.0.1 on UDP port 49161, every time there is a change in the beam state
the format is ```/BeamBreak/[Art-Net start address] [beam state(0-1)]```

## Dependencies
- [ArtNode lib](https://github.com/vertigo-dk/ArtNode)
- [Ethernet lib](https://github.com/alex-Arc/Ethernet/tree/Selectable-socket-number)
	modified version with that uses only 1 socket at full memory size
- [OctoWS2811 lib](https://github.com/alex-Arc/OctoWS2811) modified to run 4 channel LEDs
- [OSC](https://github.com/CNMAT/OSC)


## Compatibility
- teensy 3.1/3.2 with WIZ820io

## Known issues
at over 18 DMX universes it will begin to drop packages

---

## Version history

#### v0.8.6-Beta.1
##### New features
- No OpPoll for 20 sec. will result in blackout

---

#### v0.8.5-Beta
##### New features
- LED chip selection through defines
 - currently supports ws28 types and LPD8806
- Set number of leds through defines and auto fit to DMX size
- Set number of outputs through defines

##### Changes
- Octows2811 now external lib

---

#### no history back this far
