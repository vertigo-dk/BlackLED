# BlackLED
Software for the BlackLED controller

## Introduction
Code for the Teensy 3.1/3.2 running the BlackLED controller. Compatible with Arduino IDE.

## Licence
The code in this repository is available under the MIT License.

## Installation
  1. Install [Arduino (1.8.5 or newer)](https://www.arduino.cc/)
  2. Install [Teensyduino (1.41 or newer)](https://www.pjrc.com/teensy/td_download.html)
  3. Install [ArtNode lib](https://github.com/alex-Arc/ArtNode/tree/firmware)
  4. Install modified [OctoWS2811 lib](https://github.com/alex-Arc/OctoWS2811/tree/RGBW)
  5. Install [Ethernet lib](https://github.com/PaulStoffregen/Ethernet) and set the ```MAX_SOCK_NUM``` to 1 and uncomment ```ETHERNET_LARGE_BUFFERS```
  6. Open Arduino IDE
  7. In ```Tools -> Board``` select ```Teensy 3.1/3.2```
  8. In ```Tools -> CPU speed``` select 120 MHz(overclock)

## Network setup
The host computer has to be on the 2.x.x.x network (for example 2.0.0.1), and the subnet must be 255.0.0.0.

The nodes get an calculated IP address from the MAC address accordingly to ArtNet 3 specifications in the 2.x.x.x space.

## Compatibility
- teensy 3.1/3.2 with WIZ820io

## Known issues
at over 18 DMX universes it will begin to drop packages

---

## Version history

#### v0.8.7-Beta.1
- remove FastLED option
- test with new Ethernet lib
- test with Arduino 1.8.1 IDE

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
