# VardeLED
Software for the VardeLED controller

Introduction
------------
Code for the Teensy LC running the VardeLED controller. Compatible with Arduino IDE.

Licence
-------
The code in this repository is available under the MIT License.

Installation
------------
- Install [Teensyduino (1.25)](https://www.pjrc.com/teensy/td_download.html)
	- FastLED library is required when asked about libraries to install
- Install [ArtNode](https://github.com/vertigo-dk/ArtNode) 
	- `git clone https://github.com/vertigo-dk/ArtNode ~/Documents/Arduino/libraries/ArtNode`
- Open Arduino IDE, select Teensy LC from Tools -> Board 

Network setup
-----------
The host computer has to be on the 2.x.x.x network (for example 2.0.0.1), and the subnet must be 255.0.0.0. 

The nodes get an calculated IP address from the MAC address accordingly to ArtNet 3 specifications in the 2.x.x.x space.

Dependencies
------------


Compatibility
------------


Known issues
------------
none

Varde specfic informations
------------
to maximise the usage of the boards, they are configured differently regarding the usage.
The following defines need to be changed:

- NUM_PIXELS_OUT_1 288
- NUM_PIXELS_OUT_2 1
- NUM_PIXELS_OUT_3 1
- NUM_PIXELS_OUT_4 1

for the 2m Poles

- 288,1,1,1

on the ramp, the first two boxes / 2x 3 boards are flashed with

- 70,70,70,70

all the others on the ramp up to a size of <90cm

- 130, 130, 130, 1

box 10 & 11, with longer fixtures >90cm

- 160, 1 , 160, 1

For the stairs on both sides

-  130,130,130,1 (board for the smaler fixtures)
-  140, 1, 130, 130 (board for the longer fixtures)

Those under the roof run with

- 130, 130 ,1 ,1 (or 130, 130, 130, 1)


Version history
------------
### v0.5
- like 0.4
- address is now implemented correctly ArtNet specification confirm.


### v0.4
- Varde for WS2813 inkl color correction
- opAddress is not implemented correctly no masking of 0x7F

### v0.1
- initial version
