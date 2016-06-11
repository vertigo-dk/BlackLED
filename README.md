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
- Install [Teensyduino (1.28)](https://www.pjrc.com/teensy/td_download.html)
	- FastLED library is required when asked about libraries to install
- Install [ArtNode](https://github.com/vertigo-dk/ArtNode) 
	- `git clone https://github.com/vertigo-dk/ArtNode ~/Documents/Arduino/libraries/ArtNode`
- Open Arduino IDE, select Teensy 3.1/3.2 from Tools -> Board 

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
can't run more than 6 DMX universes

Version history
------------

### v0.1
- initial version
