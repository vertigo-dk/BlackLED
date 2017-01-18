//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//!!!!!!!!!!!!!!!!!! OFELIA spesific !!!!!!!!!!!!!!!!!!!
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//#include <OSCMessage.h>
#include <vector>


uint16_t OSCoutPort = 49161;
#define beam_break_pin 23
uint8_t  beam_break_stat = 1;

typedef struct S_ColorConfig {
  uint8_t  brightness;
  uint8_t  red;
  uint8_t  green;
  uint8_t  blue;

} ColorConfig_t;


ColorConfig_t colorConfig = {
  255,  // Brghtness 100%
  255,  // Red 100 %
  204,  // Green 80%
  102,  // Blue 40%
};

#define COLOR_CONFIG_MEM_START (CONFIG_MEM_START+sizeof(ArtConfig)-CONFIG_START-CONFIG_END+3)
#define COLOR_CONFIG_VERSION "ls1"

String oscAddr;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// initial user defined settings
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define NUM_OF_OUTPUTS 6
#define MAX_NUM_LED_PER_OUTPUT 260

#define num_artnet_ports 10

//#define blackOnOpSyncTimeOut
//#define blackOnOpPollTimeOut
const static uint32_t OpSyncTimeOut = 300000; //recoment more than 20000 ms
const static uint32_t OpPollTimeOut = 30000;  //recoment more than 20000 ms

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// includes and lib
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <Ethernet.h>
#include <ArtNode.h>
#include "ArtNetFrameExtension.h"

#include <FastLED.h>

#include "TeensyMAC.h"
#include <EEPROM.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// real code starts hear
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define VERSION_HI 0
#define VERSION_LO 85

#define PIN_RESET 9

EthernetUDP udp;
uint8_t udp_buffer[600];

boolean locateMode = 0;

// variables for the node.report
float tempVal = 0;
float fps = 0;
float avgUniUpdated = 0;
uint8_t numUniUpdated = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

uint32_t lastPoll = 0;
uint32_t lastSync = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// FastLED setup
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CRGB leds[2080]; //116 + 260 + 260 + 260 + 260 +116 +(ofset 144*2 + 260*2)

/*
uint8_t *ledOut_1 = (uint8_t*)leds;          //fod
uint8_t *ofset_1 = ledOut_1 + 348;
uint8_t *ledOut_2 = ofset_1 + 432;        //arm
uint8_t *ofsetOut_3 = ledOut_2 + 780;        //offset
uint8_t *ledOut_4 = ofsetOut_3 + 780;        //arm
uint8_t *ledOut_5 = ledOut_4 + 780;        //arm
uint8_t *ofset_6 = ledOut_5 + 780;         //offset
uint8_t *ledOut_7 = ofset_6 + 780;        //arm
uint8_t *ledOut_8 = ledOut_7 + 348;          //fod
uint8_t *ofset_8 = ledOut_8 + 348;
*/

CRGB *ledOut_1 = leds;
CRGB *ledOut_2 = ledOut_1+260;
CRGB *ledOut_3 = ledOut_2+260;    //empty
CRGB *ledOut_4 = ledOut_3+260;
CRGB *ledOut_5 = ledOut_4+260;
CRGB *ledOut_6 = ledOut_5+260;    //empty
CRGB *ledOut_7 = ledOut_6+260;
CRGB *ledOut_8 = ledOut_7+260;

uint8_t* dmxOut_0 = (uint8_t*)ledOut_1;
uint8_t* dmxOut_1 = (uint8_t*)ledOut_2;
uint8_t* dmxOut_2 = (uint8_t*)dmxOut_1+512;
uint8_t* dmxOut_3 = (uint8_t*)ledOut_4;
uint8_t* dmxOut_4 = (uint8_t*)dmxOut_3+512;
uint8_t* dmxOut_5 = (uint8_t*)ledOut_5;
uint8_t* dmxOut_6 = (uint8_t*)dmxOut_5+512;
uint8_t* dmxOut_7 = (uint8_t*)ledOut_7;
uint8_t* dmxOut_8 = (uint8_t*)dmxOut_7+512;
uint8_t* dmxOut_9 = (uint8_t*)ledOut_8;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Art-Net config
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ArtConfig config = {
  {0xDE, 0xAD, 0xBE, 0x00, 0x00, 0x00}, // MAC - last 3 bytes set by Teensy
  {2, 0, 0, 1},                         // IP
  {255, 0, 0, 0},                       // Subnet mask
  0x1936,                               // UDP port
  false,                                // DHCP

  // These fields get overwritten by loadConfig:
  0, 0,                                 // Net (0-127) and subnet (0-15)
  "BlackLED_6",                           // Short name
  "BlackLED_6_port",                     // Long name
  num_artnet_ports, // Number of ports
  { PortTypeDmx | PortTypeOutput,
    PortTypeDmx | PortTypeOutput,
    PortTypeDmx | PortTypeOutput,
    PortTypeDmx | PortTypeOutput
  }, // Port types
  {0, 0, 0, 0},                         // Port input universes (0-15)
  {0, 1, 2, 3},                          // Port output universes (0-15)
  VERSION_HI,
  VERSION_LO
};

ArtNodeExtended node;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// artnetSend - takes a buffer pointer and its length
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void artnetSend(byte* buffer, int length) {
  udp.beginPacket(node.broadcastIP(), config.udpPort);
  udp.write(buffer, length);
  udp.endPacket();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Blink test all the leds full white
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void blink() {
  //delay(rand()%200); //random distribute the startup blink to distribute the power draw
  for (int i = 0; i < 260*8; i++) {
    leds[i] = 0xFFFFFF;
  }
  LEDS.show();
  delay(200);
  for (int i = 0; i < 260*8; i++) {
    leds[i] = 0x0;
  }
  LEDS.show();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// EEPROM setup
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ID of the settings block
#define CONFIG_VERSION "ls1"

// Tell it where to store your config data in EEPROM
#define CONFIG_MEM_START 16
#define CONFIG_START 17
#define CONFIG_END 2

int oemCode = 0x0000; // OemUnkown

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// loadConfig - loads configurations from EEPROM
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_MEM_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_MEM_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_MEM_START + 2) == CONFIG_VERSION[2]) {
    for (unsigned int t = CONFIG_START; t < sizeof(config) - CONFIG_END; t++) {
      *((char*)&config + t ) = EEPROM.read(CONFIG_MEM_START + t + 3 - CONFIG_START);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// saveConfig - saves configurations to EEPROM
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void saveConfig() {
  EEPROM.write(CONFIG_MEM_START + 0, CONFIG_VERSION[0]);
  EEPROM.write(CONFIG_MEM_START + 1, CONFIG_VERSION[1]);
  EEPROM.write(CONFIG_MEM_START + 2, CONFIG_VERSION[2]);
  for (unsigned int t = CONFIG_START; t < sizeof(config) - CONFIG_END; t++) {
    EEPROM.write(CONFIG_MEM_START + t - CONFIG_START + 3, *((char*)&config + t));
  }
}
//----------------------------------------------------------------
//-----OFELIA-----------------------------------------------------
//----------------------------------------------------------------
/*
void saveColorConfig() {
  EEPROM.write(COLOR_CONFIG_MEM_START + 0, COLOR_CONFIG_VERSION[0]);
  EEPROM.write(COLOR_CONFIG_MEM_START + 1, COLOR_CONFIG_VERSION[1]);
  EEPROM.write(COLOR_CONFIG_MEM_START + 2, COLOR_CONFIG_VERSION[2]);
  for (unsigned int t = 0; t < sizeof(colorConfig); t++) {
    EEPROM.write(COLOR_CONFIG_MEM_START + t + 3, *((char*)&colorConfig + t));
  }
}
void loadColorConfig() {
   // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(COLOR_CONFIG_MEM_START + 0) == COLOR_CONFIG_VERSION[0] &&
      EEPROM.read(COLOR_CONFIG_MEM_START + 1) == COLOR_CONFIG_VERSION[1] &&
      EEPROM.read(COLOR_CONFIG_MEM_START + 2) == COLOR_CONFIG_VERSION[2]) {
    for (unsigned int t = 0; t < sizeof(colorConfig); t++) {
      *((char*)&colorConfig + t ) = EEPROM.read(COLOR_CONFIG_MEM_START + t + 3);
    }
  }
}
*/
//----------------------------------------------------------------
//----------------------------------------------------------------

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// setup
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  //saveConfig(); //<-- uncomment to force the EEPROM config to your settings on eatch reboot
  ArtConfig tempConfig = config;
  loadConfig();
  config.numPorts = tempConfig.numPorts;
  config.numPorts = tempConfig.numPorts;
  config.verHi = tempConfig.verHi;
  config.verLo = tempConfig.verLo;
  saveConfig();

  //-----OFELIA------
  pinMode(beam_break_pin, INPUT_PULLUP);
  String startAddr = String(node.getStartAddress(), DEC);
  oscAddr = String("/BeamBreak/" + startAddr);

  /*if (getStartAddress() > 99) {

  }else if (getStartAddress() > 9) {
  }*/
  //-----------------


#ifdef PIN_RESET
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_RESET, HIGH);
  delay(150);
#endif

  // Read MAC address
  mac_addr mac;
  for (int i = 3; i < 6; i++) {
    config.mac[i] = mac.m[i];
  }

  // Calculate IP address
  config.ip[0] = 2;
  config.ip[1] = config.mac[3] + (oemCode & 0xFF);// + ((oemCode >> 16) & 0xFF);
  config.ip[2] = config.mac[4];
  config.ip[3] = config.mac[5];

  // Open Ethernet connection
  IPAddress gateway(config.ip[0], 0, 0, 1);
  IPAddress subnet(255, 0, 0, 0);

  Ethernet.begin(config.mac, config.ip,  gateway, gateway, subnet);
  udp.begin(config.udpPort);

  // Open ArtNet
  node = ArtNodeExtended(config, sizeof(udp_buffer), udp_buffer);
  // start FastLED
  LEDS.addLeds<WS2811_PORTD, 8>(leds, 260);
  LEDS.setCorrection(colorConfig.red, colorConfig.green, colorConfig.blue);
  LEDS.setDither(BINARY_DITHER);
  //blink();

  // to read internal temperature
  analogReference(INTERNAL);
  analogReadResolution(12);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// main loop
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  //-----OFELIA------
  if(digitalRead(beam_break_pin) != beam_break_stat) {

    beam_break_stat = digitalRead(beam_break_pin);
    //char addr[15];
    //oscAddr.toCharArray(addr, 15);
    char oscStr[23] = {0x2f, 0x42, 0x65, 0x61, 0x6d, 0x42, 0x72, 0x65, 0x61, 0x6b, 0x2f, 0x30, 0x30, 0x30, 0x00, 0x00, 0x2c, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00};

    char dig3;
    char dig2;
    char dig1;
    int addrINT = node.getStartAddress();
    dig3 = addrINT/100;
    dig2 = (addrINT-dig3)/10;
    dig1 = addrINT-(dig3*100)-(dig2*10);

    oscStr[11] = dig3 + 0x30;
    oscStr[12] = dig2 + 0x30;
    oscStr[13] = dig1 + 0x30;
    //memcpy(&oscStr+11, addrCHAR, 3);
    udp.beginPacket(IPAddress(2, 0, 0, 1), OSCoutPort);
    udp.write(oscStr, 23);
    udp.write(beam_break_stat);
    udp.endPacket();
    /*OSCMessage msg(addr);
    msg.add(beam_break_stat);
    udp.beginPacket(IPAddress(2, 0, 0, 1), OSCoutPort);
    msg.send(udp);
    udp.endPacket();
    msg.empty();*/
  }
  //-----------------
  while (udp.parsePacket()) {
    // First read the header to make sure it's Art-Net
    unsigned int n = udp.read(udp_buffer, sizeof(ArtHeader));
    if (n >= sizeof(ArtHeader)) {
      ArtHeader* header = (ArtHeader*)udp_buffer;
      // Check packet ID
      if (memcmp(header->ID, "Art-Net", 8) == 0) {  //is Art-Net
        // Read the rest of the packet
        udp.read(udp_buffer + sizeof(ArtHeader), udp.available());
        // Package Op-Code determines type of packet
        switch (header->OpCode) {

          // Poll packet
          case OpPoll: {
              //T_ArtPoll* poll = (T_ArtPoll*)udp_buffer;
              //if(poll->TalkToMe & 0x2){

              #ifdef blackOnOpPollTimeOut
                lastPoll = millis();
              #endif

              float tempCelsius = 25.0 + 0.17083 * (2454.19 - tempVal);
              sprintf(node.pollReport, "numOuts;%d;numUniPOut;%d;temp;%.1f;fps;%.1f;uUniPF;%.1f;", NUM_OF_OUTPUTS, 3, tempCelsius, fps, avgUniUpdated);
              node.createPollReply(); //create pollReply
              artnetSend(udp_buffer, sizeof(ArtPollReply)); //send pollReply
              //}
              break;
            }

          // DMX packet
          case OpDmx: {
              ArtDmx* dmx = (ArtDmx*)udp_buffer;
              int port = node.getAddress(dmx->SubUni, dmx->Net) - node.getStartAddress();
              if (port >= 0 && port < config.numPorts) {
                switch (port) {
                  case 0:
                  memcpy(dmxOut_0, dmx->Data, 512);
                  break;
                  case 1:
                  memcpy(dmxOut_1, dmx->Data, 512);
                  break;
                  case 2:
                  memcpy(dmxOut_2, dmx->Data, 512);
                  break;
                  case 3:
                  memcpy(dmxOut_3, dmx->Data, 512);
                  break;
                  case 4:
                  memcpy(dmxOut_4, dmx->Data, 512);
                  break;
                  case 5:
                  memcpy(dmxOut_5, dmx->Data, 512);
                  break;
                  case 6:
                  memcpy(dmxOut_6, dmx->Data, 512);
                  break;
                  case 7:
                  memcpy(dmxOut_7, dmx->Data, 512);
                  break;
                  case 8:
                  memcpy(dmxOut_8, dmx->Data, 512);
                  break;
                  case 9:
                  memcpy(dmxOut_9, dmx->Data, 512);
                  break;
                }
                /*switch (port) {
                case 0:
                  memcpy(ledOut_1, dmx->Data, 348);
                  memcpy(ledOut_2, dmx->Data+348, 164);
                break;
                case 1:
                  memcpy(ledOut_2+164, dmx->Data, 512);
                break;
                case 2:
                  memcpy(ledOut_2+164+512, dmx->Data, 104);
                  memcpy(ledOut_4, dmx->Data+104, 408);
                break;
                case 3:
                  memcpy(ledOut_4+408, dmx->Data, 372);
                  memcpy(ledOut_5, dmx->Data+372, 140);
                break;
                case 4:
                  memcpy(ledOut_5+140, dmx->Data, 512);
                break;
                case 5:
                  memcpy(ledOut_5+140+512, dmx->Data, 128);
                  memcpy(ledOut_7, dmx->Data+128, 384);
                break;
                case 6:
                  memcpy(ledOut_7+384, dmx->Data, 396);
                  memcpy(ledOut_8, dmx->Data+396, 116);
                break;
                case 7:
                  memcpy(ledOut_8+116, dmx->Data, 232);
                break;
              }*/
                numUniUpdated++;
              }
              break;
            }

          // OpSync
          case 0x5200: {
              FastLED.show();

              #ifdef blackOnOpSyncTimeOut
                lastSync = millis();
              #endif

              // calculate framerate
              currentMillis = millis();
              if(currentMillis > previousMillis){
                fps = 1 / ((currentMillis - previousMillis) * 0.001);
              } else {
                fps = 0;
              }
              previousMillis = currentMillis;

              // calculate average universes Updated
              avgUniUpdated = numUniUpdated * 0.16 + avgUniUpdated * 0.84;
              numUniUpdated = 0;

              break;
            }

          // OpAddress
          case OpAddress: {

              T_ArtAddress * address = (T_ArtAddress*)udp_buffer;

              if (address->LongName[0] != 0) {
                memcpy(config.longName, address->LongName, 64);
              }
              if (address->ShortName[0] != 0) {
                memcpy(config.shortName, address->ShortName, 18);
              }
              if (address->NetSwitch != 0x7F) {               // Use value 0x7f for no change.
                if ((address->NetSwitch & 0x80) == 0x80) { // This value is ignored unless bit 7 is high. i.e. to program a  value 0x07, send the value as 0x87.
                  config.net = address->NetSwitch & 0x7F;
                }
              }
              if (address->SubSwitch != 0x7F) {               // Use value 0x7f for no change.
                if ((address->SubSwitch & 0x80) == 0x80) { // This value is ignored unless bit 7 is high. i.e. to program a  value 0x07, send the value as 0x87.
                  config.subnet = address->SubSwitch & 0x7F;
                }
              }
              for (int i = 0; i < 4; i++) {
                if (address->SwIn[i] != 0x7F) {
                  if ((address->SwIn[i] & 0x80) == 0x80) {
                    config.portAddrIn[i] = address->SwIn[i] & 0x7F;
                  }
                }
                if (address->SwOut[i] != 0x7F) {
                  if ((address->SwOut[i] & 0x80) == 0x80) {
                    config.portAddrOut[i] = address->SwOut[i] & 0x7F;
                  }
                }
              }

              if (address->Command == 0x04) {
                locateMode = true;
              } else {
                locateMode = false;
              }
              node = ArtNodeExtended(config, sizeof(udp_buffer), udp_buffer);
              saveConfig();
              loadConfig();
              node.createPollReply();
              artnetSend(udp_buffer, sizeof(ArtPollReply));
              //node.createExtendedPollReply();
              //artnetSend(udp_buffer, node.sizeOfExtendedPollReply());
              break;
            }

          // Unhandled packet
          default: {
              break;
            }
        }

        // answer routine for Art-Net Extended
      } else if (memcmp(header->ID, "Art-Ext", 8) == 0) {
        // Read the rest of the packet
        udp.read(udp_buffer + sizeof(ArtHeader), udp.available());
        // Package Op-Code determines type of packet
        switch (header->OpCode) {
          // ArtNet Frame Extension
          case OpPoll | 0x0001: {
              node.createExtendedPollReply();
              artnetSend(udp_buffer, node.sizeOfExtendedPollReply());
              break;
            }
        }
      }else if(memcmp(header->ID, "MadrixN", 8) == 0){
        #ifdef _use_octoWS2811
        LEDS.show();
        #endif
        #ifdef _use_FastLED
        FastLED.show();
        #endif

        #ifdef blackOnOpSyncTimeOut
          lastSync = millis();
        #endif

        // calculate framerate
        currentMillis = millis();
        if(currentMillis > previousMillis){
          fps = 1 / ((currentMillis - previousMillis) * 0.001);
        } else {
          fps = 0;
        }
        previousMillis = currentMillis;

        // calculate average universes Updated
        avgUniUpdated = numUniUpdated * 0.16 + avgUniUpdated * 0.84;
        numUniUpdated = 0;
      }
    }
  }

  // read temperature value
  tempVal = analogRead(38) * 0.01 + tempVal * 0.99;

  #ifdef blackOnOpSyncTimeOut
    currentMillis = millis();
    if (currentMillis - lastSync > OpSyncTimeOut) {
      for (int i = 0; i < num_led_per_output * 8; i++) {
        LEDS.setPixel(i, 0);
      }
      LEDS.show();
    }
  #endif

  #ifdef blackOnOpPollTimeOut
    currentMillis = millis();
    if (currentMillis - lastPoll > OpPollTimeOut) {
      for (int i = 0; i < num_led_per_output * 8; i++) {
        LEDS.setPixel(i, 0);
      }
      LEDS.show();
    }
  #endif
}
