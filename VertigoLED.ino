//#define USE_OCTOWS2811
//#include<OctoWS2811.h>

#include <FastLED.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArtNode.h>
//#include "ArtNetFrameExtension.h"

#include "TeensyMAC.h"
#include <EEPROM.h>

#define VERSION_HI 0
#define VERSION_LO 5

// Set number of pixels (RGBW) per output
#define NUM_PIXELS_OUT 288
#define NUM_PIXELS_OUT_RGB int(ceil(NUM_PIXELS_OUT*4.0/3.0))

//Artnet universe
#define RGBW_PER_UNIVERSE   128
#define RGB_PER_UNIVERSE    int(ceil(RGBW_PER_UNIVERSE*4.0/3.0))

#define PIN_RESET 9
#define PIN_DEBUG 1

// ID of the settings block
#define CONFIG_VERSION "ls1"

// Tell it where to store your config data in EEPROM
#define CONFIG_MEM_START 16
#define CONFIG_START 17
#define CONFIG_END 2

EthernetUDP udp;
byte udp_buffer[600];

// Fucked up lookup....
int lookup[] = {1, 0, 2, 4, 3, 5, 7, 6, 8, 10, 9, 11};

CRGB led_data[NUM_PIXELS_OUT_RGB * 8];
byte * led_data_ptr_1 = (byte*)led_data + 1 * NUM_PIXELS_OUT_RGB * 3; // Sending 3 Universes on first pin out (A B C)
byte * led_data_ptr_2 = (byte*)led_data + 3 * NUM_PIXELS_OUT_RGB * 3; // Sending 3 Universes on first pin out (B) 
byte * led_data_ptr_3 = (byte*)led_data + 5 * NUM_PIXELS_OUT_RGB * 3; // Sending 2 Universes on first pin out (C D) 
byte * led_data_ptr_4 = (byte*)led_data + 7 * NUM_PIXELS_OUT_RGB * 3; // Sending 1 Universes on first pin out (D) 

bool locateMode = false;

int oemCode = 0x0000; // OemUnkown

////////////////////////////////////////////////////////////
ArtConfig config = {
  {0xDE, 0xAD, 0xBE, 0x00, 0x00, 0x00}, // MAC - last 3 bytes set by Teensy
  {2, 0, 0, 1},                         // IP
  {255, 0, 0, 0},                       // Subnet mask
  0x1936,                               // UDP port
  false,                                // DHCP

  // These fields get overwritten by loadConfig:
  0, 0,                                 // Net (0-127) and subnet (0-15)
  "VardeLED",                           // Short name
  "VardeLED",                           // Long name
  4,                                    // Number of ports
  {PortTypeDmx | PortTypeOutput, PortTypeDmx | PortTypeOutput, PortTypeDmx | PortTypeOutput, PortTypeDmx | PortTypeOutput}, // Port types
  {0, 0, 0, 0},                         // Port input universes (0-15)
  {0, 1, 2, 3},                          // Port output universes (0-15)
  VERSION_HI,
  VERSION_LO
};

ArtNode node;
////////////////////////////////////////////////////////////
void blink() {
  byte* d = (byte*)led_data;

  for (int i = 0; i < 8 * NUM_PIXELS_OUT_RGB * 3; i++) {
    d[i] = 30;
  }
  LEDS.show();

  delay(300);

  for (int i = 0; i <  8 * NUM_PIXELS_OUT_RGB * 3; i++) {
    d[i] = 0;
  }
  
  LEDS.show();
  delay(100);


}


////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////
void saveConfig() {
  EEPROM.write(CONFIG_MEM_START + 0, CONFIG_VERSION[0]);
  EEPROM.write(CONFIG_MEM_START + 1, CONFIG_VERSION[1]);
  EEPROM.write(CONFIG_MEM_START + 2, CONFIG_VERSION[2]);
  for (unsigned int t = CONFIG_START; t < sizeof(config) - CONFIG_END; t++) {
    EEPROM.write(CONFIG_MEM_START + t - CONFIG_START + 3, *((char*)&config + t));
  }
}

////////////////////////////////////////////////////////////
void artnetSend(byte* buffer, int length) {
  udp.beginPacket(node.broadcastIP(), config.udpPort);
  udp.write(buffer, length);
  udp.endPacket();
}
////////////////////////////////////////////////////////////
/*   START PROGRAM                                        */
////////////////////////////////////////////////////////////



void setup() {
  loadConfig();

  config.numPorts = 4;
  for (int i = 0; i < config.numPorts; i++) {
    config.portTypes[i] = PortTypeDmx | PortTypeOutput;
  }

#ifdef PIN_RESET
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_RESET, HIGH);
  delay(150);
#endif

  // Setup DEBUG pin
  pinMode(PIN_DEBUG, OUTPUT);

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
  node = ArtNode(config, sizeof(udp_buffer), udp_buffer);


  LEDS.addLeds<WS2811_PORTD, 8>(led_data, NUM_PIXELS_OUT_RGB);
  //LEDS.addLeds<OCTOWS2811>(led_data, NUM_PIXELS_OUT_RGB);
  //FastLED.setDither(0);
  

  Serial.begin(9600);

  blink();
}

////////////////////////////////////////////////////////////
void setData(byte * dmxData, byte * ledData, int dmx_length, int offset) {

  int d =  offset;
  int bankoffset = ceil(d / 12);
 
  for (int i = 0; i < dmx_length; i++) {
    int l = lookup[d % 12] + bankoffset  * 12;
    ledData[l] = *(dmxData++);

    d++; 
    if(d%12 == 0){
      bankoffset ++;
    }
  }

  //memcpy(ledData+offset, dmxData, dmx_length);
}

////////////////////////////////////////////////////////////
void loop() {

  while (udp.parsePacket()) {
    digitalWrite(PIN_DEBUG, HIGH);

    // First read the header to make sure it's Art-Net
    unsigned int n = udp.read(udp_buffer, sizeof(ArtHeader));
    if (n >= sizeof(ArtHeader)) {

      ArtHeader* header = (ArtHeader*)udp_buffer;

      if (memcmp(header->ID, "Art-Ext", 8) == 0) {
        // Read the rest of the packet
        udp.read(udp_buffer + sizeof(ArtHeader), udp.available());

        // Package Op-Code determines type of packet
        switch (header->OpCode) {
          // ArtNet Frame Extension
          case OpPoll | 0x0001:
            node.createPollReply();
            artnetSend(udp_buffer, sizeof(ArtPollReply));
            break;

        }
      }

      // Standard ArtNet commands
      else
        // Check packet ID
        if (memcmp(header->ID, "Art-Net", 8) == 0) {

          // Read the rest of the packet
          udp.read(udp_buffer + sizeof(ArtHeader), udp.available());

          // Package Op-Code determines type of packet
          switch (header->OpCode) {

            // Poll packet. Send poll reply.
            case OpPoll:
              node.createPollReply();
              artnetSend(udp_buffer, sizeof(ArtPollReply));
              break;

            // DMX packet
            case OpDmx: {
                ArtDmx* dmx = (ArtDmx*)udp_buffer;
              int port = node.getPort(dmx->SubUni, dmx->Net);
                if (!locateMode && port >= 0 && port < config.numPorts) {
                  // Calculate length of DMX packet. Requires bit swap length from dmx packet
                  int dmx_length = ((dmx->Length & 0xF) << 8) + ((dmx->Length & 0xF0) >> 8);

                  // Check if length is longer then allocated data
                  if (dmx_length > RGBW_PER_UNIVERSE * 4) {
                    dmx_length = RGBW_PER_UNIVERSE * 4;
                  }

 /*
Varde
1 A B C       2m   1m / 1.5   0.5m
2 B                           0.5m
3 C D              1m / 1.5   0.5m
4 D           0.5m            0.5m
*/
                  switch (port) {
                    case 0: {
                        setData(dmx->Data, led_data_ptr_1, dmx_length, 0);
                        break;
                      }
                    case 1: {
                        setData(dmx->Data, led_data_ptr_1, dmx_length, RGBW_PER_UNIVERSE * 4);
                        setData(dmx->Data, led_data_ptr_2, dmx_length, 0);

                        break;
                      }
                    case 2: {
                        setData(dmx->Data, led_data_ptr_1, dmx_length, RGBW_PER_UNIVERSE * 4 * 2);
                        setData(dmx->Data, led_data_ptr_3, dmx_length, 0);
                        break;
                      }
                    case 3: {
                        setData(dmx->Data, led_data_ptr_3, dmx_length, RGBW_PER_UNIVERSE * 4);
                        setData(dmx->Data, led_data_ptr_4, dmx_length, 0);
                        break;
                      }
                  }
                }
              }
              break;
            case 0x5200: { //OpSync
                LEDS.show();

                break;
              }
            case OpAddress: {
                T_ArtAddress * address = (T_ArtAddress*)udp_buffer;

                if (address->LongName[0] != 0)
                  memcpy(config.longName, address->LongName, 64);
                if (address->ShortName[0] != 0)
                  memcpy(config.shortName, address->ShortName, 18);

                if (address->NetSwitch != 0x7f)
                  config.net = address->NetSwitch;

                if (address->SubSwitch != 0x7f)
                  config.subnet = address->SubSwitch;


                for (int i = 0; i < 4; i++) {
                  if (address->SwIn[i] != 0x7f)
                    config.portAddrIn[i] = address->SwIn[i];
                  if (address->SwOut[i] != 0x7f) {
                    config.portAddrOut[i] = address->SwOut[i];
                  }
                }

                if (address->Command == 0x04) {
                  locateMode = true;
                } else {
                  locateMode = false;
                }


                node = ArtNode(config, sizeof(udp_buffer), udp_buffer);

                saveConfig();
                loadConfig();
                node.createPollReply();
                artnetSend(udp_buffer, sizeof(ArtPollReply));
                break;
              }
            // IpProg packet

            // Unhandled packet
            default:
              break;
          }
        }
    }
    digitalWrite(PIN_DEBUG, LOW);


  }

  if (locateMode) {
    /*byte* d = (byte*)led_data;
    for (int i = 0; i < NUM_RGB_LEDS_4 * 3; i++) {
      d[i * 4] = 0;
      d[i * 4 + 1] = (sin(10 + i + millis() / 100.0) + 1) * 128.0;
      d[i * 4 + 2] = 0;
      d[i * 4 + 3] = (sin(i + millis() / 100.0) + 1) * 128.0;
    }
    LEDS.show();*/

  }

  //blink();
}



