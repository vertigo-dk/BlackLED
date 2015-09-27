#include "FastLED.h"
#include <SPI.h>
#include <Ethernet.h>
#include <ArtNode.h>

#include "TeensyMAC.h"
#include <EEPROM.h>

#define VERSION_HI 0
#define VERSION_LO 2

#define NUM_RGBW  128*2
#define NUM_RGB   (NUM_RGBW/4*3)
#define PIN_RESET 9

// ID of the settings block
#define CONFIG_VERSION "ls1"

// Tell it where to store your config data in EEPROM
#define CONFIG_MEM_START 16
#define CONFIG_START 17
#define CONFIG_END 2

EthernetUDP udp;
byte buffer[600];

CRGB leds1[NUM_RGB];
CRGB leds2[NUM_RGB];
CRGB leds3[NUM_RGB];
CRGB leds4[NUM_RGB];

bool locateMode = false;

int oemCode = 0x0000; // OemUnkown

////////////////////////////////////////////////////////////
ArtConfig config = {
  {0xDE, 0xAD, 0xBE, 0x00, 0x00, 0x00}, // MAC - last 3 bytes set by Teensy
  {2, 0, 0, 1},                         // IP
  {255, 0, 0, 0},                       // Subnet mask
  0x1936,                               // UDP port
  false,                                // DHCP
  0, 0,                                 // Net (0-127) and subnet (0-15)
  "VardeLED",                           // Short name
  "VardeLED",                           // Long name
  4,                                    // Number of ports
  {PortTypeDmx|PortTypeOutput, PortTypeDmx|PortTypeOutput, PortTypeDmx|PortTypeOutput, PortTypeDmx|PortTypeOutput}, // Port types
  {0, 0, 0, 0},                         // Port input universes (0-15)
  {0, 1, 2, 3},                          // Port output universes (0-15)
  VERSION_HI,                                    
  VERSION_LO                                    
};

ArtNode node;
////////////////////////////////////////////////////////////

void setup() {
  loadConfig();
  
  config.numPorts = 4;
  for(int i=0;i<config.numPorts;i++){
    config.portTypes[i]= PortTypeDmx|PortTypeOutput;
  }
  
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
  IPAddress gateway(config.ip[0],0,0,1);
  IPAddress subnet(255,0,0,0);
  
  Ethernet.begin(config.mac, config.ip,  gateway, gateway, subnet);
  udp.begin(config.udpPort);

  // Open ArtNet
  node = ArtNode(config, sizeof(buffer), buffer);

  FastLED.addLeds<WS2812, 5, RGB>(leds1, NUM_RGB);
  FastLED.addLeds<WS2812, 20, RGB>(leds2, NUM_RGB);
  FastLED.addLeds<WS2812, 8, RGB>(leds3, NUM_RGB);
  FastLED.addLeds<WS2812, 14, RGB>(leds4, NUM_RGB);

  Serial.begin(9600);

  blink();
}

////////////////////////////////////////////////////////////
void loop() {

  if (udp.parsePacket()) {
    // First read the header to make sure it's Art-Net
    unsigned int n = udp.read(buffer, sizeof(ArtHeader));
    if (n >= sizeof(ArtHeader)) {
      
      ArtHeader* header = (ArtHeader*)buffer;
      // Check packet ID
      if (memcmp(header->ID, "Art-Net", 8) == 0) {

        // Read the rest of the packet
        udp.read(buffer+sizeof(ArtHeader), udp.available());

        // Package Op-Code determines type of packet
        switch(header->OpCode) {

          // Poll packet. Send poll reply.
          case OpPoll:
            node.createPollReply();
            artnetSend(buffer, sizeof(ArtPollReply));
            break;

          // DMX packet
          case OpDmx: {
            ArtDmx* dmx = (ArtDmx*)buffer;
            int port = node.getPort(dmx->SubUni, dmx->Net);
            if(!locateMode){
              // Bit swap length
              int l = ((dmx->Length & 0xF) << 8) + ((dmx->Length & 0xF0) >> 8);
              
              switch (port) {
                case 0:
                  memcpy(leds1, dmx->Data, l);
                  FastLED[0].show(leds1, NUM_RGB, 255);                
                  break;
                case 1:
                  memcpy(leds2, dmx->Data, l);
                  FastLED[1].show(leds2, NUM_RGB, 255);                
                  break;
                case 2:
                  memcpy(leds3, dmx->Data, l);
                  FastLED[2].show(leds3, NUM_RGB, 255);                

//                  memcpy(leds1+256, dmx->Data, l);
  //                FastLED[0].show(leds1, NUM_RGB, 255);                
                  break;
                case 3:
                  memcpy(leds4, dmx->Data, l);
                  FastLED[3].show(leds4, NUM_RGB, 255);           
                  break;
              }
            }
          }
          break;

          case OpAddress: {
            T_ArtAddress * address = (T_ArtAddress*)buffer;

            if(address->LongName[0] != 0)
              memcpy(config.longName, address->LongName, 64);
            if(address->ShortName[0] != 0)
              memcpy(config.shortName, address->ShortName, 18);

            if(address->NetSwitch != 0x7f)
              config.net = address->NetSwitch;

            if(address->SubSwitch != 0x7f)              
              config.subnet = address->SubSwitch;

            
            for(int i=0;i<4;i++){
              if(address->SwIn[i] != 0x7f)
                config.portAddrIn[i] = address->SwIn[i];
              if(address->SwOut[i] != 0x7f){
                config.portAddrOut[i] = address->SwOut[i];                
              }              
            }            

            if(address->Command == 0x04){
              locateMode = true;
            } else {
              locateMode = false;
            }
            
            
            node = ArtNode(config, sizeof(buffer), buffer);

            saveConfig();
            loadConfig();
            node.createPollReply();
            artnetSend(buffer, sizeof(ArtPollReply));

            
            break;
          }
          // IpProg packet
          /*case OpIpProg:
            node.createIpProgReply();
            artnetSend(buffer, sizeof(ArtIpProgReply));
            break;
*/
          // Unhandled packet
          default:
            break;
        }
      }
    }
  }

  if(locateMode){  
    byte* d = (byte*)leds1;    
    for(int i=0;i<NUM_RGBW;i++){
      d[i*4] = 0;
      d[i*4+1] = (sin(10+i + millis()/100.0)+1)*128.0;
      d[i*4+2] = 0;
      d[i*4+3] = (sin(i + millis()/100.0)+1)*128.0;
      
      
    }
    FastLED[0].show(leds1, NUM_RGB, 255);
    FastLED[1].show(leds1, NUM_RGB, 255);
    FastLED[2].show(leds1, NUM_RGB, 255);
    FastLED[3].show(leds1, NUM_RGB, 255);
  }
}

void blink(){
  byte* d = (byte*)leds1;
  
  for(int i=0;i<NUM_RGBW*4;i++){
    d[i] = 30;
  }
  FastLED[0].show(leds1, NUM_RGB, 255);

  delay(300);

  for(int i=0;i<NUM_RGBW*4;i++){
    d[i] = 0;
  }
  FastLED[0].show(leds1, NUM_RGB, 255);
  delay(100);
}


////////////////////////////////////////////////////////////
void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_MEM_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_MEM_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_MEM_START + 2) == CONFIG_VERSION[2]){
    for (unsigned int t=CONFIG_START; t<sizeof(config)-CONFIG_END; t++){
      *((char*)&config + t ) = EEPROM.read(CONFIG_MEM_START + t + 3 - CONFIG_START);
    }
  }
}

////////////////////////////////////////////////////////////
void saveConfig() {
  EEPROM.write(CONFIG_MEM_START + 0, CONFIG_VERSION[0]);
  EEPROM.write(CONFIG_MEM_START + 1, CONFIG_VERSION[1]);
  EEPROM.write(CONFIG_MEM_START + 2, CONFIG_VERSION[2]);
  for (unsigned int t=CONFIG_START; t<sizeof(config)-CONFIG_END; t++){
    EEPROM.write(CONFIG_MEM_START + t - CONFIG_START + 3, *((char*)&config + t));
  }
}

////////////////////////////////////////////////////////////
void artnetSend(byte* buffer, int length) {
  udp.beginPacket(node.broadcastIP(), config.udpPort);
  //udp.beginPacket(udp.remoteIP(), config.udpPort);
  udp.write(buffer, length);
  udp.endPacket();
}

