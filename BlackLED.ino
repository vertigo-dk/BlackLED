//#define USE_OCTOWS2811
//#include<OctoWS2811.h>

//initial user defined settings
#define numRealPortOut 6
#define maxNumPixelsPerOut 360
#define numColorPerLED 4

// constants derived from user defined
#define numDMXchannels maxNumPixelsPerOut * numRealPortOut * numColorPerLED
#if maxNumPixelsPerOut % 512 != 0
  #define DMXadjustedPixelcout (((maxNumPixelsPerOut*numColorPerLED)/512+1)*512)/numColorPerLED
#else
  #define DMXadjustedPixelcout (((maxNumPixelsPerOut*numColorPerLED)/512)*512)/numColorPerLED
#endif
#define numArtNetPort (numRealPortOut * DMXadjustedPixelcout * numColorPerLED)/512

#include <SPI.h>
#include <Ethernet.h>
#include <ArtNode.h>
#include "ArtNetFrameExtension.h"
#include "OctoWS2811.h"


#if F_BUS < 60000000
#error "Teensy needs to run at 120MHz to read all packets in time"
#endif

#include "TeensyMAC.h"
#include <EEPROM.h>

#define VERSION_HI 0
#define VERSION_LO 4

#define PIN_RESET 9

EthernetUDP udp;
uint8_t udp_buffer[600];

boolean locateMode = 0;
uint32_t portSyncFlag;
uint32_t portSyncFlagCheck = 0;
uint8_t syncFlag;

unsigned long currentMillis;
unsigned long previousMillis = 0;
uint8_t dataFps;
uint8_t syncFps;

uint8_t dataFpsMin;
uint8_t syncFpsMin;
//////////////////////////////////// OCTO setup ///////////////////////
#define NUM_PIXELS_PR_STRIP 384
uint32_t dmxMemory[NUM_PIXELS_PR_STRIP*8];
DMAMEM uint32_t displayMemory[NUM_PIXELS_PR_STRIP*8];
uint32_t drawingMemory[NUM_PIXELS_PR_STRIP*8];

const int LEDconfig = WS2811_GRBW | WS2811_800kHz;

OctoWS2811 LEDS(NUM_PIXELS_PR_STRIP, displayMemory, drawingMemory, LEDconfig);

////////////////////////////////////////// Art-Net config //////////////////
ArtConfig config = {
  {0xDE, 0xAD, 0xBE, 0x00, 0x00, 0x00}, // MAC - last 3 bytes set by Teensy
  {2, 0, 0, 1},                         // IP
  {255, 0, 0, 0},                       // Subnet mask
  0x1936,                               // UDP port
  false,                                // DHCP

  // These fields get overwritten by loadConfig:
  0, 0,                                 // Net (0-127) and subnet (0-15)
  "BlackLED6",                           // Short name
  "BlackLED 6 port",                     // Long name
  18,                         // Number of ports
  {PortTypeDmx | PortTypeOutput,
  PortTypeDmx | PortTypeOutput,
  PortTypeDmx | PortTypeOutput,
  PortTypeDmx | PortTypeOutput}, // Port types
  {0, 0, 0, 0},                         // Port input universes (0-15)
  {0, 1, 2, 3},                          // Port output universes (0-15)
  VERSION_HI,
  VERSION_LO
};
ArtNodeExtended node;

//------------------------------------------ udp send ---------------------
void artnetSend(byte* buffer, int length) {
  udp.beginPacket(node.broadcastIP(), config.udpPort);
  udp.write(buffer, length);
  udp.endPacket();
}

//////////////////////////////////////// Blink test ///////////////////////////////
void blink() {
  for (int i = 0; i < 8 * NUM_PIXELS_PR_STRIP; i++) {
    LEDS.setPixel(i,0xFFFFFFFF);  //set full white
  }
  LEDS.show();
  delay(300);
  for (int i = 0; i <  8 * NUM_PIXELS_PR_STRIP; i++) {
    LEDS.setPixel(i,0x00000000);  //set 0
  }
  LEDS.show();
  delay(100);
}

////////////////////////////////////// EEPROM setup /////////////////////////////
// ID of the settings block
#define CONFIG_VERSION "ls1"

// Tell it where to store your config data in EEPROM
#define CONFIG_MEM_START 16
#define CONFIG_START 17
#define CONFIG_END 2



int oemCode = 0x0000; // OemUnkown

//------------------------------------------ load eeprom ---------------------
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

//------------------------------------------ save eeprom ---------------------
void saveConfig() {
  EEPROM.write(CONFIG_MEM_START + 0, CONFIG_VERSION[0]);
  EEPROM.write(CONFIG_MEM_START + 1, CONFIG_VERSION[1]);
  EEPROM.write(CONFIG_MEM_START + 2, CONFIG_VERSION[2]);
  for (unsigned int t = CONFIG_START; t < sizeof(config) - CONFIG_END; t++) {
    EEPROM.write(CONFIG_MEM_START + t - CONFIG_START + 3, *((char*)&config + t));
  }
}

//////////////////////////////////////////////////////////////////////////////////

void setup() {
  saveConfig();
  loadConfig();
  for (int i = 0; i < config.numPorts; i++) {
    bitSet(portSyncFlagCheck, i);
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
  IPAddress gateway(config.ip[0], 0, 0, 1);
  IPAddress subnet(255, 0, 0, 0);

  Ethernet.begin(config.mac, config.ip,  gateway, gateway, subnet);
  udp.begin(config.udpPort);

  // Open ArtNet
  node = ArtNodeExtended(config, sizeof(udp_buffer), udp_buffer);

  LEDS.begin();
  LEDS.show();

  blink();
}
////////////////////////////////////////////////////////////
void loop() {
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
              node.createPollReply(); //create pollReply
              artnetSend(udp_buffer, sizeof(ArtPollReply)); //send pollReply
              break;}
            // DMX packet
            case OpDmx: {
              ArtDmx* dmx = (ArtDmx*)udp_buffer;
              int port = node.getAddress(dmx->SubUni, dmx->Net) - node.getStartAddress();
              if (port >= 0 && port < config.numPorts) {
                bitSet(portSyncFlag, port);
                uint16_t portOffset = port*128;
                //write the dmx data to the Octo frame buffer
                uint32_t* dmxData = (uint32_t*) dmx->Data;
                for (int i = 0; i < 128; i++) {
                  LEDS.dmxPixel(i+portOffset, dmxData[i]);
                }
              }
              break;}
            // OpSync
            case 0x5200: {
              LEDS.show();
              syncFps++;
              if (portSyncFlag == portSyncFlagCheck) { //check if all ports have been updated
                syncFlag++;
                portSyncFlag = 0;
              }
              currentMillis = millis();
              if (currentMillis-previousMillis >= 995) {
                dataFps = syncFlag;
                syncFlag = 0;
                syncFps = 0;
                previousMillis = currentMillis;
              }
              break;}
            case OpAddress: {
              T_ArtAddress * address = (T_ArtAddress*)udp_buffer;
              if (address->LongName[0] != 0) {
                memcpy(config.longName, address->LongName, 64);
              }
              if (address->ShortName[0] != 0) {
                memcpy(config.shortName, address->ShortName, 18);
              }
              if (address->NetSwitch != 0x7f) {               // Use value 0x7f for no change.
                if (bitRead(address->NetSwitch,7) == true) { // This value is ignored unless bit 7 is high. i.e. to program a  value 0x07, send the value as 0x87.
                  config.net = address->NetSwitch && 0x7F;
                }
              }
              if (address->SubSwitch != 0x7f) {               // Use value 0x7f for no change.
                if (bitRead(address->SubSwitch,7) == true) { // This value is ignored unless bit 7 is high. i.e. to program a  value 0x07, send the value as 0x87.
                  config.subnet = address->SubSwitch && 0x7F;
                }
              }
              for (int i = 0; i < 4; i++) {
                if (address->SwIn[i] != 0x7f) {
                  config.portAddrIn[i] = address->SwIn[i];
                }
                if (address->SwOut[i] != 0x7f) {
                  config.portAddrOut[i] = address->SwOut[i];
                }
              }
            /*  for (int i = 1; i < config.numPorts; i++) { //for now we only do sequential port addr.
                config.portAddrOut[i] = config.portAddrOut[0]+i;
              }*/
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
              node.createExtendedPollReply();
              artnetSend(udp_buffer, node.sizeOfExtendedPollReply());
              break;}
            // IpProg packet

            // Unhandled packet
            default:{
              break;}
          }
        }else if(memcmp(header->ID, "Art-Ext", 8) == 0) {
          // Read the rest of the packet
          udp.read(udp_buffer + sizeof(ArtHeader), udp.available());
          // Package Op-Code determines type of packet
          switch (header->OpCode) {
          // ArtNet Frame Extension
          case OpPoll | 0x0001: {
            node.createExtendedPollReply();
            artnetSend(udp_buffer, node.sizeOfExtendedPollReply());
            break;}
          }
       }
    }
  }
}
