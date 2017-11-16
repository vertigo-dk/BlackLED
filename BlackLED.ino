/////////////////////////////////////////////////////////////////////////////////////
//
// includes and lib
//
/////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <Ethernet.h>
#include <ArtNode.h>
#include "ArtNetFrameExtension.h"

#include <SoftPWM.h>

#include <TeensyMAC.h>
#include <EEPROM.h>

/////////////////////////////////////////////////////////////////////////////////////
//
// settings error check
//
/////////////////////////////////////////////////////////////////////////////////////

#if F_BUS < 60000000
#error "Teensy needs to run at 120MHz to read all packets in time"
#endif
#if defined(SPI_CLOCK_30MHz)
  #warning "has 30MHz SPi Clock"
#else
  #warning "has 30MHz SPi Clock"
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// real code starts hear
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define VERSION_HI 0
#define VERSION_LO 9

#if defined(__MK66FX1M0__)
#define PIN_RESET 24
#else
#define PIN_RESET 9
#endif

const int PIN_DIM = 27;
uint8_t CH = 0;

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
  "1ch_dim-------0",                           // Short name
  "1ch_dim-------0",                     // Long name
  1, // Number of ports
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
  SoftPWMSet(PIN_DIM, 255);
  delay(500);
  SoftPWMSet(PIN_DIM, 0);
  delay(5000);
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

#define MAC_CONFIG 1000

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// setup
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  uint8_t mac[6];
  teensyMAC(mac);
  if (mac[5] != 0) {
    for (uint8_t i = 0; i < 6; ++i) {
    EEPROM.write(MAC_CONFIG + i, mac[i]);
    }
  }else{
    for (uint8_t i = 0; i < 6; ++i) {
      mac[i] = EEPROM.read(MAC_CONFIG + i);
    }
  }

  for (uint8_t i = 0; i < 6; ++i) {
   config.mac[i] = EEPROM.read(MAC_CONFIG + i);
  }

  //saveConfig(); //<-- uncomment to force the EEPROM config to your settings on eatch reboot
  ArtConfig tempConfig = config;  // save the Firmeware state
  loadConfig();
  config.numPorts = tempConfig.numPorts;
  config.numPorts = tempConfig.numPorts;
  config.verHi = tempConfig.verHi;
  config.verLo = tempConfig.verLo;

  saveConfig();


#ifdef PIN_RESET
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_RESET, HIGH);
  delay(150);
#endif

  delay(200);

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


  CH = config.shortName[14] - 49;
  SoftPWMBegin();

  blink();
  SoftPWMSet(PIN_DIM, 0);
  // to read internal temperature
  analogReference(INTERNAL);
  analogReadResolution(12);

}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
// main loop
//
////////////////////////////////////////////////////////////////////////////////////////////////////

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
              //T_ArtPoll* poll = (T_ArtPoll*)udp_buffer;
              //if(poll->TalkToMe & 0x2){

              #ifdef blackOnOpPollTimeOut
                lastPoll = millis();
              #endif

              float tempCelsius = 25.0 + 0.17083 * (2454.19 - tempVal);
              sprintf(node.pollReport, "numOuts;%d;numUniPOut;%d;temp;%.1f;fps;%.1f;uUniPF;%.1f;", 1, 1, tempCelsius, fps, avgUniUpdated);
              node.createPollReply(); //create pollReply
              artnetSend(udp_buffer, sizeof(ArtPollReply)); //send pollReply

              break;
            }//OpPoll
          case OpDmx: {
              ArtDmx* dmx = (ArtDmx*)udp_buffer;
              int port = node.getAddress(dmx->SubUni, dmx->Net) - node.getStartAddress();
              if (port >= 0 && port < config.numPorts) {
                SoftPWMSet(PIN_DIM, map(dmx->Data[CH],0,255,150,0));
                numUniUpdated++;
              }
              break;
            }//OpDMX
          case OpSync: {

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
            }//OpSync
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
              CH = config.shortName[14] - 49;
              node = ArtNodeExtended(config, sizeof(udp_buffer), udp_buffer);
              saveConfig();
              loadConfig();
              node.createPollReply();
              artnetSend(udp_buffer, sizeof(ArtPollReply));
              node.createExtendedPollReply();
              artnetSend(udp_buffer, node.sizeOfExtendedPollReply());
              break;
          }

          // Unhandled packet
          default: {
              break;
            }
        }//switch
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
      }
    }
  }

  // read temperature value
  #if defined(__MK66FX1M0__)
  tempVal = analogRead(70) * 0.01 + tempVal * 0.99;
  #else
  tempVal = analogRead(38) * 0.01 + tempVal * 0.99;
  #endif

}
