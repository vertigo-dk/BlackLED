//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// initial user defined settings
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//#define MAX_NUM_LED_PER_OUTPUT 360
#define NUM_CHANNEL_PER_LED 4 // do not change this

//#define blackOnOpSyncTimeOut
//#define blackOnOpPollTimeOut

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MAX settings
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MAX_NUM_ARTNET_PORTS 18
#define MAX_NUM_LED_PER_OUTPUT 384 //for calculating the max buffer size needs shuld allways be
#define MAX_NUM_OF_OUTPUTS 8

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// default settings
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t OpSyncTimeOut = 300000;//recoment more than 20000 ms
uint32_t OpPollTimeOut = 30000;//recoment more than 20000 ms

uint16_t num_outputs = 6;
uint16_t num_led_per_output = 384; //512/NUM_CHANNEL_PER_LED*num_universes_per_output
uint16_t num_artnet_ports = 18;
uint16_t num_universes_per_output = 3;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// includes
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <Ethernet.h>
#include <ArtNode.h>
#include "ArtNetFrameExtension.h"

#include <OctoWS2811.h>

#include "TeensyMAC.h"
#include <EEPROM.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// settings error check
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

#define VERSION_HI 0
#define VERSION_LO 10

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
// octoWS2811
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DMAMEM uint32_t displayMemory[MAX_NUM_LED_PER_OUTPUT*8];
uint32_t drawingMemory[MAX_NUM_LED_PER_OUTPUT*8];

uint8_t LEDconfig = WS2811_RGBW | SK6812_820kHz;

OctoWS2811 LEDS(num_led_per_output, displayMemory, drawingMemory, LEDconfig);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Art-Net config
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ArtConfig_t config = {
//   {0xDE, 0xAD, 0xBE, 0x00, 0x00, 0x00}, // MAC - last 3 bytes set by Teensy
//   {2, 0, 0, 1},                         // IP
//   {255, 0, 0, 0},                       // Subnet mask
//   0x1936,                               // UDP port
//   false,                                // DHCP
//
//   // These fields get overwritten by loadConfig:
//   0, 0,                                 // Net (0-127) and subnet (0-15)
//   false, //extendedFeatures
//   0x00FF,//OEMcode
//   "BlackLED_6",                           // Short name
//   "BlackLED_6_port",                     // Long name
//   3, // Number of ports
//   { PortTypeDmx | PortDirOut,
//     PortTypeDmx | PortDirOut,
//     PortTypeDmx | PortDirOut,
//     PortTypeDmx | PortDirOut
//   }, // Port types
//   {0, 0, 0, 0},                         // Port input universes (0-15)
//   {0, 1, 2, 3},                          // Port output universes (0-15)
//   1,      //bindIndex
//   VERSION_HI,
//   VERSION_LO
// };

NodeConfig_t config = {
  {0xDE, 0xAD, 0xBE, 0x00, 0x00, 0x00}, // MAC - last 3 bytes set by Teensy
  {2, 0, 0, 1},                         // IP
  {255, 0, 0, 0},                       // Subnet mask
  0x1936,                               // UDP port
  false,                                // DHCP

  // These fields get overwritten by loadConfig:
  {0, 0, 0, 0, 0, 0, 0, 0},                   // Net (0-127)
  {0, 0, 0, 0, 0, 0, 0, 0},                   // and subnet (0-15)
  false, //extendedFeatures
  0x00FF,//OEMcode
  "BlackLED_6",                           // Short name
  "BlackLED_6_port",                     // Long name
  6, // Number of sub nodes
  3, // Number of ports in every sub node
  {0, 3, 6, 9, 12, 15, 0, 0},                          // Port output universes (0-15)
  VERSION_HI,
  VERSION_LO
};

// ArtNode node;
ArtNode node;

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
  for (int i = 0; i < 8 * num_led_per_output; i++) {
    LEDS.setPixel(i, 0xFFFFFFFF); //set full white
  }
  LEDS.show();
  delay(300);
  for (int i = 0; i <  8 * num_led_per_output; i++) {
    LEDS.setPixel(i, 0x00000000); //set 0
  }
  LEDS.show();
  delay(100);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// EEPROM setup
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ID of the settings block
#define CONFIG_VERSION "ls2"

// Tell it where to store your config data in EEPROM
#define CONFIG_MEM_START 16
#define CONFIG_START 0
#define CONFIG_END 0

int oemCode = 0x00FF; // OemUnkown

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
  EEPROM.update(CONFIG_MEM_START + 0, CONFIG_VERSION[0]);
  EEPROM.update(CONFIG_MEM_START + 1, CONFIG_VERSION[1]);
  EEPROM.update(CONFIG_MEM_START + 2, CONFIG_VERSION[2]);
  for (unsigned int t = CONFIG_START; t < sizeof(config) - CONFIG_END; t++) {
    EEPROM.update(CONFIG_MEM_START + t - CONFIG_START + 3, *((char*)&config + t));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// setup
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  //saveConfig(); //<-- uncomment to force the EEPROM config to your settings on eatch reboot
  // ArtConfig_t tempConfig = config;
  // loadConfig();
  // config.numPorts = tempConfig.numPorts;
  // config.numPorts = tempConfig.numPorts;
  // config.verHi = tempConfig.verHi;
  // config.verLo = tempConfig.verLo;
  // saveConfig();

  // Serial.begin(115200);

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
  config.ip[1] = config.mac[3] + oemCode;// + ((oemCode >> 16) & 0xFF);
  config.ip[2] = config.mac[4];
  config.ip[3] = config.mac[5];

  // Open Ethernet connection
  IPAddress gateway(config.ip[0], 0, 0, 1);
  IPAddress subnet(255, 0, 0, 0);

  Ethernet.begin(config.mac, config.ip,  gateway, gateway, subnet);
  udp.begin(config.udpPort);

  // Open ArtNet
  node = ArtNode(config, sizeof(udp_buffer), udp_buffer);

  //set up octoWS2811
  LEDS.setLength(num_led_per_output);
  LEDS.setType(LEDconfig);

  LEDS.begin();
  LEDS.show();

  blink();

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
  while (udp.parsePacket()) {
    // First read the header to make sure it's Art-Net
    unsigned int n = udp.read(udp_buffer, sizeof(ArtHeader_t));
    if (n >= sizeof(ArtHeader_t)) {
      ArtHeader_t* header = (ArtHeader_t*)udp_buffer;
      // Check packet ID
      if (memcmp(header->ID, "Art-Net", 8) == 0) {  //is Art-Net
        // Read the rest of the packet
        udp.read(udp_buffer + sizeof(ArtHeader_t), udp.available());
        // Package Op-Code determines type of packet
        switch (header->OpCode) {
          case OpPoll: {  // Poll packet
            float tempCelsius = 25.0 + 0.17083 * (2454.19 - tempVal);
            // sprintf(node.pollReport, "numOuts;%d;numUniPOut;%d;temp;%.1f;fps;%.1f;uUniPF;%.1f;", NUM_OF_OUTPUTS, num_universes_per_output, tempCelsius, fps, avgUniUpdated);
            for(int i = 0; i < config.numSubNodes; i++) {
              node.createPollReply(i); //create pollReply
              artnetSend(udp_buffer, sizeof(ArtPollReply_t)); //send pollReply
            }
            break;
          }
          case OpDmx: { // DMX packet
            ArtDmx_t* dmx = (ArtDmx_t*)udp_buffer;
            uint16_t len = __builtin_bswap16(dmx->Length);
            int address = node.getAddress(dmx->SubUni, dmx->Net);
            // Serial.print("address: ");
            // Serial.print(address);
            // Serial.print("\t");
            for (int i = 0; i < config.numSubNodes; i++) {
              if(address >= node.getStartAddress(i) && address <= node.getStartAddress(i)+config.numPorts) {
                int ledOffset = address - node.getStartAddress(i);
                ledOffset *= 512;
                ledOffset = ledOffset/NUM_CHANNEL_PER_LED;
                ledOffset += i * num_led_per_output;
                // Serial.print("ledOffset: ");
                // Serial.println(ledOffset);
                for(int j = 0; j < len; j+=4) {
                  LEDS.setPixel(ledOffset, dmx->Data[j], dmx->Data[j+1], dmx->Data[j+2], dmx->Data[j+3]);
                  ledOffset++;
                }
                break;
              }
            }
            // if (port >= 0 && port < config.numPorts) {
              // uint16_t portOffset = port/num_universes_per_output;
              //write the dmx data to the Octo frame buffer
              // uint32_t* dmxData = (uint32_t*) dmx->Data;
              // for (int i = 0; i < num_led_per_output; i+=) {
              //   LEDS.setPixel(i + port, dmxData[i]);
              // }
              numUniUpdated++;
            // }
            break;
          }
          case OpSync: {  // OpSync
            LEDS.show();
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
/*
            ArtAddress_t * address = (ArtAddress_t*)udp_buffer;

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
              delay(1000);
              CPU_RESTART

            }

            if (address->Command == 0x04) {
              locateMode = true;
            } else {
              locateMode = false;
            }
            saveConfig();
            loadConfig();
            node.createPollReply();
            artnetSend(udp_buffer, sizeof(ArtPollReply_t));*/
            break;

          }
          default: { // Unhandled packet
            break;
          }
        }
      }
    }
  }
}
