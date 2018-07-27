//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// initial user defined settings
//
#define BUILD 12
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define NUM_OF_OUTPUTS 6
#define MAX_NUM_LED_PER_OUTPUT 360
#define NUM_CHANNEL_PER_LED 4 // do not change this

//#define blackOnOpSyncTimeOut //recoment more than 20000 ms
//#define blackOnOpPollTimeOut //recoment more than 20000 ms
const uint32_t OpSyncTimeOut = 300000;
const uint32_t OpPollTimeOut = 30000;
const uint16_t internalFps = 1000; //in ms (1000ms = 44Hz)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// definitions calculated from user settings
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int num_channel_per_output = MAX_NUM_LED_PER_OUTPUT * NUM_CHANNEL_PER_LED;

const int num_universes_per_output = (num_channel_per_output%512) ? num_channel_per_output/512+1 : num_channel_per_output/512;

const int num_led_per_output = num_universes_per_output*512/NUM_CHANNEL_PER_LED;

const int num_artnet_ports = num_universes_per_output*NUM_OF_OUTPUTS;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// includes and lib
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <Ethernet.h>
#include <ArtNode.h>
#include "ArtNetFrameExtension.h"

#include <OctoWS2811.h>

#include <TeensyMAC.h>
#include <EEPROM.h>

#include <FirmwareFlasher.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// settings error check
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if F_BUS < 60000000
#error "Teensy needs to run at 120MHz to read all packets in time"
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// real code starts hear
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define VERSION_HI 0
#define VERSION_LO 10

#if defined(__MK66FX1M0__)
#define PIN_RESET 24
#else
#define PIN_RESET 9
#endif

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

bool firmware_update_in_progress = false;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// octoWS2811
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DMAMEM uint32_t displayMemory[num_led_per_output * 8];
uint32_t drawingMemory[num_led_per_output * 8];

const int LEDconfig = WS2811_RGBW | WS2811_800kHz;

OctoWS2811 LEDS(num_led_per_output, displayMemory, drawingMemory, LEDconfig);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Art-Net config
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ArtConfig config = {
  {0,0,0,0,0,0}, // MAC - last 3 bytes set by Teensy
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
void allWhite() {
  for (int i = 0; i < 8 * num_led_per_output; i++) {
    LEDS.setPixel(i, 0xFFFFFFFF); //set full white
  }
  LEDS.show();
}

void allBlack() {
  for (int i = 0; i <  8 * num_led_per_output; i++) {
    LEDS.setPixel(i, 0x00000000); //set 0
  }
  LEDS.show();
}

void locate() {
  uint8_t val = map(sin(millis()/100), -1.0, 1.0, 0x00, 0xFF);
  for (int i = 0; i < 8 * num_led_per_output; i++) {
    LEDS.setPixel(i, val, val, val, val);
  }
  LEDS.show();
}

void blink() {
  allWhite();
  delay(300);
  allBlack();
  delay(100);
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// setup
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

char line[200];

void setup() {
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
  // Read MAC address
  uint64_t mac_addr = teensyMAC();

  config.mac[0] = (mac_addr >> 8*5) & 0xFF;
  config.mac[1] = (mac_addr >> 8*4) & 0xFF;
  config.mac[2] = (mac_addr >> 8*3) & 0xFF;
  config.mac[3] = (mac_addr >> 8*2) & 0xFF;
  config.mac[4] = (mac_addr >> 8*1) & 0xFF;
  config.mac[5] = (mac_addr >> 8*0) & 0xFF;

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
    // Serial.println("udp");
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
              #ifdef blackOnOpPollTimeOut
                lastPoll = millis();
              #endif

              // read temperature value
              tempVal = analogRead(38) * 0.01 + tempVal * 0.99;
              float tempCelsius = 25.0 + 0.17083 * (2454.19 - tempVal);

              sprintf(node.pollReport, "numOuts;%d;numUniPOut;%d;temp;%.1f;fps;%.1f;uUniPF;%.1f;build;%d", NUM_OF_OUTPUTS, num_universes_per_output, tempCelsius, fps, avgUniUpdated, BUILD);
              node.createPollReply(); //create pollReply
              artnetSend(udp_buffer, sizeof(ArtPollReply)); //send pollReply
              break;
            }

          // DMX packet
          case OpDmx: {
              ArtDmx* dmx = (ArtDmx*)udp_buffer;
              int port = node.getAddress(dmx->SubUni, dmx->Net) - node.getStartAddress();
              if (port >= 0 && port < config.numPorts) {
                uint16_t portOffset = port * 512/NUM_CHANNEL_PER_LED;

                //write the dmx data to the Octo frame buffer
                uint32_t* dmxData = (uint32_t*) dmx->Data;
                for (int i = 0; i < 128; i++) {
                  LEDS.setPixel(i + portOffset, dmxData[i]);
                }
                numUniUpdated++;
              }
              break;
            }

          // OpSync
          case OpSync: {
              if (locateMode == false) {
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
              }

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
                allBlack();
                locateMode = false;
              }
              node = ArtNodeExtended(config, sizeof(udp_buffer), udp_buffer);
              saveConfig();
              loadConfig();
              node.createPollReply();
              artnetSend(udp_buffer, sizeof(ArtPollReply));
              break;
            }
          case OpFirmwareMaster: {
            if (firmware_update_in_progress == false) {
              int ret  = FirmwareFlasher.prepare_flash();
              if (ret == 0) {
                firmware_update_in_progress = true;
                udp.stop();
                udp.begin(8050);
              }else {
                delay(1000);
                CPU_RESTART;
              }
            }else {
              CPU_RESTART;
            }
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
      }
    }
  }
  if (locateMode == true) {
    locate();
  }
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

  if (firmware_update_in_progress == true) {
    udp.beginPacket(IPAddress(2,0,0,1), 8050);
    udp.write(10);
    udp.endPacket();

    for (int i = 0; i < 200000; i++) {
      while (udp.parsePacket()) {
        unsigned int n = udp.read(udp_buffer, 15);
        if (n >= 15) {
          if (memcmp(udp_buffer, "firmware_line__", 15) == 0) {
            unsigned int m = udp.read(udp_buffer, 400);
            // Serial.printf("length_%d\n", m);
            for (int i = 0; i < m; i++) {
              if (udp_buffer[i] == '\n') {
                line[i] = 0;
                if (FirmwareFlasher.flash_hex_line(line) != 0) {
                }else {
                  udp.beginPacket(udp.remoteIP(), udp.remotePort());
                  udp.write(10);
                  udp.endPacket();
                }
              }else {
                line[i] = udp_buffer[i];
              }
            }
          }
        }
      }
      delay(1);
    }
    CPU_RESTART;
  }
}
