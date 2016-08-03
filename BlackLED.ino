#include "FastLED.h"
#include <SPI.h>
#include <Ethernet.h>
#include <ArtNode.h>

#include "TeensyMAC.h"
#include <EEPROM.h>

#define VERSION_HI 0
#define VERSION_LO 4

// Set number of pixels (RGB) per output
#define NUM_PIXELS_OUT_1 288
#define NUM_PIXELS_OUT_2 1
#define NUM_PIXELS_OUT_3 1
#define NUM_PIXELS_OUT_4 1

#define PIN_RESET 9
#define PIN_DEBUG 2
#define PIN_LED_1 5
#define PIN_LED_2 20
#define PIN_LED_3 8
#define PIN_LED_4 14

// ID of the settings block
#define CONFIG_VERSION "ls1"
#define COLOR_CONFIG_VERSION "ls1"

// Tell it where to store your config data in EEPROM
#define CONFIG_MEM_START 16
#define CONFIG_START 17
#define CONFIG_END 2

#define COLOR_CONFIG_MEM_START (CONFIG_MEM_START+sizeof(ArtConfig)-CONFIG_START-CONFIG_END+3)



EthernetUDP udp;
byte udp_buffer[600];

CRGB led_data[683]; // (512*4)/3 = 682.6666

byte * dmx_data_ptr_1 = (byte*)led_data;
byte * dmx_data_ptr_2 = (byte*)led_data + 512;
byte * dmx_data_ptr_3 = (byte*)led_data + 512 * 2;
byte * dmx_data_ptr_4 = (byte*)led_data + 512 * 3;

bool locateMode = false;

int oemCode = 0x0000; // OemUnkown
////////////////////////////////////////////////////////////

#define OpColorCorrection 0x6100

typedef struct S_ColorConfig {
  uint8_t  brightness;
  uint8_t  red;
  uint8_t  green;
  uint8_t  blue;

} T_ColorConfig;

typedef struct S_ArtColorCorrection {
  uchar ID[8];                    // protocol ID = "Art-Net"
  ushort OpCode;                  // == OpPoll
  uchar ProtVerHi;                // 0
  uchar ProtVerLo;                // protocol version, set to ProtocolVersion

  uchar Brightness;               // Set brigthness scaleing
  uchar Red;                      // set red color scaleing
  uchar Green;                    // set green color scaleing
  uchar Blue;                     // set blue color scaleing

} T_ArtColorCorrection;

////////////////////////////////////////////////////////////

T_ColorConfig colorConfig = {
  255,  // Brghtness 100%
  255,  // Red 100 %
  204,  // Green 80%
  102,  // Blue 40%
};

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
  VERSION_LO,

};

ArtNode node;
////////////////////////////////////////////////////////////



void blink() {
  byte* d = (byte*)led_data;

  for (int i = 0; i < sizeof(led_data); i++) {
    d[i] = 30;
  }
  FastLED[0].show(led_data, NUM_PIXELS_OUT_1, 255);
  FastLED[1].show(led_data, NUM_PIXELS_OUT_2, 255);
  FastLED[2].show(led_data, NUM_PIXELS_OUT_3, 255);
  FastLED[3].show(led_data, NUM_PIXELS_OUT_4, 255);

  delay(300);

  for (int i = 0; i < sizeof(led_data); i++) {
    d[i] = 0;
  }
  FastLED[0].show(led_data, NUM_PIXELS_OUT_1, colorConfig.brightness);
  FastLED[1].show(led_data, NUM_PIXELS_OUT_2, colorConfig.brightness);
  FastLED[2].show(led_data, NUM_PIXELS_OUT_3, colorConfig.brightness);
  FastLED[3].show(led_data, NUM_PIXELS_OUT_4, colorConfig.brightness);
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

void artnetSend(byte* buffer, int length) {
  udp.beginPacket(node.broadcastIP(), config.udpPort);
  udp.write(buffer, length);
  udp.endPacket();
}

////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////


void setup() {
  loadConfig();
  loadColorConfig();

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

  // Create 4 outputs with different lengths (can be tweaked)
  FastLED.addLeds<WS2813, PIN_LED_1, GRB>(led_data, NUM_PIXELS_OUT_1);
  FastLED.addLeds<WS2813, PIN_LED_2, GRB>(led_data, NUM_PIXELS_OUT_2);
  FastLED.addLeds<WS2813, PIN_LED_3, GRB>(led_data, NUM_PIXELS_OUT_3);
  FastLED.addLeds<WS2813, PIN_LED_4, GRB>(led_data, NUM_PIXELS_OUT_4);
  FastLED.setDither(0);

  Serial.begin(9600);

  blink();
  FastLED.setCorrection(  (colorConfig.red << 16) + (colorConfig.green << 8) + colorConfig.blue  );
}

////////////////////////////////////////////////////////////
void loop() {

  while (udp.parsePacket()) {
    //  digitalWrite(PIN_DEBUG, HIGH);

    // First read the header to make sure it's Art-Net
    unsigned int n = udp.read(udp_buffer, sizeof(ArtHeader));
    if (n >= sizeof(ArtHeader)) {

      ArtHeader* header = (ArtHeader*)udp_buffer;
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
              if (!locateMode && port >= 0 && port < 4) {
                // Calculate length of DMX packet. Requires bit swap length from dmx packet
                int dmx_length = ((dmx->Length & 0xF) << 8) + ((dmx->Length & 0xF0) >> 8);

                // Check if length is longer then allocated data
                if (dmx_length > 512) {
                  dmx_length = 512;
                }

                switch (port) {
                  case 0: {
                      // Copy dmx data to the first 1/4 of led_data
                      memcpy(dmx_data_ptr_1, dmx->Data, dmx_length);
                      break;
                    }
                  case 1: {
                      memcpy(dmx_data_ptr_2, dmx->Data, dmx_length);
                      break;
                    }
                  case 2: {
                      memcpy(dmx_data_ptr_3, dmx->Data, dmx_length);
                      break;
                    }
                  case 3: {
                      memcpy(dmx_data_ptr_4, dmx->Data, dmx_length);
                      break;
                    }
                }
              }
            }
            break;
          case 0x5200: { //OpSync
              FastLED[0].show((CRGB*)dmx_data_ptr_1, NUM_PIXELS_OUT_1, colorConfig.brightness);
              FastLED[1].show((CRGB*)dmx_data_ptr_2, NUM_PIXELS_OUT_2, colorConfig.brightness);
              FastLED[2].show((CRGB*)dmx_data_ptr_3, NUM_PIXELS_OUT_3, colorConfig.brightness);
              FastLED[3].show((CRGB*)dmx_data_ptr_4, NUM_PIXELS_OUT_4, colorConfig.brightness);
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
          case OpColorCorrection: {
              T_ArtColorCorrection * colorCorrection = (T_ArtColorCorrection*)udp_buffer;

              colorConfig.brightness = colorCorrection->Brightness;
              colorConfig.red = colorCorrection->Red;
              colorConfig.green = colorCorrection->Green;
              colorConfig.blue = colorCorrection->Blue;

              FastLED.setCorrection( (colorConfig.red << 16) + (colorConfig.green << 8) + colorConfig.blue );

              saveColorConfig();
              loadColorConfig();
              
              break;
            }
          // IpProg packet
          /*case OpIpProg:
            node.createIpProgReply();
            artnetSend(udp_buffer, sizeof(ArtIpProgReply));
            break;
          */
          // Unhandled packet
          default:
            break;
        }
      }
    }
    //  digitalWrite(PIN_DEBUG, LOW);
  }
}

