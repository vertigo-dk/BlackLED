#include "FastLED.h"
#include <SPI.h>
#include <Ethernet.h>
#include <ArtNode.h>

#define NUM_RGBW  144
#define NUM_RGB   (NUM_RGBW/4*3)
#define PIN_RESET 9

EthernetUDP udp;
byte buffer[600];

CRGB leds1[NUM_RGB];
CRGB leds2[NUM_RGB];

////////////////////////////////////////////////////////////
ArtConfig config = {
  {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}, // MAC
  {172, 16, 0, 111},                    // IP
  {255, 255, 255, 0},                   // Subnet mask
  0x1936,                               // UDP port
  false,                                // DHCP
  0, 0,                                 // Net (0-127) and subnet (0-15)
  "VardeLED",                           // Short name
  "VardeLED",                           // Long name
  4,                                    // Number of ports
  {PortTypeDmx|PortTypeOutput, PortTypeDmx|PortTypeOutput, PortTypeDmx|PortTypeOutput, PortTypeDmx|PortTypeOutput}, // Port types
  {0, 1, 2, 3},                         // Port input universes (0-15)
  {0, 1, 2, 3}                          // Port output universes (0-15)
};
ArtNode node = ArtNode(config, sizeof(buffer), buffer);

////////////////////////////////////////////////////////////
void setup() {

#ifdef PIN_RESET
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_RESET, HIGH);
  delay(150);
#endif

  Ethernet.begin(config.mac, config.ip);
  udp.begin(config.udpPort);

  FastLED.addLeds<WS2812, 5, RGB>(leds1, NUM_RGB);
  FastLED.addLeds<WS2812, 21, RGB>(leds2, NUM_RGB);
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
            switch (port) {
              case 0:
                mergeRGBtoRGBW(dmx->Data, (byte*)leds1, NUM_RGBW);
                FastLED[0].show(leds1, NUM_RGB, 255);
                break;
              case 1:
                mergeWtoRGBW(dmx->Data, (byte*)leds1, NUM_RGBW);
                FastLED[0].show(leds1, NUM_RGB, 255);
                break;
              case 2:
                mergeRGBtoRGBW(dmx->Data, (byte*)leds2, NUM_RGBW);
                FastLED[1].show(leds2, NUM_RGB, 255);
                break;
              case 3:
                mergeWtoRGBW(dmx->Data, (byte*)leds2, NUM_RGBW);
                FastLED[1].show(leds2, NUM_RGB, 255);
                break;
            }
          }
          break;

          // IpProg packet
          case OpIpProg:
            node.createIpProgReply();
            artnetSend(buffer, sizeof(ArtIpProgReply));
            break;

          // Unhandled packet
          default:
            break;
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////
void artnetSend(byte* buffer, int length) {
  udp.beginPacket(node.broadcastIP(), config.udpPort);
  //udp.beginPacket(udp.remoteIP(), config.udpPort);
  udp.write(buffer, length);
  udp.endPacket();
}

////////////////////////////////////////////////////////////
void mergeRGBtoRGBW(uint8_t* rgb, uint8_t* rgbw, int n) {
  for (int i=0; i<n; i++) {
    rgbw[1] = rgb[2];
    rgbw[0] = rgb[0];
    rgbw[2] = rgb[1];
    rgb += 3;
    rgbw += 4;
  }
}

////////////////////////////////////////////////////////////
void mergeWtoRGBW(uint8_t* w, uint8_t* rgbw, int n) {
  for (int i=0; i<n; i++) {
    rgbw[3] = w[0];
    w ++;
    rgbw += 4;
  }
}

