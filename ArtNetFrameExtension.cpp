
#include <stdio.h>
#include <stdlib.h> // malloc
#include <string.h> // memcpy
#include "ArtNetFrameExtension.h"


ArtNodeExtended::ArtNodeExtended() {
}

ArtNodeExtended::ArtNodeExtended(ArtConfig & config, int size, unsigned char * buffer) {
  this->config = &config;
  this->bufferSize = size;
  this->buffer = buffer;
}

int ArtNodeExtended::sizeOfExtendedPollReply() {
  return sizeof(T_Ext_ArtPollReply) + sizeof(short) * config->numPorts;
}

uint16_t ArtNodeExtended::getAddress(uint8_t subUni, uint8_t net) {
  return subUni + (net << 8);
}
uint16_t ArtNodeExtended::getStartAddress() {
  return config->portAddrOut[0] + (config->subnet << 4) + (config->net << 8);
}



void ArtNodeExtended::createExtendedPollReply() {
  T_Ext_ArtPollReply *reply = (T_Ext_ArtPollReply*)buffer;
  memset(buffer, 0, sizeof(T_Ext_ArtPollReply) + sizeof(uint16_t)*config->numPorts);

  memcpy(reply->ID, "Art-Ext", 8) ;
  reply->OpCode = OpPollReply | 0x0001;
  memcpy(reply->BoxAddr, config->ip, 4);

  memcpy(reply->Mac, config->mac, 6);

  reply->ProtocolVersionLo = 1;
  reply->ProtocolVersionHi = 0;


  reply->ResponseNum = 1;
  reply->TotalResponses = 1;

  reply->NumPortsIn = 0;
  reply->NumPortsOut = config->numPorts;

  uint16_t * addresses = (uint16_t*)(buffer + sizeof(T_Ext_ArtPollReply));
  for (int i = 0; i < config->numPorts; i++) {
    uint16_t addr = config->portAddrOut[0]+ i + (config->subnet << 4) + (config->net << 8);
//    uint16_t addr = config->portAddrOut[0]+ i + (config->subnet * 16) + (config->net * 256);
    addresses[i] = addr;
  }

  /*
   *
    reply->VersionInfoHi = config->verHi;
    reply->VersionInfoLo = config->verLo;

    reply->NetSwitch = config->net;
    reply->SubSwitch = config->subnet;

    strcpy((char*)reply->ShortName, config->shortName);
    strcpy((char*)reply->LongName, config->longName);

    reply->NumPortsLo = config->numPorts;
    memcpy(reply->PortTypes, config->portTypes, 4);
    memset(reply->GoodOutput, 0x80, config->numPorts); // Very important for MadMapper!
    memcpy(reply->SwIn, config->portAddrIn, 4);
    memcpy(reply->SwOut, config->portAddrOut, 4);
    reply->Style = StyleNode;
    */
}

void ArtNodeExtended::createPollReply() {
  ArtPollReply *reply = (ArtPollReply*)buffer;
  memset(buffer, 0, sizeof(ArtPollReply));

  memcpy(reply->ID, "Art-Net", 8);
  reply->OpCode = OpPollReply;
  memcpy(reply->BoxAddr.IP, config->ip, 4);
  reply->BoxAddr.Port = config->udpPort;

  reply->VersionInfoHi = config->verHi;
  reply->VersionInfoLo = config->verLo;

  reply->NetSwitch = config->net;
  reply->SubSwitch = config->subnet;


  strcpy((char*)reply->ShortName, config->shortName);
  strcpy((char*)reply->LongName, config->longName);
  
  strcpy((char*)reply->NodeReport, pollReport);
  

  int numPorts = config->numPorts;
  if (numPorts > 4) numPorts = 4;
  reply->NumPortsLo = numPorts;
  memcpy(reply->PortTypes, config->portTypes, 4);
  memset(reply->GoodOutput, 0x80, numPorts); // Very important for MadMapper!
  memcpy(reply->SwIn, config->portAddrIn, 4);
  memcpy(reply->SwOut, config->portAddrOut, 4);
  reply->Style = StyleNode;
  memcpy(reply->Mac, config->mac, 6);
}
