/**
* ArtNet Frame Extenstion understood by MadMapper
**/

#include "ArtNode.h"

typedef struct S_Ext_ArtPoll {
    uchar ID[8];                    // protocol ID = "Art-Ext"
    ushort OpCode;                  // == OpPoll + 1 0x2001
    uchar ProtVerHi;                // 0
    uchar ProtVerLo;                // protocol version, set to ProtocolVersion
} T_Ext_ArtPoll;

typedef struct S_Ext_ArtPollReply {
  uchar ID[8];                    // protocol ID = "Art-Ext"
  ushort OpCode;                  // == OpPollReply + 1 
  
  uchar ProtocolVersionHi;        // The protocol version of ART-EXT (1 currently)
  uchar ProtocolVersionLo;        // The protocol version of ART-EXT (1 currently)
  
  uchar BoxAddr[4];                 // 0 if not yet configured
  uchar Mac[6];                   // Mac Address, zero if info not available

  uchar ResponseNum;              // This is packet x of y
  uchar TotalResponses;           // There are y packets

  uchar NumPortsIn;
  uchar NumPortsOut;

  //short swinout; 
  

} T_Ext_ArtPollReply;



class ArtNodeExtended : public ArtNode {
public:
  ArtNodeExtended();
  ArtNodeExtended(ArtConfig & config, int size, unsigned char * buffer);

  void createPollReply();
  void createExtendedPollReply();
  int sizeOfExtendedPollReply();

  
  uint16_t getAddress(uint8_t subUni, uint8_t net);
  uint16_t getStartAddress();

  char pollReport[64];
};

