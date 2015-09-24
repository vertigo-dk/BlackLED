/** Retrieve Ethernet MAC from Teensy 3 */
class mac_addr : public Printable {
public:
    uint8_t m[6];

    mac_addr() : m({0}) {
        // Retrieve the 6 byte MAC address Paul burnt into two 32 bit words
        // at the end of the "READ ONCE" area of the flash controller.
        read(0xe,0);
        read(0xf,3);
    }

    void read(uint8_t word, uint8_t loc) {

        // To understand what's going on here, see
        // "Kinetis Peripheral Module Quick Reference" page 85 and
        // "K20 Sub-Family Reference Manual" page 548.

        FTFL_FCCOB0 = 0x41;             // Selects the READONCE command
        FTFL_FCCOB1 = word;             // read the given word of read once area
                                        // -- this is one half of the mac addr.
        FTFL_FSTAT = FTFL_FSTAT_CCIF;   // Launch command sequence
        while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF)) {
                                        // Wait for command completion
        }
        *(m+loc) =   FTFL_FCCOB5;       // collect only the top three bytes,
        *(m+loc+1) = FTFL_FCCOB6;       // in the right orientation (big endian).
        *(m+loc+2) = FTFL_FCCOB7;       // Skip FTFL_FCCOB4 as it's always 0.
    }

    virtual size_t printTo(Print & p) const {
        size_t count = 0;
        for(uint8_t i = 0; i < 6; ++i) {
            if (i!=0) count += p.print(":");
            count += p.print((*(m+i) & 0xF0) >> 4, 16);
            count += p.print(*(m+i) & 0x0F, 16);
        }
        return count;
    }
};
