#ifndef TESTER_H
#define TESTER_H

#include <Arduino.h>

class tester {
    public:
        //Constructor
        tester();

    //Test functions
    // Validate a 32-byte packet against the specified structure.
    // Returns true if valid, false otherwise.
    bool protocolTest(const uint8_t* inComingPacket, size_t packetLength);
};

#endif