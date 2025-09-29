#ifndef TESTER_H
#define TESTER_H

#include <Arduino.h>

class tester {
    public:
        //Constructor
        tester();

    //Test functions
    bool protocolTest(const uint8_t* inComingPacket, size_t packetLength);

    bool crc16CCITT(const uint8_t* data, size_t length);

};

#endif