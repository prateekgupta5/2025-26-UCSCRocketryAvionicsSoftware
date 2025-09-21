#include "tester.h"

//Constructor
tester::tester() {}

//Test functions
/*
    Fucntion to check the validity of a packet
    Protcol:
    Byte 0: Start byte
    Byte 1~3: Packet Sequence ID
    Byte 4~5: Message Type ID
    Byte 6~10 : Timestamp
    Byte 11~27: Payload
    Byte 28~29: Error Check
    Byte 30~31: End byte

    Functions:
        Check if the packet is 32 bytes long
        Check the range of each field of the packet

    Input:  
        Pointer to a packet
        Length to read
    Output:
        True if the packet is valid, false otherwise
*/
bool tester::protocolTest(const uint8_t* inComingPacket, size_t packetLength) {
    //print incoming packet
    Serial.println("Incoming Packet:");
    for (size_t i = 0; i < packetLength; i++) {
        Serial.print(inComingPacket[i], HEX);
        Serial.print(" ");
    }

    Serial.println();

    //check length
    if (packetLength != 32) {
        Serial.println("Invalid packet length");
        return false; // Invalid length
    }

    //TO-DO:Check range of each field

    //passed all checks
    Serial.println("Packet is valid");
    return true;
}