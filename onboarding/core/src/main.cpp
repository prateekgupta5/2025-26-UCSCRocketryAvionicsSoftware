#include <Arduino.h>
#include <iostream>
#include <cstring>
// #include <bit>
#include <bitset>
#include "tester.h"
//below is the library for the IMU, which is the BNO088
#include "Adafruit_BNO08x_RVC.h"

using namespace std;

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
const long baudrate = 115200;
float vy_i = 0;

#define PACKET_SIZE 32
#define    MSG_SIZE 17
union UART_Packet_t {
    UART_Packet_t () : end({0x0D, 0x0A}) {} 

    uint8_t bytes[PACKET_SIZE];
    struct {
        uint8_t  head;
        uint32_t seq ;
        uint16_t id  ;
        uint32_t timstmp;
        uint8_t  msg[MSG_SIZE];
        uint16_t crc;
        uint16_t end;
    }
}

float y_velocity (float vi, float dt, float ay)
{
    // To calculate the velocity, we will be using this simple physics equation: y_f = y_i + a(△t), assuming that acceleration (a) is constant for the short time period. 
    return vi + ay * dt;
}


// template <typename T>
// void writeToPacket(uint8_t* packet, int& offset, T data) {
//     //
// }


inline uint16_t calculateCRC(uint8_t* data, length = MSG_SIZE + 1) constexpr {
    //data should include crc so length will be longer on this side
    uint16_t crc = 0xFFFF; // Initial value for CRC-16-CCITT

    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8; //Bitshift left 1 byte

        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) { // If MSB is 1
                crc = (crc << 1) ^ 0x1021; // XOR with polynomial
            } else {
                crc <<= 1; // Shift left
            }
        }
    }

    return crc == 0;
}


void setup()
{
    Serial.begin(baudrate);
    while (!Serial)
    {
        delay(10);
    }
    Serial.println("Adafruit BNO08x IMU - UART-RVC mode");
    Serial1.begin(baudrate);
    while (!Serial1){delay(10);}
    if (!rvc.begin(&Serial1)) { // connect to the sensor over hardware serial
        Serial.println("Could not find BNO08x");
        while (1)
        delay(10);
    }
    Serial.println("Found BNO08x");
}

void loop()
    BNO08x_RVC_Data heading;
    if (!rvc.read(&heading)) {
        return;
    }

    //read IMU acceleration data and store in three variables 
    float x_acceleration = heading.x_accel;
    float y_acceleration = heading.y_accel;
    float z_acceleration = heading.z_accel;

    //store in struct

    float delta_time = 0.01;

    //call stateEstimation function
    vy_i = y_velocity(vy_i, delta_time, y_acceleration);

    // uint8_t start_byte = '!'; //stores the ascii value of ! into the variable
    // uint32_t sequence_id = 0x0000;
    // uint16_t id = 0x696D; //hex for "im", the id for IMU
    // uint32_t timestamp = 0x00000001;

    //create payload and prepend with 0s up until 17 bytes
    //
    
    UART_Packet_t packet;
    //put together our UART packet
    packet.head = '!';
    packet.seq = 0x0000;
    packet.id = 0x696D;
    packet.timstmp = 0x00000001;
    memcpy(packet.msg, vy_i, sizeof(float));
    packet.crc = calculateCRC(packet.msg);

    //testing the datapacket with tester class
    tester test;
    test.protocolTest(packet.bytes, PACKET_SIZE);
    delay(5000);
}