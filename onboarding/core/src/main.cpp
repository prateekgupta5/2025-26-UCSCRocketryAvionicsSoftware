#include <Arduino.h>
#include <iostream>
#include <cstring>
#include "tester.h"
//below is the library for the IMU, which is the BNO088
#include "Adafruit_BNO08x_RVC.h"

using namespace std;

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
const long baudrate = 115200;
float current_velocity_y = 0;
//this is the sensor's factury default baud rate for the RVC setting. 

//packet structure
// struct UART_Packet
// {
//     uint8_t start_byte;
//     uint32_t sequence_id;
//     uint16_t id;
//     uint32_t timestamp;
//     uint8_t payload[17]; 
//     uint16_t crc_16;
//     uint16_t end_byte;
// };

//this will hold values after state estimation is done
// struct IMU_Data
// {
//     float ax;
//     float ay;
//     float az;
// };

//relevent functions 

float y_velocity (float initial_velocity_y, float delta_time, float y_acceleration)
{
    // To calculate the velocity, we will be using this simple physics equation: y_f = y_i + a(△t), assuming that acceleration (a) is constant for the short time period. 
    float final_velocity = initial_velocity_y + (y_acceleration*delta_time);
    return final_velocity;
}

// UART_Packet createPacket(uint8_t start_byte, uint32_t sequence_id, uint16_t id, uint32_t timestamp, uint8_t (&payload)[17],  uint16_t crc_16, uint16_t end_byte)
// {
//     UART_Packet returnPacket = {start_byte, sequence_id, id, timestamp, payload, crc_16, end_byte};

//     return returnPacket;
// }

uint16_t calculateCRC(uint8_t* data, size_t length)
{
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

    return crc;
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
    while (!Serial1)
    {
        delay(10);
    }
    //initalize IMU
    //initalize pins

    
    if (!rvc.begin(&Serial1)) { // connect to the sensor over hardware serial
        Serial.println("Could not find BNO08x");
        while (1)
        delay(10);
    }

    Serial.println("Found BNO08x");
}

void loop()
{
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
    current_velocity_y = y_velocity(current_velocity_y, delta_time, y_acceleration);

    uint8_t start_byte = '!'; //stores the ascii value of ! into the variable
    uint32_t sequence_id = 0x0000;
    uint16_t id = 0x696D; //hex for "im", the id for IMU
    uint32_t timestamp = 0x00000001;

    //We have a 17 byte payload, but our data (current_velocity_y) is only 4 bits! To ensure our payload
    //always stays 17 bytes long, we will have the beginning with zeros. 
    uint8_t payload[17] = {0}; //lets start with making the payload have all 0s, then lets modify its last four bytes to hold our data 
    memcpy(&payload[13], &current_velocity_y, sizeof(current_velocity_y)); 

    uint16_t end_byte = 0x0D0A;


    //time to put together our UART packet
    uint8_t packet[32];

    int offset = 0; //start adding data at byte 13
    packet[offset++] = start_byte;

    packet[offset++] = (sequence_id >> 24) & 0xFF; // Byte 0
    packet[offset++] = (sequence_id >> 16) & 0xFF; // Byte 1
    packet[offset++] = (sequence_id >> 8) & 0xFF;  // Byte 2
    packet[offset++] = sequence_id & 0xFF;         // Byte 3

    packet[offset++] = (id >> 8) & 0xFF; // Byte 0
    packet[offset++] = id & 0xFF;        // Byte 1

    packet[offset++] = (timestamp >> 24) & 0xFF; // Byte 0
    packet[offset++] = (timestamp >> 16) & 0xFF; // Byte 1
    packet[offset++] = (timestamp >> 8) & 0xFF;  // Byte 2
    packet[offset++] = timestamp & 0xFF;         // Byte 3
    
    memcpy(&packet[offset], payload, sizeof(payload));
    offset += sizeof(payload);

    uint16_t crc_16 = calculateCRC(packet, offset);
    packet[offset++] = (crc_16 >> 8) & 0xFF; // Byte 0
    packet[offset++] = crc_16 & 0xFF;        // Byte 1

    packet[offset++] = (end_byte >> 8) & 0xFF; // Byte 0
    packet[offset++] = end_byte & 0xFF;        // Byte 1

    //testing the datapacket with tester class
    tester test;
    if (test.protocolTest(packet, 32))
    {
        Serial.println("Packet Valid");
    } else {
        Serial.println("Packet Invalid");
    }









    //setup all variables for packet creation
    //call createPacket function

    //print packet variables 



    //add delay
    delay(5000);
    


}