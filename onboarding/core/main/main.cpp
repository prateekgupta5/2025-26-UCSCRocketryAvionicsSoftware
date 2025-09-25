#include <Arduino.h>

const long baudrate = 57600;

//packet structure
struct UART_Packet
{
    uint8_t start_byte;
    uint32_t sequence_id;
    uint16_t id;
    uint32_t timestamp;
    uint8_t payload[17]; 
    uint16_t crc_16;
    uint16_t end_byte;
};

//this will hold values after state estimation is done
struct IMU_Data
{
    float ax;
    float ay;
    float az;
};

//relevent functions 

IMU_Data stateEstimation(uint8_t x, uint8_t y, uint8_t z)
{
    //subtract x, y and z from their bias: x -= x_bias
    //multiply x, y, and z with the scale factor to convert counts given from IMU to m/s^2 unit

    //store in struct
    IMU_Data returnData = {x, y, z};

    return returnData;
}

UART_Packet createPacket(uint8_t start_byte, uint32_t sequence_id, uint16_t id, uint32_t timestamp, uint8_t payload[17], uint16_t crc_16, uint16_t end_byte)
{
    UART_Packet returnPacket = {start_byte, sequence_id, id, timestamp, payload, crc_16, end_byte};

    return returnPacket;
}


void setup()
{
    Serial.begin(baudrate);
    //initalize IMU
    //initalize pins
}

void loop()
{
    //read IMU data and store in three variables 
    //call stateEstimation function 

    //setup all variables for packet creation
    //call createPacket function

    //print packet variables 
    //add delay

    

}