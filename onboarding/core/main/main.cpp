#include <Arduino.h>
//below is the library for the IMU, which is the BNO088
#include "Adafruit_BNO08x_RVC.h"


Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
const long baudrate = 115200;
//this is the sensor's factury default baud rate for the RVC setting. 

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
    //read IMU data and store in three variables 
    //call stateEstimation function 

    //setup all variables for packet creation
    //call createPacket function

    //print packet variables 
    //add delay

    

}