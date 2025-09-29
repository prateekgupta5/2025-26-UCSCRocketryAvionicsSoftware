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
 

float y_velocity (float initial_velocity_y, float delta_time, float y_acceleration)
{
    // To calculate the velocity, we will be using this simple physics equation: y_f = y_i + a(△t), assuming that acceleration (a) is constant for the short time period. 
    
}


template <typename T>
void writeToPacket(uint8_t* packet, int& offset, T data) {
    
}


uint16_t calculateCRC(uint8_t* data, size_t length)
{
    
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

    //create payload and prepend with 0s up until 17 bytes
    
    //put together our UART packet
    

    //testing the datapacket with tester class
    tester test;
    test.protocolTest(packet, 32);
    delay(5000);
}