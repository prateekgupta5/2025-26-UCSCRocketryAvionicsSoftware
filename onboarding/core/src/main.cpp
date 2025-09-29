#include <Arduino.h>
//below is the library for the IMU, which is the BNO088
#include "Adafruit_BNO08x_RVC.h"


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

    //read IMU acceleration data and store in three variables 
    float x_acceleration = heading.x_accel;
    float y_acceleration = heading.y_accel;
    float z_acceleration = heading.z_accel;

    //store in struct
    IMU_Data acceleration_data = {x_acceleration, y_acceleration, z_acceleration};

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
    std::memcpy(&payload[13], &current_velocity_y, sizeof(current_velocity_y)); 

    uint16_t crc_16 = 0x1101 //random checksum
    uint16_t end_byte = 0x0D0A


    //time to put together our UART packet
    uint8_t packet[32];

    int offset = 0; //start adding data at byte 13
    std::memcpy(&packet[offset], &start_byte, sizeof(start_byte));
    offset += sizeof(start_byte);
    std::memcpy(&packet[offset], &sequence_id, sizeof(sequence_id));
    offset += sizeof(sequence_id);
    std::memcpy(&packet[offset], &id, sizeof(id));
    offset += sizeof(id);
    std::memcpy(&packet[offset], &timestamp, sizeof(timestamp));
    offset += sizeof(timestamp);
    std::memcpy(&packet[offset], payload, sizeof(payload));
    offset += sizeof(payload);
    std::memcpy(&packet[offset], &crc_16, sizeof(crc_16));
    offset += sizeof(crc_16);
    std::memcpy(&packet[offset], &end_byte, sizeof(end_byte));
    offset += sizeof(end_byte);

    if (protocolTest(packet, 32))
    {
        Serial.println("Packet Valid");
    }









    //setup all variables for packet creation
    //call createPacket function

    //print packet variables 



    //add delay
    delay(5000);
    


}