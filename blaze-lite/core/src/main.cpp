#include <Arduino.h>
#include <iostream>
#include <cstring>
#include "MS5611.h"

const long baudrate = 115200;

MS5611 MS5611(0x77);

void setup()
{
    Serial.begin(baudrate);
    while (!Serial)
    {
        Serial.println();
        Serial.println(__FILE__);
        Serial.print("MS5611_LIB_VERSION: ");
        Serial.println(MS5611_LIB_VERSION);
        Serial.println();
        delay(100);
    }
    Serial.println("This is on the BlackPill board");

    Wire.begin();

    if (MS5611.begin() == true)
    {
    Serial.print("MS5611 found: ");
    Serial.println(MS5611.getAddress());
    }
    else
    {
    Serial.println("MS5611 not found. halt.");
    //  while (1);
    }




    Serial.println();
    Serial.println("Celsius\tmBar\tPascal");
}



void loop()
{
    MS5611.read();           //  note no error checking => "optimistic".
    Serial.print(MS5611.getTemperature(), 2);
    Serial.print('\t');
    Serial.print(MS5611.getPressure(), 2);
    Serial.print('\t');
    Serial.print(MS5611.getPressurePascal(), 2);
    Serial.println();
    delay(1000);
}