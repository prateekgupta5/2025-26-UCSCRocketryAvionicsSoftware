#include <Arduino.h>
#include <iostream>
#include <cstring>

const long baudrate = 115200;

void setup()
{
    Serial.begin(baudrate);
    while (!Serial)
    {
        delay(10);
    }
    Serial.println("This is on the BlackPill board");
}

void loop()
{
    Serial.println("Hello World - BlackPill");
    delay(1000);
}