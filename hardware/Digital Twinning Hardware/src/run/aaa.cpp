#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
    delay(5000);

    Serial.println("Hello, world!");
}

void loop()
{
    Serial.println("Looping...");
    delay(1000);
}