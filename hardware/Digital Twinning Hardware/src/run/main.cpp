#include "controllers/ServoRotaryController.h"
#include "controllers/MotorRotaryController.h"
#include "Arduino.h"

DT::ServoRotaryController axisA = DT::ServoRotaryController(4, 5, "A", false, false, DT::Angle::fromDegrees(90.0));
DT::ServoRotaryController axisB = DT::ServoRotaryController(5, 4, "B", false, false, DT::Angle::fromDegrees(167.0));
DT::ServoRotaryController axisC = DT::ServoRotaryController(16, 3, "C", true, true, DT::Angle::fromDegrees(90.0));
DT::MotorRotaryController axisR = DT::MotorRotaryController(2, 2, "R", false, true);

bool flip = true;
int counter = 0;

void parseCommandFromSerial() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        Serial.println("Received command: " + command);

        if (command.startsWith("A")) {
            int angle = command.substring(1).toInt();
            axisA.addAngleToQueue(DT::Angle::fromDegrees(angle));
        } else if (command.startsWith("B")) {
            int angle = command.substring(1).toInt();
            axisB.addAngleToQueue(DT::Angle::fromDegrees(angle));
        } else if (command.startsWith("C")) {
            int angle = command.substring(1).toInt();
            axisC.addAngleToQueue(DT::Angle::fromDegrees(angle));
        } else if (command.startsWith("R")) {
            int angle = command.substring(1).toInt();
            axisR.addAngleToQueue(DT::Angle::fromDegrees(angle));
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(5000);

    axisA.start();
    axisB.start();
    axisC.start();
    axisR.start();
}

void loop()
{
    parseCommandFromSerial();

    axisA.update();
    axisB.update();
    axisC.update();
    axisR.update();
    delay(10);
}

