#include "controllers/ServoRotaryController.h"
#include "controllers/MotorRotaryController.h"
#include "devices/Potentiometer.h"

DT::ServoRotaryController axisA = DT::ServoRotaryController(4, 5, "A", false, false, DT::Angle::fromDegrees(90.0));
DT::ServoRotaryController axisB = DT::ServoRotaryController(5, 4, "B", false, false, DT::Angle::fromDegrees(167.0));
DT::ServoRotaryController axisC = DT::ServoRotaryController(16, 3, "C", true, true, DT::Angle::fromDegrees(90.0));
DT::MotorRotaryController axisR = DT::MotorRotaryController(2, 2, "R", false, true);

bool flip = true;
int counter = 0;

void parseCommandFromSerial()
{
    if (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        command.trim();

        Serial.println("Received command: " + command);

        if (command.startsWith("A"))
        {
            int angle = command.substring(1).toInt();
            axisA.addAngleToQueue(DT::Angle::fromDegrees(angle));
        }
        else if (command.startsWith("B"))
        {
            int angle = command.substring(1).toInt();
            axisB.addAngleToQueue(DT::Angle::fromDegrees(angle));
        }
        else if (command.startsWith("C"))
        {
            int angle = command.substring(1).toInt();
            axisC.addAngleToQueue(DT::Angle::fromDegrees(angle));
        }
        else if (command.startsWith("R"))
        {
            int angle = command.substring(1).toInt();
            axisR.addAngleToQueue(DT::Angle::fromDegrees(angle));
        }
    }
}

// tmp
DT::Potentiometer sideScroll = DT::Potentiometer(32, 0, 2420, 1210, true, -60, 80, 0);
DT::Potentiometer frontBack = DT::Potentiometer(33, 1750, 2420, 2085, false, 0, 160, 0);
DT::Potentiometer backButton = DT::Potentiometer(35, 2030, 2420, 2225, true, -90, 90, 0);

unsigned long lastPotentiometerReadTime = 0;

void setup()
{
    Serial.begin(115200);

    axisA.start();
    axisB.start();
    axisC.start();
    axisR.start();
}

// int sideScroll  = analogRead(32); // 2420 - 0 (all the way back is 2420)
// int frontBack = analogRead(33); // 1750 - 2420 (all the way back is 1750)
// // int c = analogRead(34);
// int backButton = analogRead(35); // 2420 - 2030 (not pressed is 2420, pressed is 2030)

// Serial.println("Analog reading: sideScroll: " + String(sideScroll.getCurrentVirtual()));
// Serial.println("Analog reading: frontBack: " + String(frontBack.getCurrentVirtual()));
// // Serial.println("Analog reading: C: " + String(c));
// Serial.println("Analog reading: backButton: " + String(backButton.getCurrentVirtual()));

void loop()
{
    // if (millis() - lastPotentiometerReadTime > 1000)
    // {
    //     lastPotentiometerReadTime = millis();
    //     Serial.println("Analog reading: sideScroll: " + String(sideScroll.getCurrentVirtual()));
    //     Serial.println("Analog reading: frontBack: " + String(frontBack.getCurrentVirtual()));
    //     Serial.println("Analog reading: backButton: " + String(backButton.getCurrentVirtual()));

    //     axisA.setHoldControllerInputAngle(DT::Angle::fromDegrees(sideScroll.getCurrentVirtual()));
    // }
    axisA.setHoldControllerInputAngle(DT::Angle::fromDegrees(backButton.getCurrentVirtual()));
    axisB.setHoldControllerInputAngle(DT::Angle::fromDegrees(frontBack.getCurrentVirtual()));
    axisC.setHoldControllerInputAngle(DT::Angle::fromDegrees(sideScroll.getCurrentVirtual()));
    
    Serial.println("RobotAngleA:" + String(axisA.getCurrentAngle().getInDegrees()));
    Serial.println("RobotAngleB:" + String(axisB.getCurrentAngle().getInDegrees()));
    Serial.println("RobotAngleC:" + String(axisC.getCurrentAngle().getInDegrees()));
    Serial.println("RobotAngleR:" + String(axisR.getCurrentAngle().getInDegrees()));

    // parseCommandFromSerial();

    axisA.update();
    axisB.update();
    axisC.update();
    axisR.update();
    delay(10);
}