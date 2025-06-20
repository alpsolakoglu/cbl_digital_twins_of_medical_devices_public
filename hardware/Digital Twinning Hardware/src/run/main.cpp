#include "controllers/ServoRotaryController.h"
#include "controllers/MotorRotaryController.h"
#include "devices/Potentiometer.h"

DT::Potentiometer sideScroll = DT::Potentiometer(32, 0, 2260, 1130, true, -60, 80, 0);
DT::Potentiometer frontBack = DT::Potentiometer(33, 1650, 2270, 1960, false, 0, 160, 0);
DT::Potentiometer twist = DT::Potentiometer(34, 0, 2250, 1125, true, -60, 60, 0);
DT::Potentiometer backButton = DT::Potentiometer(35, 1900, 2270, 2085, true, -90, 90, 0);

DT::ServoRotaryController axisA = DT::ServoRotaryController(4, 5, "A", false, false, DT::Angle::fromDegrees(90.0), &backButton, 0.9);
DT::ServoRotaryController axisB = DT::ServoRotaryController(5, 4, "B", false, false, DT::Angle::fromDegrees(167.0), &frontBack, 0.93);
DT::ServoRotaryController axisC = DT::ServoRotaryController(16, 3, "C", true, true, DT::Angle::fromDegrees(90.0), &sideScroll, 0.9);
DT::MotorRotaryController axisR = DT::MotorRotaryController(2, 2, "R", false, true, DT::Angle::fromDegrees(0.0), &twist, 0.9);

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
    // Serial.println("RobotAngleA:" + String(axisA.getCurrentAngle().getInDegrees()));
    // Serial.println("RobotAngleB:" + String(axisB.getCurrentAngle().getInDegrees()));
    // Serial.println("RobotAngleC:" + String(axisC.getCurrentAngle().getInDegrees()));
    // Serial.println("RobotAngleR:" + String(axisR.getCurrentAngle().getInDegrees()));

    axisA.update();
    axisB.update();
    axisC.update();
    axisR.update();
    delay(10);
}