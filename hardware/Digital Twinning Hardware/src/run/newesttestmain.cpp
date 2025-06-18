// #include "controllers/ServoRotaryController.h"
// #include "controllers/MotorRotaryController.h"

// DT::ServoRotaryController axisA = DT::ServoRotaryController(4, 5, "A", false, false, DT::Angle::fromDegrees(90.0));
// DT::ServoRotaryController axisB = DT::ServoRotaryController(5, 4, "B", false, false, DT::Angle::fromDegrees(167.0));
// DT::ServoRotaryController axisC = DT::ServoRotaryController(16, 3, "C", true, true, DT::Angle::fromDegrees(90.0));
// DT::MotorRotaryController axisR = DT::MotorRotaryController(2, 2, "R", false, true);

// bool flip = true;
// int counter = 0;

// void setup()
// {
//     Serial.begin(115200);
//     delay(5000);

//     axisA.start();
//     axisB.start();
//     axisC.start();
//     axisR.start();

//     axisA.addAngleToQueue(DT::Angle::fromDegrees(-90.0));
//     axisA.addAngleToQueue(DT::Angle::fromDegrees(90.0));
//     axisA.addAngleToQueue(DT::Angle::fromDegrees(-45.0));
//     axisA.addAngleToQueue(DT::Angle::fromDegrees(45.0));
//     axisA.addAngleToQueue(DT::Angle::fromDegrees(-22.5));
//     axisA.addAngleToQueue(DT::Angle::fromDegrees(22.5));

//     axisB.addAngleToQueue(DT::Angle::fromDegrees(0));
//     axisB.addAngleToQueue(DT::Angle::fromDegrees(90.0));
//     axisB.addAngleToQueue(DT::Angle::fromDegrees(0));
//     axisB.addAngleToQueue(DT::Angle::fromDegrees(90));
//     axisB.addAngleToQueue(DT::Angle::fromDegrees(0));
//     axisB.addAngleToQueue(DT::Angle::fromDegrees(90));

//     axisC.addAngleToQueue(DT::Angle::fromDegrees(0));
//     axisC.addAngleToQueue(DT::Angle::fromDegrees(35));
//     axisC.addAngleToQueue(DT::Angle::fromDegrees(0));
//     axisC.addAngleToQueue(DT::Angle::fromDegrees(35));
//     axisC.addAngleToQueue(DT::Angle::fromDegrees(0));
//     axisC.addAngleToQueue(DT::Angle::fromDegrees(35));

//     axisR.addAngleToQueue(DT::Angle::fromDegrees(-45.0));
//     axisR.addAngleToQueue(DT::Angle::fromDegrees(45.0));
//     axisR.addAngleToQueue(DT::Angle::fromDegrees(-45.0));
//     axisR.addAngleToQueue(DT::Angle::fromDegrees(45.0));
//     axisR.addAngleToQueue(DT::Angle::fromDegrees(0.0));
// }

// void loop()
// {
//     axisA.update();
//     axisB.update();
//     axisC.update();
//     axisR.update();
//     delay(10);
// }