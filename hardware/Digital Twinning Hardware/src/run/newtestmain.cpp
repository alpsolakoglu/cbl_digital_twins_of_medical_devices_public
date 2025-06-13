// #include "ServoRotary.h"
// #include "devices/Motor.h"

// DT::ServoRotary axisA = DT::ServoRotary(4, 5, "A", true, DT::Angle::fromDegrees(90.0));
// DT::ServoRotary axisB = DT::ServoRotary(5, 4, "B", true, DT::Angle::fromDegrees(175.0));
// DT::ServoRotary axisC = DT::ServoRotary(16, 3, "C", false, DT::Angle::fromDegrees(95.0));
// DT::Motor motorR = DT::Motor(2, "R");

// bool flip = true;
// int counter = 0;

// void setup()
// {
//   Serial.begin(115200);
//   delay(5000);

//   axisC.start();
//   axisB.start();
//   axisA.start();
//   motorR.start();
// }

// void loop()
// {
//   Serial.println("Reading angles:");
//   DT::Angle angleA = axisA.getAngleWithRotary();
//   DT::Angle angleB = axisB.getAngleWithRotary();
//   DT::Angle angleC = axisC.getAngleWithRotary();

//   Serial.println("Axis A: " + String(angleA.getInDegrees()) + " degrees");
//   Serial.println("Axis B: " + String(angleB.getInDegrees()) + " degrees");
//   Serial.println("Axis C: " + String(angleC.getInDegrees()) + " degrees");
//   Serial.println("-----");

//   uint16_t pulseWidthRange = motorR.getMaxPulseWidth() - motorR.getMinPulseWidth();
//   if (counter % 2 == 0)
//   {
//     if (flip)
//     {
//       // motorR.drive(motorR.getMinPulseWidth() + pulseWidthRange / 4);
//       axisA.setAngleWithRotary(DT::Angle::fromDegrees(0));
//       axisB.setAngleWithRotary(DT::Angle::fromDegrees(0));
//       axisC.setAngleWithRotary(DT::Angle::fromDegrees(0));
//     }
//     else
//     {
//       // motorR.drive(motorR.getMinPulseWidth() + pulseWidthRange / 2);
//       axisA.setAngleWithRotary(DT::Angle::fromDegrees(45));
//       axisB.setAngleWithRotary(DT::Angle::fromDegrees(90));
//       axisC.setAngleWithRotary(DT::Angle::fromDegrees(35));
//     }
//     flip = !flip;
//     counter = 0;
//   }
//   counter++;

//   delay(500);
// }