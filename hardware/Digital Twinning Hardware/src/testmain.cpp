#include <RotaryEncoder.h>
#include <TCAMultiplexer.h>

DT::RotaryEncoder axisA_rot = DT::RotaryEncoder(16, 2, "A");
DT::RotaryEncoder axisB_rot = DT::RotaryEncoder(5, 3, "B");
DT::RotaryEncoder axisC_rot = DT::RotaryEncoder(4, 4, "C");
DT::RotaryEncoder axisR_rot = DT::RotaryEncoder(2, 5, "R");

void setup() {
  Serial.begin(115200);
  DT::TCAMultiplexer::getInstance().start();
  axisA_rot.start();
  axisB_rot.start();
  axisC_rot.start();
  axisR_rot.start();
}

void loop() {
  Serial.println("Reading angles:");
  DT::Angle angleA = axisA_rot.readAngle();
  DT::Angle angleB = axisB_rot.readAngle();
  DT::Angle angleC = axisC_rot.readAngle();
  DT::Angle angleR = axisR_rot.readAngle();

  Serial.println("Axis A: " + String(angleA.getInDegrees()) + " degrees");
  Serial.println("Axis B: " + String(angleB.getInDegrees()) + " degrees");
  Serial.println("Axis C: " + String(angleC.getInDegrees()) + " degrees");
  Serial.println("Axis R: " + String(angleR.getInDegrees()) + " degrees");
  Serial.println("-----");
  delay(50);
}