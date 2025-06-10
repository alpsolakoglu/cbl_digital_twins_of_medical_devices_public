#include <ServoRotary.h>

DT::ServoRotary axisA = DT::ServoRotary(4, 5, "A", true, DT::Angle::fromDegrees(90.0));
DT::ServoRotary axisB = DT::ServoRotary(5, 4, "B", true, DT::Angle::fromDegrees(175.0));
DT::ServoRotary axisC = DT::ServoRotary(16, 3, "C", false, DT::Angle::fromDegrees(95.0));


bool flip = true;
bool first = true;
int counter = 0;

void setup() {
  Serial.begin(115200);
  delay(5000);

  axisC.start();
  axisB.start();
  axisA.start();
}

void loop() {
  Serial.println("Reading angles:");
  DT::Angle angleA = axisA.getAngleWithRotary();
  DT::Angle angleB = axisB.getAngleWithRotary();
  DT::Angle angleC = axisC.getAngleWithRotary();

  Serial.println("Axis A: " + String(angleA.getInDegrees()) + " degrees");
  Serial.println("Axis B: " + String(angleB.getInDegrees()) + " degrees");
  Serial.println("Axis C: " + String(angleC.getInDegrees()) + " degrees");
  Serial.println("-----");

  if (counter % 4 == 0) {
    if (flip) {
      axisA.setAngleWithRotary(DT::Angle::fromDegrees(0));
      axisB.setAngleWithRotary(DT::Angle::fromDegrees(0));
      axisC.setAngleWithRotary(DT::Angle::fromDegrees(0));
    } else {
      axisA.setAngleWithRotary(DT::Angle::fromDegrees(45));
      axisB.setAngleWithRotary(DT::Angle::fromDegrees(90));
      axisC.setAngleWithRotary(DT::Angle::fromDegrees(35));
    }
    flip = !flip;
    counter = 0;
  }
  counter++;
  
  delay(500);
}