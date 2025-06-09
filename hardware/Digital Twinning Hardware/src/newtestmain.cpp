#include <ServoRotary.h>

DT::ServoRotary axisA = DT::ServoRotary(4, 5, "A", true, DT::Angle::fromDegrees(90.0));

bool flip = true;
bool first = true;
int counter = 0;

void setup() {
  Serial.begin(115200);
  delay(5000);

  axisA.start();
}

void loop() {
  Serial.println("Reading angles:");
  DT::Angle angleA = axisA.getAngleWithRotary();

  Serial.println("Axis A: " + String(angleA.getInDegrees()) + " degrees");
  Serial.println("-----");

  if (counter % 4 == 0) {
    if (flip) {
      axisA.setAngleWithRotary(DT::Angle::fromDegrees(0));
    } else {
      axisA.setAngleWithRotary(DT::Angle::fromDegrees(-90));
    }
    flip = !flip;
    counter = 0;
  }
  counter++;
  
  delay(500);
}