#include <RotaryEncoder.h>
#include <TCAMultiplexer.h>
#include <Servo.h>


DT::RotaryEncoder axisA_rot = DT::RotaryEncoder(5, "A", true);
DT::RotaryEncoder axisB_rot = DT::RotaryEncoder(4, "B", true);
DT::RotaryEncoder axisC_rot = DT::RotaryEncoder(3, "C", false);
DT::RotaryEncoder axisR_rot = DT::RotaryEncoder(2, "R", true);

DT::Servo axisA_servo = DT::Servo(4, "A", DT::Angle::fromDegrees(90));
DT::Servo axisB_servo = DT::Servo(5, "B", DT::Angle::fromDegrees(175));
DT::Servo axisC_servo = DT::Servo(16, "C", DT::Angle::fromDegrees(95));

bool flip = true;
bool first = true;
int counter = 0;

void setup() {
  Serial.begin(115200);
  delay(5000);

  axisA_rot.start();
  axisB_rot.start();
  axisC_rot.start();
  axisR_rot.start();

  axisA_servo.start();
  axisB_servo.start();
  axisC_servo.start();
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

  if (counter % 4 == 0) {
    if (flip) {
      axisA_servo.setAngle(DT::Angle::fromDegrees(90));
      axisB_servo.setAngle(DT::Angle::fromDegrees(175));
      axisC_servo.setAngle(DT::Angle::fromDegrees(95));

      if (first) {
        delay(4000); // Wait for servos to stabilize
        bool a = axisA_rot.configure();
        bool b = axisB_rot.configure();
        bool c = axisC_rot.configure();
        bool r = axisR_rot.configure();
        Serial.println("Axis A configured: " + String(a));
        Serial.println("Axis B configured: " + String(b));
        Serial.println("Axis C configured: " + String(c));
        Serial.println("Axis R configured: " + String(r));
        Serial.println("-----");
        first = false;
      }
    } else {
      axisA_servo.setAngle(DT::Angle::fromDegrees(45));
      axisB_servo.setAngle(DT::Angle::fromDegrees(85));
      axisC_servo.setAngle(DT::Angle::fromDegrees(130));
    }
    flip = !flip;
    counter = 0;
  }
  counter++;
  
  delay(500);
}