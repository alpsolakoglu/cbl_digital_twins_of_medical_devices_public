#include <Wire.h>
#include <AS5600.h>
#include <ESP32Servo.h>

#define TCA_ADDR 0x70  // Default address for PCA9548A

Servo A_axis;
Servo B_axis;
Servo C_axis;
Servo R_axis;

const int A_axis_pin = 16;
const int B_axis_pin = 5;
const int C_axis_pin = 4;
const int R_axis_pin = 2;

// Channel assignments
const uint8_t channels[4] = {2, 3, 4, 5};
AS5600 sensors[4];

void setoffset

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);  // Bitmask to select channel
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();  // SDA = GPIO21, SCL = GPIO22

  //Initialize servos
  A_axis.attach(A_axis_pin);
  B_axis.attach(B_axis_pin);
  C_axis.attach(C_axis_pin);
  R_axis.attach(R_axis_pin);

   // Initialize all sensors on channels 2–5
  for (int i = 0; i < 4; i++) {
    tcaSelect(channels[i]);
    sensors[i].begin();

    String status = sensors[i].isConnected() ? "detected." : "NOT found!";
    Serial.println("Sensor " + String(sensorLabels[i]) +
                   " (CH" + String(channels[i]) + ") " + status);
  }
}

void loop() {
   for (int i = 0; i < 4; i++) {
    tcaSelect(channels[i]);

    int angle = sensors[i].readAngle();       // Raw 12-bit angle
    float deg = angle * 360.0 / 4096.0;       // Convert to degrees

    Serial.println("Sensor " + String(sensorLabels[i]) +
                   " (CH" + String(channels[i]) + "): " +
                   String(angle) + " | " + String(deg, 2) + " deg");
  }

  A_axis.write(95);
  B_axis.write(180);
  C_axis.write(90);


  Serial.println("-------------------------");
  delay(50);
}