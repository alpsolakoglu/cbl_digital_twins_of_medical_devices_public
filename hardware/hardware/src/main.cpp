#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>


Servo A_axis;
Servo B_axis;
Servo C_axis;
Servo R_axis;

const int A_axis_pin = 4;
const int B_axis_pin = 5;
const int C_axis_pin = 16;
const int R_axis_pin = 2;

// Potentiometer input pins (ADC)
const int A_pot_pin = 25;
const int B_pot_pin = 33;
const int C_pot_pin = 34;

const int delayMs = 15;

// int pos1 = 0;
// int dir1 = 1;

// int pos2 = 90;
// int dir2 = 1;

// int pos3 = 45;
// int dir3 = 1;

void setup() {
  Serial.begin(115200);
  Serial.println("Setup done");

  A_axis.attach(A_axis_pin);
  B_axis.attach(B_axis_pin);
  C_axis.attach(C_axis_pin);
  R_axis.attach(R_axis_pin);
}

void loop() {

  // A_axis.write(90);

  // // Read potentiometer values
  // int potA = analogRead(A_pot_pin);
  // int potB = analogRead(B_pot_pin);
  // int potC = analogRead(C_pot_pin);
  // //int portR = analogRead(R_axis_pin);

  // // Map to servo angles (0–180)
  // int angleA = map(potA, 0, 4095, 0, 180);
  // int angleB = map(potB, 0, 4095, 0, 180);
  // int angleC = map(potC, 0, 4095, 0, 180);
  // //int angleR = map(portR, 0, 4095, 0, 180);

  // // Move servos
  // A_axis.write(angleA);
  // B_axis.write(angleB);
  // C_axis.write(angleC);
  // //R_axis.write(angleR);

  // delay(15);  
  // Sweep A_axis: 0–180 and back
  // for (int pos = 0; pos <= 180; pos++) {
  //   A_axis.write(pos);
  //   delay(delayMs);
  // }
  // for (int pos = 180; pos >= 0; pos--) {
  //   A_axis.write(pos);
  //   delay(delayMs);
  // }

  // // Sweep B_axis: 90–180 and back 
  // for (int pos = 90; pos <= 180; pos++) {
  //   B_axis.write(pos);
  //   delay(delayMs);
  // }
  // for (int pos = 180; pos >= 90; pos--) {
  //   B_axis.write(pos);
  //   delay(delayMs);
  // }

  // // Sweep C_axis: 45–110 and back 
  // for (int pos = 45; pos <= 110; pos++) {
  //   C_axis.write(pos);
  //   delay(delayMs);
  // }
  // for (int pos = 110; pos >= 45; pos--) {
  //   C_axis.write(pos);
  //   delay(delayMs);
  // }

  // // Sweep R_axis: 0–? and back
  // for (int pos = 0; pos <= ; pos++) {
  //   R_axis.write(pos);
  //   delay(delayMs);
  // }
  // for (int pos = ; pos >= 0; pos--) {
  //   R_axis.write(pos);
  //   delay(delayMs);
  // }

  // float degrees = (rawAngle * 360.0) / 4096.0;
  
  // if (degrees > 260) {
  //   rotate_clockwise = false;
  // } else if (degrees < 180) {
  //   rotate_clockwise = true;
  // }

  // if (rotate_clockwise) {
  //   Serial.println("Clockwise");
  //   R_axis.write(0);
  // } else {
  //   Serial.println("Counter-Clockwise");
  //   R_axis.write(180);
  // }

  delay(delayMs);
  
}

// put function definitions here:
