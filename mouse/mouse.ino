#include "pins.h"

#include <VL6180X.h>
#include <Wire.h>

// Invert encoder directions if needed
const boolean INVERT_ENCODER_LEFT = true;
const boolean INVERT_ENCODER_RIGHT = true;

// Invert motor directions if needed
const boolean INVERT_MOTOR_LEFT = false;
const boolean INVERT_MOTOR_RIGHT = true;

// Loop count, used for print statements
int count = 0;

// Sensor states
float velocity_angular = 0;
float velocity_linear = 0;
float left_dist;
float right_dist;
float center_dist;

//DiFfErENTtiAL EQuatIoNs
float kp = 0.5;
float k_lin = 0.01;

void setup() {
  Serial.begin(9600);
  hardwareSetup();
}

void loop() {
  // Read sensor data
  left_dist = getDistanceLeft();
  right_dist = getDistanceRight();
  center_dist = getDistanceCenter();

  velocity_linear = getLinearVelocity();
  velocity_angular = getAngularVelocity();
  
  ////////////////////////////////////
  // Your changes should start here //
  ////////////////////////////////////

  float left_power = 0.2;
  float right_power = 0.2;

  float ang_error = -1 * velocity_angular;
  float ang_u = ang_error * kp;

  float lin_error = 500 - velocity_linear;
  float lin_u = lin_error * k_lin;

  applyPowerLeft(lin_u - ang_u);
  applyPowerRight(lin_u + ang_u);

  // Print debug info every 500 loops
  if (count % 500 == 0) {
    Serial.print(velocity_linear / 100.0);
    Serial.print(" ");
    Serial.print(velocity_angular);
//    Serial.print(" ");
//    Serial.print(left_dist);
//    Serial.print(" ");
//    Serial.print(center_dist);
//    Serial.print(" ");
//    Serial.print(right_dist);
      Serial.print(" ");
      Serial.print(ang_error);
      Serial.println();
  }
  count++;

  checkEncodersZeroVelocity();
  updateDistanceSensors();
}
