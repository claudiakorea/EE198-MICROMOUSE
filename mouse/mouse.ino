#include "pins.h"

#include <VL6180X.h>
#include <Wire.h>
#include <PID_v1.h>

// Invert encoder directions if needed
const boolean INVERT_ENCODER_LEFT = true;
const boolean INVERT_ENCODER_RIGHT = true;

// Invert motor directions if needed
const boolean INVERT_MOTOR_LEFT = false;
const boolean INVERT_MOTOR_RIGHT = true;

// Loop count, used for print statements
int count = 0;

// Sensor states
double velocity_angular = 0;
double velocity_linear = 0;
float left_dist;
float right_dist;
float center_dist;

// Power variables to apply
double velocity_linear_power;
double velocity_angular_power;

// Angular and Linear setpoints
double velocity_linear_setpoint = 400;
double velocity_angular_setpoint = 0;

// DiFfErENTtiAL EQuatIoNs
double kp_angular = 0.4;
double kp_linear = 0.004;

// Integral
double ki_angular = 0.05;
double ki_linear = 0.0005;

// Derivative
double kd = 0.00005;

// controllers
PID pid_linear(&velocity_linear, &velocity_linear_power, &velocity_linear_setpoint, kp_linear, ki_linear, kd, DIRECT);
PID pid_angular(&velocity_angular, &velocity_angular_power, &velocity_angular_setpoint, kp_angular, ki_angular, kd, DIRECT);


void setup() {
  Serial.begin(9600);
  hardwareSetup();
  // velocity_linear = 0;
  
  pid_linear.SetOutputLimits(-1.0, 1.0); // 
  pid_angular.SetOutputLimits(-1.0, 1.0); // nani?!?!?
  pid_linear.SetSampleTime(10); // Run control loop at 100Hz (10ms period)
  pid_angular.SetSampleTime(10);

  // Turn the PID loop on
  pid_linear.SetMode(AUTOMATIC);
  pid_angular.SetMode(AUTOMATIC);
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

  pid_linear.Compute();
  pid_angular.Compute();

  // float left_power = 0.2;
  // float right_power = 0.2;

  // float ang_error = -1 * velocity_angular;
  // float ang_u = ang_error * kp;

  // float lin_error = 500 - velocity_linear;
  // float lin_u = lin_error * k_lin;

  // applyPowerLeft(lin_u - ang_u);
  // applyPowerRight(lin_u + ang_u);
  applyPowerLeft(velocity_linear_power - velocity_angular_power);
  applyPowerRight(velocity_linear_power + velocity_angular_power);

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
//      Serial.print(" ");
//      Serial.print(ang_error);
      Serial.println();
  }
  count++;

  checkEncodersZeroVelocity();
  updateDistanceSensors();
}
