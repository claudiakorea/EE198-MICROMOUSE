#include <Wire.h>
#include <VL6180X.h>

VL6180X distance_sensor_center;

void setup() {
  Serial.begin(9600);

  // Initialize I2C -- this is the protocol the VL6180x sensor library uses
  Wire.begin();

  // Set up center sensor with address 0x01
  configureDistanceSensor(&distance_sensor_center, 0x01);
}

void loop() {
  // Read a distance and store it in the distance variable
  float distance = distance_sensor_center.readRangeContinuousMillimeters();
  Serial.println(distance);
}

//////////////////////
// Helper functions //
//////////////////////

void configureDistanceSensor(VL6180X* sensor, uint8_t addr) {
  delay(100);
  sensor->setAddress(addr);
  sensor->init();
  sensor->configureDefault();
  sensor->writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor->writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor->setTimeout(500);
  sensor->stopContinuous();
  sensor->startRangeContinuous(100);
}
