#include "src/Imu/Imu.h"

Imu imu(0);

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
    Serial.println("Hello1");

  imu.initSensors();
    Serial.println("Hello2");
}

void loop() { // run over and over
  Serial.println("Hello");
  imu.updateData();
  Serial.println(imu.getRoll());
  Serial.println(imu.getPitch());
  Serial.println(imu.getYaw());
  delay(100);
}
