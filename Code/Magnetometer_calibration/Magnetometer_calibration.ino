#include <DFRobot_QMC5883.h>
#include <MPU9250_WE.h>
#include <Wire.h>

DFRobot_QMC5883 compass(&Wire, 0x0D);

void setup() {
  Serial.begin(115200);
  while (!compass.begin()) {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }

}

void loop() {
  sVector_t mag = compass.readRaw();

  Serial.print(mag.XAxis/10);
  Serial.print(" ");
  Serial.print(mag.YAxis/10);
  Serial.print(" ");
  Serial.print(mag.ZAxis/10);
  Serial.println();

  delay(100);
  
}
