#include "TLx493D_inc.hpp"

using namespace ifx::tlx493d;

TLx493D_A1B6 sensor(Wire, TLx493D_IIC_ADDR_A0_e);

void setup() {
  Serial.begin(9600);

  Serial.println(F("TLV493D init..."));

  if (!sensor.begin()) {
    Serial.println(F("ERROR: sensor init failed!"));
    Serial.println(F("Check I2C: SDA=A4, SCL=A5"));
    while (1) {
      delay(1000);
    }
  }

  Serial.println(F("OK"));
}

void loop() {
  double sumX = 0, sumY = 0, sumZ = 0;
  int sampleCount = 0;

  // Collect samples for 1 second (5 samples at 0.2s intervals)
  for (int i = 0; i < 5; i++) {
    double x, y, z;

    if (sensor.getMagneticField(&x, &y, &z)) {
      sumX += x;
      sumY += y;
      sumZ += z;
      sampleCount++;
    }

    delay(200);
  }

  // Print average
  if (sampleCount > 0) {
    Serial.print(F("X:"));
    Serial.print(sumX / sampleCount);
    Serial.print(F(" Y:"));
    Serial.print(sumY / sampleCount);
    Serial.print(F(" Z:"));
    Serial.println(sumZ / sampleCount);
  } else {
    Serial.println(F("ERR"));
  }
}
