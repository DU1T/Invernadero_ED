#include <Wire.h>
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println("Escaneando bus I2C...");
}
void loop() {
  byte count = 0;
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("Dispositivo encontrado en 0x");
      Serial.println(a, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("Ninguno (revisa cableado y pull-ups)");
  Serial.println("----");
  delay(3000);
}