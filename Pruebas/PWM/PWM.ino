#define PIN_LED 4  
void setup() { pinMode(PIN_LED, OUTPUT); }
void loop() {
  for (int b = 0; b <= 255; b += 5) { analogWrite(PIN_LED, b); delay(30); } // sube brillo
  for (int b = 255; b >= 0; b -= 5) { analogWrite(PIN_LED, b); delay(30); } // baja brillo
}