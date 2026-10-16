#define PIN_BUZZER 11   // buzzer activo via 2N2222
#define PIN_LUZ    12   // luz piloto 12V via TIP120
void setup() {
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LUZ, OUTPUT);
}
void loop() {
  digitalWrite(PIN_BUZZER, HIGH); digitalWrite(PIN_LUZ, HIGH); delay(400); // alarma ON
  digitalWrite(PIN_BUZZER, LOW);  digitalWrite(PIN_LUZ, LOW);  delay(600); // alarma OFF
}