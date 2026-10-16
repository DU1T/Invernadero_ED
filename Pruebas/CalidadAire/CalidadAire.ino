#define PIN_MQ135 A2
#define PIN_MQ7   A3
void setup() {
  Serial.begin(9600);
}
void loop() {
  int aire = analogRead(PIN_MQ135);
  int co   = analogRead(PIN_MQ7);
  Serial.print("MQ-135 (aire): "); Serial.print(aire);
  Serial.print("   MQ-7 (CO): "); Serial.println(co);
  delay(1000);
}