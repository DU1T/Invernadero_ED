#define PIN_SUELO A0
#define VAL_SECO 560   // ajustar
#define VAL_AGUA 240   // ajustar
void setup() { Serial.begin(9600); }
void loop() {
  int crudo = analogRead(PIN_SUELO);
  int pct = constrain(map(crudo, VAL_SECO, VAL_AGUA, 0, 100), 0, 100);
  Serial.print("Crudo: "); Serial.print(crudo);
  Serial.print("   Humedad: "); Serial.print(pct); Serial.println(" %");
  delay(800);
}