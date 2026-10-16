#define PIN_PIR 3
void setup() {
  Serial.begin(9600);
  pinMode(PIN_PIR, INPUT);
}
void loop() {
  Serial.println(digitalRead(PIN_PIR) == HIGH ? "Movimiento detectado" : "Sin movimiento");
  delay(300);
}