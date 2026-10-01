#define PIN_FLOTADOR 4   // en el diseno final del Mega ira en A1
void setup() {
  Serial.begin(9600);
  pinMode(PIN_FLOTADOR, INPUT_PULLUP);
}
void loop() {
  bool hayAgua = digitalRead(PIN_FLOTADOR); // LOW = flotador cierra a GND
  Serial.println(hayAgua ? "Nivel OK (hay agua)" : "NIVEL BAJO");
  delay(500);
}