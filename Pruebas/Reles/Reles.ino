#define RELE_BOMBA 8
#define RELE_VENT  9

//HIGH = Apagado | LOW = Encendido
void setup() {
  Serial.begin(9600);
  pinMode(RELE_BOMBA, OUTPUT);
  pinMode(RELE_VENT, OUTPUT);
  digitalWrite(RELE_BOMBA, HIGH); // apagado
  digitalWrite(RELE_VENT, HIGH);  // apagado
}
void loop() {
  Serial.println("BOMBA ON");
  digitalWrite(RELE_BOMBA, LOW);  delay(1500);
  Serial.println("BOMBA OFF");
  digitalWrite(RELE_BOMBA, HIGH); delay(1500); 
  Serial.println("Ventilador ON");
  digitalWrite(RELE_VENT, LOW);   delay(1500); 
  Serial.println("Ventilador OFF");  
  digitalWrite(RELE_VENT, HIGH);  delay(1500); 
}