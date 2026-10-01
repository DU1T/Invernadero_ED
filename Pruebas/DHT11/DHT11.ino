#include <DHT.h>
#define DHTPIN 3
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);
void setup() {
  Serial.begin(9600);
  dht.begin();
}
void loop() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
    Serial.print("T: "); Serial.print(t); Serial.print(" C   ");
    Serial.print("H: "); Serial.print(h); Serial.println(" %");
  delay(2000);
}