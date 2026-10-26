/* ============================================================
   ESP32 - PRUEBA DEL ENLACE CON EL MEGA (fase 5)
   Imprime por USB (115200) todo lo que manda el Mega por el
   TXS0108E, y reenvia al Mega lo que escribas en el monitor.

   Cableado (TXS0108E: VA=3.3V, VB=5V, OE=3.3V, GND comun):
     Mega D18 (TX1) -> B1 | A1 -> ESP32 PIN_RX
     ESP32 PIN_TX   -> A2 | B2 -> Mega D19 (RX1)

   Pines segun el modulo:
     WROOM (clasico) : RX 16, TX 17  (Serial2)
     WROVER          : 16/17 son de la PSRAM -> usa RX 25, TX 26
     S2 / S3         : no hay Serial2 -> cambia MEGA por Serial1
   Prueba: escribe  #10 VENT ON 30  y espera {"ack":10,"ok":1}
   ============================================================ */
#define PIN_RX 16
#define PIN_TX 17
#define MEGA   Serial2      // Serial1 en ESP32-S2 / S3

String linea;
unsigned long ultimaTrama = 0;
uint32_t tramas = 0;

void setup() {
  Serial.begin(115200);
  MEGA.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(300);
  Serial.println("\nESP32 listo. Esperando JSON del Mega cada 2 s...");
}

void loop() {
  // Del Mega al monitor
  while (MEGA.available()) {
    char c = MEGA.read();
    if (c == '\n') {
      linea.trim();
      if (linea.length()) {
        if (linea.startsWith("{\"ack\"")) {
          Serial.print("ACK  <- "); Serial.println(linea);
        } else {
          tramas++; ultimaTrama = millis();
          Serial.printf("JSON #%lu (%u bytes) <- ", (unsigned long)tramas, linea.length());
          Serial.println(linea);
        }
      }
      linea = "";
    } else if (c != '\r' && linea.length() < 600) {
      linea += c;
    }
  }
  // Del monitor al Mega
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length()) { MEGA.println(cmd); Serial.print("CMD  -> "); Serial.println(cmd); }
  }
  // Aviso si no llega nada
  static unsigned long tAviso = 0;
  if (millis() - ultimaTrama > 5000 && millis() - tAviso > 5000) {
    tAviso = millis();
    Serial.println("Sin datos del Mega: revisa D18->B1, A1->RX, OE a 3.3V, GND comun y el baud (9600).");
  }
}
