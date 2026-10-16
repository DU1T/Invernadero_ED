#include <ModbusMaster.h>

// ================= DIAGNOSTICO 7-EN-1 RS485 (Mega) =================
#define DE_RE 7

uint16_t modbusCRC(uint8_t *buf, int len){
  uint16_t crc = 0xFFFF;
  for(int i=0;i<len;i++){
    crc ^= buf[i];
    for(int b=0;b<8;b++){
      if(crc & 1){ crc >>= 1; crc ^= 0xA001; } else crc >>= 1;
    }
  }
  return crc;
}

long bauds[] = {4800, 9600};

void setup(){
  Serial.begin(9600);
  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW);      // recepcion por defecto
  Serial.println(F("== Diagnostico 7-en-1 RS485 =="));
}

void probe(long baud, uint8_t addr){
  Serial2.begin(baud);
  delay(40);
  uint8_t f[8] = {addr, 0x03, 0x00, 0x00, 0x00, 0x07, 0, 0};  // leer 7 regs desde 0x0000
  uint16_t crc = modbusCRC(f, 6);
  f[6] = crc & 0xFF; f[7] = crc >> 8;

  while(Serial2.available()) Serial2.read();   // limpiar RX

  digitalWrite(DE_RE, HIGH);                    // transmitir
  delayMicroseconds(50);
  Serial2.write(f, 8);
  Serial2.flush();                              // espera fin de TX
  delayMicroseconds(50);
  digitalWrite(DE_RE, LOW);                     // recibir

  unsigned long t0 = millis();
  uint8_t resp[40]; int n = 0;
  while(millis() - t0 < 300){
    while(Serial2.available() && n < 40) resp[n++] = Serial2.read();
  }

  Serial.print("baud="); Serial.print(baud);
  Serial.print(" addr="); Serial.print(addr);
  Serial.print(" -> bytes="); Serial.print(n);
  if(n > 0){
    Serial.print("  [");
    for(int i=0;i<n;i++){ if(resp[i]<16) Serial.print('0'); Serial.print(resp[i],HEX); Serial.print(' '); }
    Serial.print("]");
  }
  Serial.println();
  Serial2.end();
}

void loop(){
  for(int b=0;b<2;b++)
    for(uint8_t a=1;a<=4;a++)
      probe(bauds[b], a);
  Serial.println(F("--- fin de barrido. Si TODO es bytes=0: revisa fuente 9-24V, GND comun y SWAP A<->B ---"));
  delay(3000);
}