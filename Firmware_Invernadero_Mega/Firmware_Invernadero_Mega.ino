/* ============================================================
   INVERNADERO INTELIGENTE - FIRMWARE v3 (Arduino Mega 2560)
   v3 (2026-10-25): v2 + control de acceso con RFID RC522 (ver seccion RFID).
   Sensores + logica + LCD + JSON al ESP32 + comandos con id/ACK

   Base: version del 2026-10-03 (la que mejor funciono). Se conservan
   pines, calibraciones (UMBRAL_LUZ 100, MQ135 180, MQ7 270, riego 30/60,
   pH 6.0-7.5), mapa del 7-en-1 y pantallas de la LCD.

   Cambios v2 (2026-10-20), ajustes 1-5 acordados con software:
     1. Protecciones SIEMPRE, tambien en manual:
          sin agua  -> bomba apagada
          gas alto  -> ventilador + alarma encendidos
     2. Manual POR ACTUADOR con tiempo (TTL). 600 s por defecto,
        maximo 3600 s. Al vencer, ese actuador vuelve a automatico.
     3. Comandos con id y ACK:  #507 BOMBA ON 600
          -> {"ack":507,"ok":1,"cmd":"BOMBA ON 600"}   (cmd = lo que el Mega entendio)
          -> {"ack":508,"ok":0,"err":"NO_WATER"}
        Errores: NO_WATER, INTERLOCK_CO, BAD_CMD, BAD_VAL.
        Un id repetido devuelve el mismo ACK sin repetir la accion.
     4. No decide con lecturas fallidas (dht_ok / suelo_ok).
     5. JSON v1 compatible con la web: v, seq, up, dht_ok, suelo_ok,
        ventana, puerta, man; aptitud en MAYUSCULAS; sin "manual".
   Ademas: alarma estable (histeresis + tiempo minimo + precalentamiento
   de los MQ), luz con histeresis, buffers char[] en vez de String.

   Comandos (Monitor Serial a 9600 o ESP32 por Serial1). El #id es
   opcional en el Monitor; el ESP32 siempre lo manda:
     BOMBA ON|OFF [ttl]   VENT ON|OFF [ttl]   ALARMA ON|OFF [ttl]
     LUZ 0..100|ON|OFF [ttl]  VENTANA 0..90 [ttl]  PUERTA 0..90 [ttl]
     AUTO                 (todo a automatico)
     AUTO BOMBA           (solo ese actuador)

   RFID (v3):
     - El Mega decide quien entra con una lista guardada en EEPROM (no se
       borra al apagar). Sin EEPROM grabada usa las 4 tarjetas del equipo.
     - Tarjeta valida: bip corto, LCD "Hola <nombre>", puerta abierta 5 s
       (aunque la puerta este en manual). Desconocida: 3 bips.
     - Cada intento se reporta al ESP32:  {"rfid":"53D80151220001","ok":1}
     - La web cambia la lista; el ESP32 la envia con lineas que empiezan
       con '@' (no usan #id ni ACK de comandos):
         @CARDS?                     -> {"cards":<ver>,"ok":1,"n":<n>}
         @CARDS <ver> <n>            inicia la lista nueva
         @CARD <uidhex> [nombre]     una por tarjeta
         @END <ver>                  graba si llegaron las n -> {"cards":<ver>,"ok":1,"n":<n>}
                                     si no -> {"cards":<ver>,"ok":0}
       Desde el Monitor Serial tambien sirve  @LIST  para ver la lista.

   Librerias: LiquidCrystal_I2C, DHT (+Adafruit Unified Sensor), Servo, BH1750,
   MFRC522 (GithubCommunity). SPI y EEPROM ya vienen con el IDE.
   (ModbusMaster ya NO se usa: la lectura del 7-en-1 esta escrita aqui)
   ============================================================ */
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Servo.h>

// ---------- CONFIGURACION ----------
#define USA_BH1750      1      // 1 = sensor de luz activo
#define USA_7EN1        1      // 1 = sensor de suelo 7-en-1 (RS485)
#define USA_MQ7         0      // 0 = ignora el MQ-7 (CO) mientras se revisa su cableado
#define T_7EN1_FALLA_MS 5000UL  // si el 7-en-1 no responde, reintenta cada 5 s (cada intento bloquea 0.25 s)
#define DHTTYPE         DHT11  // DHT22 si usas ese
#define LCD_ADDR        0x27   // 0x3F si el escaner I2C dice eso
#define LCD_COLS        16
#define LCD_ROWS        2
#define NIVEL_HAY_AGUA  LOW    // estado del pin A1 cuando HAY agua (confirmar con la prueba del flotador)
#define USA_RFID        1      // 1 = lector RC522 abre la puerta | 0 = sin RFID
#define PUERTA_RFID_MS  5000UL // tiempo que la puerta queda abierta con una tarjeta valida
#define PIR_ABRE_PUERTA 0      // 0 = PIR solo informa (decidido 20 oct; la puerta sera por RFID) | 1 = PIR abre la puerta

// ---------- PINES ----------
#define PIN_DHT      2
#define PIN_PIR      3
#define PIN_LED      4        // PWM (MOSFET tira 12V)
#define PIN_SRV_VENT 5
#define PIN_SRV_PUER 6
#define RS485_DIR    7        // MAX485 DE+RE
#define RELE_BOMBA   8        // activo en bajo
#define RELE_VENT    9        // activo en bajo
#define PIN_BUZZER   11       // buzzer activo via 2N2222
#define PIN_LUZ      12       // luz piloto 12V via TIP120
#define PIN_SUELO    A0       // humedad capacitiva (respaldo si no hay 7-en-1)
#define PIN_NIVEL    A1       // flotador a GND, INPUT_PULLUP
#define PIN_MQ135    A2
#define PIN_MQ7      A3

// ---------- CALIBRACION (ajusta con tus pruebas) ----------
#define SUELO_SECO       560   // crudo capacitivo al aire (solo si USA_7EN1 = 0)
#define SUELO_AGUA       240   // crudo capacitivo en agua (solo si USA_7EN1 = 0)
#define UMBRAL_RIEGO     30    // % : riega por debajo
#define UMBRAL_RIEGO_OFF 60    // % : apaga al superar
#define UMBRAL_TEMP      30.0  // C : ventila por encima (aire)
#define UMBRAL_HUM       80.0  // % : ventila por encima (aire)
#define HIST_TEMP        1.0   // C : deja de ventilar 1 C abajo del umbral
#define HIST_HUM         5.0   // % : deja de ventilar 5 % abajo del umbral
#define UMBRAL_LUZ       100   // lx : enciende la tira por debajo
#define HIST_LUZ         20    // lx : apaga la tira arriba de UMBRAL_LUZ + HIST_LUZ
#define LUZ_PWM_MIN      40    // brillo minimo util cuando enciende (0-255)
#define UMBRAL_MQ135     180   // crudo : aire malo por encima
#define UMBRAL_MQ7       270   // crudo : CO alto por encima
#define HIST_GAS         20    // crudo : la alarma apaga 20 puntos abajo del umbral
#define GAS_MIN_ON_MS    10000UL  // alarma encendida al menos 10 s
#define GAS_PRECAL_MS    60000UL  // ignora los MQ el primer minuto (precalentamiento)
#define PH_APTO_MIN      6.0
#define PH_APTO_MAX      7.5
#define PH_REG_MIN       5.5
#define PH_REG_MAX       8.0

#define TTL_DEFAULT_S    600
#define ACK_CON_CMD      1     // 1 = el ACK ok lleva "cmd" (pedido de Wilson, 20 oct). 0 = ACK sin cmd
#define TTL_MAX_S        3600
#define DHT_FALLAS_MAX   3     // lecturas fallidas seguidas para marcar dht_ok = 0

// ---------- OBJETOS ----------
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
DHT dht(PIN_DHT, DHTTYPE);
Servo servoVentana, servoPuerta;
#if USA_BH1750
  #include <BH1750.h>
  BH1750 luxMeter;
#endif
#if USA_RFID
  #include <SPI.h>
  #include <MFRC522.h>
  #include <EEPROM.h>
  #define RFID_SS        53     // SDA del RC522
  #define RFID_RST       49
  #define RFID_MAX       20     // tarjetas que caben en la lista
  #define RFID_UID_MAX   10     // bytes de UID (4, 7 o 10)
  #define RFID_NOMBRE    11     // 10 letras + fin
  MFRC522 rfid(RFID_SS, RFID_RST);

  struct Tarjeta { uint8_t len; uint8_t uid[RFID_UID_MAX]; char nombre[RFID_NOMBRE]; };
  Tarjeta  tarjetas[RFID_MAX];       // lista vigente (copia de la EEPROM)
  uint8_t  nTarjetas = 0;
  uint32_t verTarjetas = 0;          // version de la lista (la pone la nube; 0 = la de fabrica)
  Tarjeta  nuevas[RFID_MAX];         // lista que esta llegando del ESP32
  uint8_t  nNuevas = 0, esperadas = 0;
  uint32_t verNueva = 0;
  bool     sincronizando = false;
  unsigned long tSync = 0;
  unsigned long puertaRfidHasta = 0;
  bool     puertaRfid = false;
  unsigned long bipInicio = 0;
  bool     bipPermitido = false;
  bool     bipEncendido = false;
  unsigned long rfidMsgHasta = 0;
  char     rfidMsg[2][LCD_COLS + 1];
#endif
#if USA_7EN1
  #define SUELO_ADDR      2      // direccion Modbus del 7-en-1 (confirmada)
  #define SUELO_TIMEOUT_MS 250   // espera maxima de respuesta (bloquea como mucho esto)
#endif

// ---------- ACTUADORES ----------
// Orden fijo: el indice sirve para los arreglos de abajo.
enum { A_BOMBA, A_VENT, A_LUZ, A_VENTANA, A_PUERTA, A_ALARMA, N_ACT };
const char* const NOMBRE_JSON[N_ACT] = { "bomba", "vent", "luz", "ventana", "puerta", "alarma" };
const char* const NOMBRE_CMD [N_ACT] = { "BOMBA", "VENT", "LUZ", "VENTANA", "PUERTA", "ALARMA" };
// Valores: bomba/vent/alarma 0-1, luz 0-255 (PWM), ventana/puerta 0-90 grados
int  valAuto[N_ACT];             // lo que decide la logica automatica
int  valMan [N_ACT];             // lo que pidio el operador
bool manOn  [N_ACT];             // true = ese actuador esta en manual
unsigned long manHasta[N_ACT];   // millis() en que vence el manual
int  salida [N_ACT];             // valor final aplicado a los pines

// ---------- LECTURAS ----------
float t = 0, h = 0, lux = -1;
bool  dhtOK = false;
uint8_t dhtFallas = DHT_FALLAS_MAX;
float humSuelo = 0;
int   aire = 0, co = 0;
bool  hayAgua = true, presencia = false;
float sTemp = 0, sPH = 0;
uint16_t sEC = 0, sN = 0, sP = 0, sK = 0;
bool  sueloOK = false;
bool  gasActivo = false;           // alarma de gases enclavada (con histeresis)
unsigned long tGasUltimo = 0;      // ultima vez que un MQ supero el umbral

// ---------- COMUNICACION ----------
unsigned long tPantalla = 0, tPublica = 0;
uint32_t seq = 0;
int pantalla = 0;
const uint8_t LINEA_MAX = 64;
char lineaUSB[LINEA_MAX]; uint8_t lenUSB = 0;
char lineaESP[LINEA_MAX]; uint8_t lenESP = 0;

// Memoria de ACKs recientes para no repetir acciones (idempotencia)
const uint8_t N_ACKS = 16;
const uint8_t ECO_MAX = 33;      // "cmd" del ACK: maximo 32 caracteres
struct AckGuardado { uint32_t id; uint8_t err; char eco[ECO_MAX]; };
AckGuardado acks[N_ACKS];
uint8_t acksPos = 0, acksN = 0;

enum { E_OK, E_NO_WATER, E_INTERLOCK_CO, E_BAD_CMD, E_BAD_VAL };
const char* const NOMBRE_ERR[] = { "", "NO_WATER", "INTERLOCK_CO", "BAD_CMD", "BAD_VAL" };

// ============================================================
void setup() {
  // Reles en HIGH (apagados) antes de pasarlos a salida: evita un pulso al arrancar
  digitalWrite(RELE_BOMBA, HIGH); digitalWrite(RELE_VENT, HIGH);
  pinMode(RELE_BOMBA, OUTPUT);    pinMode(RELE_VENT, OUTPUT);

  Serial.begin(9600);     // Monitor Serial
  Serial1.begin(9600);    // ESP32 (via TXS0108E)
  #if USA_7EN1
    Serial2.begin(9600);
    digitalWrite(17, HIGH);          // pull-up en RX2: RO del MAX485 queda flotando al transmitir
    pinMode(RS485_DIR, OUTPUT);
    digitalWrite(RS485_DIR, LOW);
  #endif
  dht.begin();
  Wire.begin();
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(25000, true);   // si el ruido del motor traba el I2C, no se cuelga el Mega
  #endif
  lcd.init(); lcd.backlight();
  #if USA_BH1750
    luxMeter.begin();
  #endif
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_NIVEL, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT); pinMode(PIN_LUZ, OUTPUT);
  servoVentana.attach(PIN_SRV_VENT); servoPuerta.attach(PIN_SRV_PUER);

  for (uint8_t i = 0; i < N_ACT; i++) { valAuto[i] = 0; valMan[i] = 0; manOn[i] = false; salida[i] = 0; }
  aplicarSalidas();
  #if USA_RFID
    rfidIniciar();
  #endif

  lcd.setCursor(0,0); lcd.print("Invernadero v3");
  lcd.setCursor(0,1); lcd.print("iniciando...");
  Serial.println(F("Invernadero v3 listo. Ejemplo: #1 BOMBA ON 60"));
  delay(1200); lcd.clear();
}

void loop() {
  leerSensores();
  aplicarLogica();          // calcula valAuto[]
  vencerManuales();         // TTL
  #if USA_RFID
    rfidAtender();          // lee tarjetas, abre la puerta, reporta
  #endif
  combinarYProteger();      // salida[] = manual o auto, + protecciones
  aplicarSalidas();
  atenderSerial();
  if (millis() - tPantalla > 2500) { tPantalla = millis(); mostrarLCD(); }
  if (millis() - tPublica  > 2000) { tPublica  = millis(); publicarJSON(); }
}

// ---------- LECTURA ----------
void leerSensores() {
  static unsigned long tDHT = 0, t7 = 0;

  if (millis() - tDHT > 2000) {
    tDHT = millis();
    float tt = dht.readTemperature();
    float hh = dht.readHumidity();
    if (isnan(tt) || isnan(hh)) {
      if (dhtFallas < 255) dhtFallas++;
    } else {
      t = tt; h = hh; dhtFallas = 0;
    }
    dhtOK = (dhtFallas < DHT_FALLAS_MAX);
  }

  #if USA_7EN1
    if (t7 == 0 || millis() - t7 > (sueloOK ? 2000UL : T_7EN1_FALLA_MS)) { leer7en1(); t7 = millis(); }
  #else
    int crudo = analogRead(PIN_SUELO);
    humSuelo = constrain(map(crudo, SUELO_SECO, SUELO_AGUA, 0, 100), 0, 100);
    sueloOK = true;
  #endif

  hayAgua   = (digitalRead(PIN_NIVEL) == NIVEL_HAY_AGUA);
  aire      = analogRead(PIN_MQ135);
  #if USA_MQ7
    co      = analogRead(PIN_MQ7);
  #else
    co      = 0;
  #endif
  presencia = (digitalRead(PIN_PIR) == HIGH);
  #if USA_BH1750
    lux = luxMeter.readLightLevel();   // -1 / -2 si falla
  #endif
}

// ---------- LOGICA AUTOMATICA ----------
bool gasListo() { return millis() > GAS_PRECAL_MS; }

void aplicarLogica() {
  // Riego con histeresis. Sin lectura valida de suelo no riega.
  if (!sueloOK || !hayAgua)                valAuto[A_BOMBA] = 0;
  else if (humSuelo < UMBRAL_RIEGO)        valAuto[A_BOMBA] = 1;
  else if (humSuelo > UMBRAL_RIEGO_OFF)    valAuto[A_BOMBA] = 0;

  // Ventilacion por calor o humedad del aire, con histeresis.
  // Sin lectura valida del DHT no ventila por clima.
  static bool ventilarClima = false;
  if (!dhtOK) ventilarClima = false;
  else if (t > UMBRAL_TEMP || h > UMBRAL_HUM) ventilarClima = true;
  else if (t < UMBRAL_TEMP - HIST_TEMP && h < UMBRAL_HUM - HIST_HUM) ventilarClima = false;

  // Gases: enciende arriba del umbral; apaga solo si ambos bajan HIST_GAS
  // puntos y paso el tiempo minimo. Ignora el primer minuto (precalentamiento).
  bool sobre = (aire > UMBRAL_MQ135) || (co > UMBRAL_MQ7);
  bool bajo  = (aire < UMBRAL_MQ135 - HIST_GAS) && (co < UMBRAL_MQ7 - HIST_GAS);
  if (!gasListo()) {
    gasActivo = false;
  } else if (sobre) {
    gasActivo = true; tGasUltimo = millis();
  } else if (gasActivo && bajo && millis() - tGasUltimo > GAS_MIN_ON_MS) {
    gasActivo = false;
  }

  bool ventilar = ventilarClima || gasActivo;
  valAuto[A_VENT]    = ventilar ? 1 : 0;
  valAuto[A_VENTANA] = ventilarClima ? 90 : 0;   // como en v1: la ventana abre por clima, no por gas
  valAuto[A_ALARMA]  = gasActivo ? 1 : 0;

  // Iluminacion PWM proporcional (mas oscuro = mas brillo) con histeresis.
  #if USA_BH1750
    static bool luzOn = false;
    if (lux < 0)                             luzOn = false;  // lectura fallida: no decide
    else if (lux < UMBRAL_LUZ)               luzOn = true;
    else if (lux > UMBRAL_LUZ + HIST_LUZ)    luzOn = false;
    if (luzOn) {
      long l = constrain((long)lux, 0, UMBRAL_LUZ);
      valAuto[A_LUZ] = constrain(map(l, 0, UMBRAL_LUZ, 255, LUZ_PWM_MIN), LUZ_PWM_MIN, 255);
    } else {
      valAuto[A_LUZ] = 0;
    }
  #else
    valAuto[A_LUZ] = 0;
  #endif

  #if PIR_ABRE_PUERTA
    valAuto[A_PUERTA] = presencia ? 90 : 0;
  #else
    valAuto[A_PUERTA] = 0;
  #endif
}

// ---------- MANUAL CON TTL ----------
void vencerManuales() {
  unsigned long ahora = millis();
  for (uint8_t i = 0; i < N_ACT; i++) {
    if (manOn[i] && (long)(ahora - manHasta[i]) >= 0) {
      manOn[i] = false;
      Serial.print(F("Manual vencido -> AUTO ")); Serial.println(NOMBRE_CMD[i]);
    }
  }
}
bool algunManual() {
  for (uint8_t i = 0; i < N_ACT; i++) if (manOn[i]) return true;
  return false;
}
uint16_t segundosRestantes(uint8_t i) {
  if (!manOn[i]) return 0;
  long ms = (long)(manHasta[i] - millis());
  if (ms <= 0) return 0;
  return (uint16_t)((ms + 999) / 1000);
}

// ---------- COMBINAR + PROTECCIONES (siempre al final) ----------
void combinarYProteger() {
  for (uint8_t i = 0; i < N_ACT; i++) salida[i] = manOn[i] ? valMan[i] : valAuto[i];
  if (!hayAgua)  salida[A_BOMBA] = 0;                       // nunca bombear en seco
  if (gasActivo) { salida[A_VENT] = 1; salida[A_ALARMA] = 1; }  // interlock de gases
  #if USA_RFID
    if (puertaRfidActiva()) salida[A_PUERTA] = 90;          // tarjeta valida: abre aunque este en manual
  #endif
}

// ---------- APLICAR A LOS PINES ----------
void aplicarSalidas() {
  digitalWrite(RELE_BOMBA, salida[A_BOMBA] ? LOW : HIGH);   // activo en bajo
  digitalWrite(RELE_VENT,  salida[A_VENT]  ? LOW : HIGH);
  analogWrite(PIN_LED, salida[A_LUZ]);
  servoVentana.write(salida[A_VENTANA]);
  servoPuerta.write(salida[A_PUERTA]);
  bool bip = false;
  #if USA_RFID
    bip = bipActivo();
  #endif
  digitalWrite(PIN_BUZZER, (salida[A_ALARMA] || bip) ? HIGH : LOW);
  digitalWrite(PIN_LUZ,    salida[A_ALARMA] ? HIGH : LOW);
}

// ---------- APTITUD DEL SUELO (por ahora solo pH, Q-23) ----------
const char* aptitudCodigo() {
  #if USA_7EN1
    if (!sueloOK) return "SIN_LECTURA";
    if (sPH >= PH_APTO_MIN && sPH <= PH_APTO_MAX) return "APTO";
    if (sPH >= PH_REG_MIN  && sPH <= PH_REG_MAX)  return "REGULAR";
    return "NO_APTO";
  #else
    return "SIN_LECTURA";
  #endif
}
const char* aptitudTexto() {   // version corta para la LCD
  const char* c = aptitudCodigo();
  if (c[0] == 'A') return "Apto";
  if (c[0] == 'R') return "Regular";
  if (c[0] == 'N') return "No apto";
  return "Sin dato";
}

// ---------- LCD 16x2 ----------
void mostrarLCD() {
  #if defined(WIRE_HAS_TIMEOUT)
    if (Wire.getWireTimeoutFlag()) {     // hubo un bloqueo del I2C: reinicia LCD y sensor de luz
      Wire.clearWireTimeoutFlag();
      lcd.init(); lcd.backlight();
      #if USA_BH1750
        luxMeter.begin();
      #endif
      Serial.println(F("I2C reiniciado (ruido?)"));
    }
  #endif
  lcd.clear();
  #if USA_RFID
    if ((long)(rfidMsgHasta - millis()) > 0) {      // mensaje de acceso por 3 s
      lcd.setCursor(0,0); lcd.print(rfidMsg[0]);
      lcd.setCursor(0,1); lcd.print(rfidMsg[1]);
      return;
    }
  #endif
  int luzPct = map(salida[A_LUZ], 0, 255, 0, 100);
#if USA_7EN1
  const int NS = 6;
  switch (pantalla) {
    case 0:
      lcd.setCursor(0,0); lcd.print("T:"); lcd.print(t,1); lcd.print("C H:"); lcd.print(h,0); lcd.print("%");
      if (!dhtOK) lcd.print("!");
      lcd.setCursor(0,1); lcd.print("Luz:"); if (lux < 0) lcd.print("N/D"); else { lcd.print((int)lux); lcd.print(" lx"); }
      break;
    case 1:
      lcd.setCursor(0,0); lcd.print("Suelo:"); lcd.print(sTemp,1); lcd.print("C");
      lcd.setCursor(0,1); lcd.print("Hum:"); lcd.print(humSuelo,0); lcd.print("%"); if (!sueloOK) lcd.print(" sin dato");
      break;
    case 2:
      lcd.setCursor(0,0); lcd.print("pH:"); lcd.print(sPH,1); lcd.print(" EC:"); lcd.print(sEC);
      lcd.setCursor(0,1); lcd.print("N"); lcd.print(sN); lcd.print(" P"); lcd.print(sP); lcd.print(" K"); lcd.print(sK);
      break;
    case 3:
      lcd.setCursor(0,0); lcd.print("Suelo: "); lcd.print(aptitudTexto());
      lcd.setCursor(0,1); lcd.print(hayAgua ? "Tanque OK" : "Tanque BAJO");
      break;
    case 4:
      lcd.setCursor(0,0); lcd.print("Bom:"); lcd.print(salida[A_BOMBA]?"ON":"OFF"); lcd.print(" Ven:"); lcd.print(salida[A_VENT]?"ON":"OFF");
      lcd.setCursor(0,1); lcd.print("Luz:"); lcd.print(luzPct); lcd.print("% "); lcd.print(algunManual()?"MAN":"AUT");
      break;
    default:
      lcd.setCursor(0,0); lcd.print("Gas:"); lcd.print(aire); lcd.print("/"); lcd.print(co); if (!gasListo()) lcd.print(" prec");
      lcd.setCursor(0,1); lcd.print("PIR:"); lcd.print(presencia?"SI":"NO"); lcd.print(" "); lcd.print(salida[A_ALARMA]?"ALARMA":"OK");
      break;
  }
#else
  const int NS = 4;
  switch (pantalla) {
    case 0:
      lcd.setCursor(0,0); lcd.print("T:"); lcd.print(t,1); lcd.print("C H:"); lcd.print(h,0); lcd.print("%");
      lcd.setCursor(0,1); lcd.print("Luz:"); if (lux < 0) lcd.print("N/D"); else { lcd.print((int)lux); lcd.print("lx"); }
      break;
    case 1:
      lcd.setCursor(0,0); lcd.print("Hum.suelo:"); lcd.print(humSuelo,0); lcd.print("%");
      lcd.setCursor(0,1); lcd.print(hayAgua ? "Tanque OK" : "Tanque BAJO");
      break;
    case 2:
      lcd.setCursor(0,0); lcd.print("Bom:"); lcd.print(salida[A_BOMBA]?"ON":"OFF"); lcd.print(" Ven:"); lcd.print(salida[A_VENT]?"ON":"OFF");
      lcd.setCursor(0,1); lcd.print("Luz:"); lcd.print(luzPct); lcd.print("% "); lcd.print(algunManual()?"MAN":"AUT");
      break;
    default:
      lcd.setCursor(0,0); lcd.print("Gas:"); lcd.print(aire); lcd.print("/"); lcd.print(co); if (!gasListo()) lcd.print(" prec");
      lcd.setCursor(0,1); lcd.print("PIR:"); lcd.print(presencia?"SI":"NO"); lcd.print(" "); lcd.print(salida[A_ALARMA]?"ALARMA":"OK");
      break;
  }
#endif
  pantalla = (pantalla + 1) % NS;
}

// ---------- SALIDA JSON (contrato v1) ----------
// Se arma en un buffer fijo y se manda igual al ESP32 y al Monitor.
char json[512];
size_t jlen;
void jAdd(const char* s) {
  size_t n = strlen(s);
  if (jlen + n < sizeof(json)) { memcpy(json + jlen, s, n); jlen += n; json[jlen] = 0; }
}
void jInt(const char* clave, long v) {
  char tmp[40]; snprintf(tmp, sizeof(tmp), ",\"%s\":%ld", clave, v); jAdd(tmp);
}
void jFloat(const char* clave, float v, uint8_t dec) {
  char num[16]; dtostrf(v, 1, dec, num);
  char* p = num; while (*p == ' ') p++;           // dtostrf rellena con espacios
  char tmp[40]; snprintf(tmp, sizeof(tmp), ",\"%s\":%s", clave, p); jAdd(tmp);
}
void jStr(const char* clave, const char* v) {
  char tmp[48]; snprintf(tmp, sizeof(tmp), ",\"%s\":\"%s\"", clave, v); jAdd(tmp);
}

void publicarJSON() {
  seq++;
  jlen = 0; json[0] = 0;
  { char tmp[24]; snprintf(tmp, sizeof(tmp), "{\"v\":1"); jAdd(tmp); }
  jInt  ("seq",     (long)seq);
  jInt  ("up",      (long)(millis() / 1000UL));
  jFloat("aire_t",  constrain(t, -40, 80), 1);
  jInt  ("aire_h",  (long)constrain(h, 0, 100));
  jInt  ("dht_ok",  dhtOK ? 1 : 0);
  jInt  ("suelo_h", (long)constrain(humSuelo, 0, 100));
  #if USA_7EN1
    jFloat("suelo_t", constrain(sTemp, -40, 80), 1);
    jFloat("ph",      constrain(sPH, 0, 14), 1);
    jInt  ("ec",      sEC);
    jInt  ("n",       sN);
    jInt  ("p",       sP);
    jInt  ("k",       sK);
  #endif
  jInt  ("suelo_ok", sueloOK ? 1 : 0);
  #if USA_7EN1
    jStr("aptitud", aptitudCodigo());
  #endif
  jInt  ("lux",        lux < 0 ? -1L : (long)lux);
  jInt  ("aire_mq135", aire);
  jInt  ("co",         co);
  jInt  ("agua",       hayAgua ? 1 : 0);
  jInt  ("pir",        presencia ? 1 : 0);
  jInt  ("bomba",      salida[A_BOMBA]);
  jInt  ("vent",       salida[A_VENT]);
  jInt  ("luz",        map(salida[A_LUZ], 0, 255, 0, 100));
  jInt  ("ventana",    salida[A_VENTANA]);
  jInt  ("puerta",     salida[A_PUERTA]);
  jInt  ("alarma",     salida[A_ALARMA]);
  // man: solo los actuadores en manual, con los segundos que les quedan
  jAdd(",\"man\":{");
  bool primero = true;
  for (uint8_t i = 0; i < N_ACT; i++) {
    if (!manOn[i]) continue;
    char tmp[24];
    snprintf(tmp, sizeof(tmp), "%s\"%s\":%u", primero ? "" : ",", NOMBRE_JSON[i], segundosRestantes(i));
    jAdd(tmp); primero = false;
  }
  jAdd("}}");

  Serial1.println(json);   // al ESP32
  Serial.println(json);    // al Monitor
}

// ---------- COMANDOS ----------
void atenderSerial() {
  while (Serial.available())  leerChar(Serial.read(),  lineaUSB, lenUSB, Serial);
  while (Serial1.available()) leerChar(Serial1.read(), lineaESP, lenESP, Serial1);
}
void leerChar(char c, char* linea, uint8_t& len, Stream& origen) {
  if (c == '\n' || c == '\r') {
    if (len) { linea[len] = 0; ejecutar(linea, origen); len = 0; }
  } else if (c == '#' || c == '@') {
    // '#' y '@' solo aparecen al inicio de una linea: descarta basura previa sin salto de linea
    // (ruido en la linea del ESP32) para que no se pegue al comando y lo invalide.
    len = 0; linea[len++] = c;
  } else if (len < LINEA_MAX - 1) {
    linea[len++] = c;
  } else {
    len = 0;   // linea demasiado larga: se descarta
  }
}

// Convierte un texto a entero (0..tope); false si no son solo digitos o se pasa.
bool aNumero(const char* s, uint32_t tope, uint32_t& out) {
  if (!s || !*s || strlen(s) > 10) return false;
  uint32_t v = 0;
  for (const char* p = s; *p; p++) {
    if (*p < '0' || *p > '9') return false;
    uint8_t d = *p - '0';
    if (v > (tope - d) / 10) return false;      // evita desbordar
    v = v * 10 + d;
  }
  out = v; return true;
}
int8_t buscarActuador(const char* nombre) {
  for (uint8_t i = 0; i < N_ACT; i++) if (strcmp(nombre, NOMBRE_CMD[i]) == 0) return i;
  return -1;
}
int8_t buscarAck(uint32_t id) {
  for (uint8_t i = 0; i < acksN; i++) if (acks[i].id == id) return i;
  return -1;
}
void guardarAck(uint32_t id, uint8_t err, const char* eco) {
  AckGuardado& a = acks[acksPos];
  a.id = id; a.err = err;
  strncpy(a.eco, eco, sizeof(a.eco)); a.eco[sizeof(a.eco) - 1] = 0;
  acksPos = (acksPos + 1) % N_ACKS;
  if (acksN < N_ACKS) acksN++;
}
// ACK al origen. Con ok:1 agrega "cmd" con lo que el Mega entendio y va a ejecutar.
void responder(Stream& origen, bool conId, uint32_t id, uint8_t err, const char* eco, const char* recibido) {
  if (conId) {
    char tmp[80];
    if (err == E_OK) {
      if (ACK_CON_CMD) snprintf(tmp, sizeof(tmp), "{\"ack\":%lu,\"ok\":1,\"cmd\":\"%s\"}", (unsigned long)id, eco);
      else             snprintf(tmp, sizeof(tmp), "{\"ack\":%lu,\"ok\":1}", (unsigned long)id);
    } else {
      snprintf(tmp, sizeof(tmp), "{\"ack\":%lu,\"ok\":0,\"err\":\"%s\"}", (unsigned long)id, NOMBRE_ERR[err]);
    }
    origen.println(tmp);
    if (&origen != &Serial) { Serial.print(F("ESP32 -> ")); Serial.print(recibido); Serial.print(F(" | ")); Serial.println(tmp); }
  } else {
    if (err == E_OK) { Serial.print(F("OK -> ")); Serial.println(eco); }
    else { Serial.print(F("ERROR ")); Serial.print(NOMBRE_ERR[err]); Serial.print(F(" -> ")); Serial.println(recibido); }
  }
}

void ejecutar(char* linea, Stream& origen) {
  #if USA_RFID
    if (linea[0] == '@') { rfidLinea(linea, origen); return; }   // lista de tarjetas
  #endif
  // Mayusculas
  for (char* p = linea; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;
  char recibido[LINEA_MAX]; strncpy(recibido, linea, LINEA_MAX); recibido[LINEA_MAX - 1] = 0;

  char* tok[5]; uint8_t n = 0;
  for (char* p = strtok(linea, " \t"); p && n < 5; p = strtok(NULL, " \t")) tok[n++] = p;
  if (n == 0) return;
  // Del ESP32 solo se aceptan comandos con #id: ignora ruido o eco del propio JSON
  if (&origen != &Serial && tok[0][0] != '#') return;

  bool conId = false; uint32_t id = 0; uint8_t k = 0;
  if (tok[0][0] == '#') {
    if (!aNumero(tok[0] + 1, 4294967295UL, id) || id == 0) return;   // id invalido: sin respuesta posible
    conId = true; k = 1;
    int8_t previo = buscarAck(id);
    if (previo >= 0) {   // reintento de la nube: mismo ACK, sin repetir la accion
      responder(origen, true, id, acks[previo].err, acks[previo].eco, recibido);
      return;
    }
  }
  char eco[ECO_MAX] = "";
  uint8_t err = procesar(tok + k, n - k, eco);
  if (conId) guardarAck(id, err, eco);
  responder(origen, conId, id, err, eco, recibido);
}

// Devuelve E_OK o el codigo de error. Solo aplica cambios si es E_OK.
// En eco deja el comando armado con los valores ya interpretados
// (ej. "VENTANA 90 600"); la nube lo compara con lo que envio.
uint8_t procesar(char** tok, uint8_t n, char* eco) {
  if (n == 0) return E_BAD_CMD;

  if (strcmp(tok[0], "AUTO") == 0) {
    if (n == 1) {
      for (uint8_t i = 0; i < N_ACT; i++) manOn[i] = false;
      snprintf(eco, ECO_MAX, "AUTO");
      return E_OK;
    }
    if (n != 2) return E_BAD_CMD;
    int8_t a = buscarActuador(tok[1]);
    if (a < 0) return E_BAD_CMD;
    manOn[a] = false;
    snprintf(eco, ECO_MAX, "AUTO %s", NOMBRE_CMD[a]);
    return E_OK;
  }

  int8_t a = buscarActuador(tok[0]);
  if (a < 0 || n < 2 || n > 3) return E_BAD_CMD;

  int  valor;            // valor interno (luz en PWM 0-255)
  char arg[8];           // como se escribe en el eco: ON, OFF o el numero
  bool esSwitch = (strcmp(tok[1], "ON") == 0 || strcmp(tok[1], "OFF") == 0);
  if (a == A_BOMBA || a == A_VENT || a == A_ALARMA) {
    if (!esSwitch) return E_BAD_CMD;
    valor = (tok[1][1] == 'N') ? 1 : 0;
    strcpy(arg, valor ? "ON" : "OFF");
  } else if (a == A_LUZ && esSwitch) {          // LUZ ON = 100 %, LUZ OFF = 0 %
    valor = (tok[1][1] == 'N') ? 255 : 0;
    strcpy(arg, valor ? "ON" : "OFF");
  } else {
    uint32_t v;
    uint32_t tope = (a == A_LUZ) ? 100 : 90;
    if (!aNumero(tok[1], tope, v)) return E_BAD_VAL;
    valor = (a == A_LUZ) ? (int)map(v, 0, 100, 0, 255) : (int)v;
    snprintf(arg, sizeof(arg), "%lu", (unsigned long)v);
  }

  uint32_t ttl = TTL_DEFAULT_S;
  if (n == 3) {
    if (!aNumero(tok[2], TTL_MAX_S, ttl) || ttl < 1) return E_BAD_VAL;
  }

  // Interlocks: se rechaza lo que las protecciones anularian de todos modos,
  // para que la web nunca muestre un estado que el Mega no va a cumplir.
  if (a == A_BOMBA && valor == 1 && !hayAgua)  return E_NO_WATER;
  if (a == A_VENT   && valor == 0 && gasActivo) return E_INTERLOCK_CO;
  if (a == A_ALARMA && valor == 0 && gasActivo) return E_INTERLOCK_CO;

  valMan[a]   = valor;
  manOn[a]    = true;
  manHasta[a] = millis() + ttl * 1000UL;
  snprintf(eco, ECO_MAX, "%s %s %lu", NOMBRE_CMD[a], arg, (unsigned long)ttl);
  return E_OK;
}

// ---------- SENSOR 7-EN-1 (RS485 / Modbus) ----------
//   reg0 = temperatura /10   reg1 = humedad /10   reg2 = EC (us/cm)
//   reg3 = pH /10            reg4 = N   reg5 = P   reg6 = K  (mg/kg)
// Lectura propia en vez de ModbusMaster: el MAX485 mete un byte basura
// (FC, FE, F8...) antes de la respuesta, y ModbusMaster la descartaba
// completa por "direccion invalida". Aqui se busca la trama valida
// (02 03 0E + 14 datos + CRC) dentro de lo recibido.
#if USA_7EN1
uint16_t crcModbus(const uint8_t* b, uint8_t n) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < n; i++) {
    crc ^= b[i];
    for (uint8_t k = 0; k < 8; k++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}
void leer7en1() {
  uint8_t req[8] = { SUELO_ADDR, 0x03, 0x00, 0x00, 0x00, 0x07, 0, 0 };
  uint16_t c = crcModbus(req, 6);
  req[6] = c & 0xFF; req[7] = c >> 8;

  while (Serial2.available()) Serial2.read();      // limpia restos
  digitalWrite(RS485_DIR, HIGH);                    // transmitir
  delayMicroseconds(50);
  Serial2.write(req, 8);
  Serial2.flush();                                  // espera a que salga el ultimo bit
  delayMicroseconds(50);
  digitalWrite(RS485_DIR, LOW);                     // recibir

  const uint8_t LARGO = 19;                         // 3 cabecera + 14 datos + 2 CRC
  uint8_t buf[40]; uint8_t n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < SUELO_TIMEOUT_MS) {
    while (Serial2.available() && n < sizeof(buf)) buf[n++] = Serial2.read();
    for (uint8_t s = 0; s + LARGO <= n; s++) {      // salta bytes basura al inicio
      const uint8_t* f = buf + s;
      if (f[0] != SUELO_ADDR || f[1] != 0x03 || f[2] != 14) continue;
      if (crcModbus(f, LARGO - 2) != (uint16_t)(f[LARGO - 2] | (f[LARGO - 1] << 8))) continue;
      uint16_t r[7];
      for (uint8_t i = 0; i < 7; i++) r[i] = (f[3 + 2 * i] << 8) | f[4 + 2 * i];
      sTemp    = r[0] / 10.0;
      humSuelo = r[1] / 10.0;
      sEC      = r[2];
      sPH      = r[3] / 10.0;
      sN       = r[4];
      sP       = r[5];
      sK       = r[6];
      sueloOK  = true;
      return;
    }
    if (n >= sizeof(buf)) break;
  }
  sueloOK = false;                                  // conserva el ultimo valor valido
}
#endif


// ============================================================
//                    RFID RC522 (v3)
// ============================================================
#if USA_RFID
// Tarjetas del equipo: se usan mientras la EEPROM no tenga una lista de la nube.
const uint8_t UID_FABRICA[][7] = {
  { 0x53, 0xD8, 0x01, 0x51, 0x22, 0x00, 0x01 },   // Wilson
  { 0x53, 0xC7, 0x37, 0x51, 0x22, 0x00, 0x01 },   // William
  { 0x53, 0x03, 0xF2, 0x51, 0x22, 0x00, 0x01 },   // Axel
  { 0x53, 0xE0, 0x0A, 0x51, 0x22, 0x00, 0x01 },   // Luis
};
const char* const NOMBRE_FABRICA[] = { "Wilson", "William", "Axel", "Luis" };

// EEPROM: [0..1] 'R''F'  [2] formato  [3..6] version  [7] cantidad  [8..] tarjetas
#define EE_BASE     0
#define EE_FORMATO  1
#define EE_LISTA    (EE_BASE + 8)

void rfidGuardar() {
  EEPROM.update(EE_BASE, 'R'); EEPROM.update(EE_BASE + 1, 'F');
  EEPROM.update(EE_BASE + 2, EE_FORMATO);
  EEPROM.put(EE_BASE + 3, verTarjetas);
  EEPROM.update(EE_BASE + 7, nTarjetas);
  for (uint8_t i = 0; i < nTarjetas; i++) EEPROM.put(EE_LISTA + i * sizeof(Tarjeta), tarjetas[i]);
}

void rfidCargar() {
  uint8_t n = EEPROM.read(EE_BASE + 7);
  if (EEPROM.read(EE_BASE) == 'R' && EEPROM.read(EE_BASE + 1) == 'F' &&
      EEPROM.read(EE_BASE + 2) == EE_FORMATO && n <= RFID_MAX) {
    EEPROM.get(EE_BASE + 3, verTarjetas);
    nTarjetas = n;
    for (uint8_t i = 0; i < n; i++) {
      EEPROM.get(EE_LISTA + i * sizeof(Tarjeta), tarjetas[i]);
      if (tarjetas[i].len > RFID_UID_MAX) tarjetas[i].len = 0;      // dato corrupto: no coincide con nada
      tarjetas[i].nombre[RFID_NOMBRE - 1] = 0;
    }
    return;
  }
  // EEPROM vacia o de otro programa: lista de fabrica
  verTarjetas = 0;
  nTarjetas = sizeof(UID_FABRICA) / sizeof(UID_FABRICA[0]);
  for (uint8_t i = 0; i < nTarjetas; i++) {
    tarjetas[i].len = 7;
    memcpy(tarjetas[i].uid, UID_FABRICA[i], 7);
    strncpy(tarjetas[i].nombre, NOMBRE_FABRICA[i], RFID_NOMBRE - 1);
    tarjetas[i].nombre[RFID_NOMBRE - 1] = 0;
  }
}

void rfidIniciar() {
  rfidCargar();
  SPI.begin();
  rfid.PCD_Init();
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print(F("RFID: lector 0x")); Serial.print(v, HEX);
  Serial.print(v == 0x00 || v == 0xFF ? F(" NO RESPONDE") : F(" OK"));
  Serial.print(F(" | lista v")); Serial.print(verTarjetas);
  Serial.print(F(", ")); Serial.print(nTarjetas); Serial.println(F(" tarjetas"));
}

// "53D80151220001" (mayusculas, sin espacios)
void uidATexto(const uint8_t* uid, uint8_t len, char* out) {
  const char HEX_[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < len; i++) { out[2 * i] = HEX_[uid[i] >> 4]; out[2 * i + 1] = HEX_[uid[i] & 0x0F]; }
  out[2 * len] = 0;
}

// Texto hex (8, 14 o 20 digitos) -> bytes. false si no es valido.
bool textoAUid(const char* txt, Tarjeta& t) {
  uint8_t n = strlen(txt);
  if (n != 8 && n != 14 && n != 20) return false;
  for (uint8_t i = 0; i < n; i++) {
    char c = txt[i]; uint8_t d;
    if (c >= '0' && c <= '9') d = c - '0';
    else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
    else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
    else return false;
    if (i % 2 == 0) t.uid[i / 2] = d << 4; else t.uid[i / 2] |= d;
  }
  t.len = n / 2;
  return true;
}

int8_t rfidBuscar(const uint8_t* uid, uint8_t len) {
  for (uint8_t i = 0; i < nTarjetas; i++)
    if (tarjetas[i].len == len && memcmp(tarjetas[i].uid, uid, len) == 0) return i;
  return -1;
}

bool puertaRfidActiva() {
  if (puertaRfid && (long)(millis() - puertaRfidHasta) >= 0) puertaRfid = false;
  return puertaRfid;
}

// Permitido: 1 bip de 150 ms. Denegado: 3 bips de 100 ms.
bool bipActivo() {
  if (!bipEncendido) return false;
  unsigned long e = millis() - bipInicio;
  if (bipPermitido) { if (e < 150) return true; }
  else if (e < 500) return (e / 100) % 2 == 0;
  bipEncendido = false;
  return false;
}

void rfidMensaje(const char* l1, const char* l2) {
  snprintf(rfidMsg[0], sizeof(rfidMsg[0]), "%s", l1);
  snprintf(rfidMsg[1], sizeof(rfidMsg[1]), "%s", l2);
  rfidMsgHasta = millis() + 3000;
  tPantalla = millis();
  mostrarLCD();
}

void rfidAtender() {
  static unsigned long tLeer = 0, tRevisar = 0, tUltima = 0;
  static uint8_t ultima[RFID_UID_MAX]; static uint8_t ultimaLen = 0;

  // Lista a medio llegar por mas de 10 s: se descarta
  if (sincronizando && millis() - tSync > 10000) { sincronizando = false; Serial.println(F("RFID: lista incompleta, descartada")); }

  // El ruido del motor puede "dormir" al RC522: cada 30 s se revisa y se reinicia
  if (millis() - tRevisar > 30000) {
    tRevisar = millis();
    byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
    if (v == 0x00 || v == 0xFF) { rfid.PCD_Init(); Serial.println(F("RFID: lector reiniciado")); }
  }

  if (millis() - tLeer < 150) return;
  tLeer = millis();
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

  uint8_t len = rfid.uid.size;
  if (len > RFID_UID_MAX) len = RFID_UID_MAX;
  bool repetida = (len == ultimaLen && memcmp(ultima, rfid.uid.uidByte, len) == 0 && millis() - tUltima < 3000);
  memcpy(ultima, rfid.uid.uidByte, len); ultimaLen = len; tUltima = millis();
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  if (repetida) return;                       // la misma tarjeta sostenida frente al lector

  char uidTxt[2 * RFID_UID_MAX + 1];
  uidATexto(ultima, len, uidTxt);
  int8_t i = rfidBuscar(ultima, len);
  bool ok = (i >= 0);

  if (ok) {
    puertaRfid = true;
    puertaRfidHasta = millis() + PUERTA_RFID_MS;
    char l1[LCD_COLS + 1]; snprintf(l1, sizeof(l1), "Hola %s", tarjetas[i].nombre);
    rfidMensaje(l1, "Puerta abierta");
  } else {
    char l2[LCD_COLS + 1]; snprintf(l2, sizeof(l2), "%s", uidTxt);   // se corta a 16
    rfidMensaje("Acceso denegado", l2);
  }
  bipPermitido = ok; bipInicio = millis(); bipEncendido = true;

  char ev[48];
  snprintf(ev, sizeof(ev), "{\"rfid\":\"%s\",\"ok\":%d}", uidTxt, ok ? 1 : 0);
  Serial1.println(ev);                        // al ESP32 -> registro en la nube
  Serial.print(F("RFID ")); Serial.print(ok ? F("PERMITIDO ") : F("DENEGADO "));
  if (ok) { Serial.print(tarjetas[i].nombre); Serial.print(' '); }
  Serial.println(ev);
}

void rfidResponder(Stream& origen, uint32_t ver, bool ok) {
  char tmp[48];
  if (ok) snprintf(tmp, sizeof(tmp), "{\"cards\":%lu,\"ok\":1,\"n\":%u}", (unsigned long)ver, nTarjetas);
  else    snprintf(tmp, sizeof(tmp), "{\"cards\":%lu,\"ok\":0}", (unsigned long)ver);
  origen.println(tmp);
  if (&origen != &Serial) { Serial.print(F("RFID lista -> ")); Serial.println(tmp); }
}

// Lineas que empiezan con '@' (lista de tarjetas). Ver el encabezado del archivo.
void rfidLinea(char* linea, Stream& origen) {
  char* tok[4]; uint8_t n = 0;
  for (char* p = strtok(linea + 1, " \t"); p && n < 4; p = strtok(NULL, " \t")) tok[n++] = p;
  if (n == 0) return;
  for (char* p = tok[0]; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;
  uint32_t v;

  if (strcmp(tok[0], "CARDS?") == 0) { rfidResponder(origen, verTarjetas, true); return; }

  if (strcmp(tok[0], "LIST") == 0) {
    Serial.print(F("Lista v")); Serial.print(verTarjetas); Serial.print(F(": ")); Serial.println(nTarjetas);
    for (uint8_t i = 0; i < nTarjetas; i++) {
      char u[2 * RFID_UID_MAX + 1]; uidATexto(tarjetas[i].uid, tarjetas[i].len, u);
      Serial.print(F("  ")); Serial.print(u); Serial.print(' '); Serial.println(tarjetas[i].nombre);
    }
    return;
  }

  if (strcmp(tok[0], "CARDS") == 0 && n == 3) {
    uint32_t cant;
    if (!aNumero(tok[1], 4294967295UL, v) || !aNumero(tok[2], RFID_MAX, cant)) { rfidResponder(origen, 0, false); return; }
    sincronizando = true; verNueva = v; esperadas = cant; nNuevas = 0; tSync = millis();
    return;
  }

  if (strcmp(tok[0], "CARD") == 0 && n >= 2) {
    if (!sincronizando || nNuevas >= esperadas) return;
    Tarjeta& t = nuevas[nNuevas];
    if (!textoAUid(tok[1], t)) { sincronizando = false; rfidResponder(origen, verNueva, false); return; }
    snprintf(t.nombre, RFID_NOMBRE, "%s", n >= 3 ? tok[2] : "");
    nNuevas++;
    return;
  }

  if (strcmp(tok[0], "END") == 0 && n == 2) {
    bool ok = sincronizando && aNumero(tok[1], 4294967295UL, v) && v == verNueva && nNuevas == esperadas;
    sincronizando = false;
    if (ok) {
      memcpy(tarjetas, nuevas, sizeof(Tarjeta) * nNuevas);
      nTarjetas = nNuevas; verTarjetas = verNueva;
      rfidGuardar();
      Serial.print(F("RFID: lista v")); Serial.print(verTarjetas); Serial.print(F(" guardada, ")); Serial.print(nTarjetas); Serial.println(F(" tarjetas"));
    }
    rfidResponder(origen, ok ? verTarjetas : (aNumero(tok[1], 4294967295UL, v) ? v : 0), ok);
    return;
  }
}
#endif