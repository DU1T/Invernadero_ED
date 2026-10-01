#include <Wire.h>
#include <LiquidCrystal_I2C.h>
// Usa la direccion del escaner (0x27 o 0x3F). Para 16x2 cambia a lcd(0x27,16,2)
LiquidCrystal_I2C lcd(0x27, 20, 4);
void setup() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Invernadero OK");
  lcd.setCursor(0, 1); lcd.print("LCD funcionando");
}
void loop() {}