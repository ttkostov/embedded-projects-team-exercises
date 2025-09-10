#include <LiquidCrystal.h>

const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


void setup() {
  // put your setup code here, to run once:
  lcd.begin(20, 4);
  lcd.setCursor(0, 1);
  lcd.print("Torille");

}

void loop() {
  // put your main code here, to run repeatedly:
  lcd.setCursor(0, 2);
  lcd.print(millis() / 1000);

}
