#include <ESP32Servo.h>

#define MAX_DEGER 1940
#define MIN_DEGER 1060

Servo onsagalt, onsolalt, onsolust, onsagust;
Servo arsagalt, arsagust, arsolalt, arsolust;

int onsagalt_deger, onsolalt_deger, arsagalt_deger, arsolalt_deger;
int onsagust_deger, onsolust_deger, arsagust_deger, arsolust_deger;

// Joystick değerleri
int X1 = 1500, Y1 = 1500, X2 = 1500, Y2 = 1500;

void setup() {
  Serial.begin(115200);  // Python ile eşleşen baudrate

  onsagust.attach(D1);
  onsolust.attach(D2);
  arsolust.attach(D3);
  arsagust.attach(D4);
  onsagalt.attach(D5);
  onsolalt.attach(D6);
  arsolalt.attach(D7);
  arsagalt.attach(D8);

  pinMode(D9, OUTPUT);
  pinMode(D0, OUTPUT);

  Serial.println("AKINTAY OTONOM MOD BAŞLADI");
}

void loop() {
  seriVeriOku();
  motorHesapla();
  motorUygula();
  geriBildirimGonder();
  delay(50);
}

void seriVeriOku() {
  if (Serial.available()) {
    String veri = Serial.readStringUntil('\n');
    veri.trim();

    int x1Idx = veri.indexOf("X1:");
    int y1Idx = veri.indexOf("Y1:");
    int x2Idx = veri.indexOf("X2:");
    int y2Idx = veri.indexOf("Y2:");

    if (x1Idx != -1 && y1Idx != -1 && x2Idx != -1 && y2Idx != -1) {
      X1 = veri.substring(x1Idx + 3, veri.indexOf(",", x1Idx)).toInt();
      Y1 = veri.substring(y1Idx + 3, veri.indexOf(",", y1Idx)).toInt();
      X2 = veri.substring(x2Idx + 3, veri.indexOf(",", x2Idx)).toInt();
      Y2 = veri.substring(y2Idx + 3).toInt();

      X1 = constrain(X1, MIN_DEGER, MAX_DEGER);
      Y1 = constrain(Y1, MIN_DEGER, MAX_DEGER);
      X2 = constrain(X2, MIN_DEGER, MAX_DEGER);
      Y2 = constrain(Y2, MIN_DEGER, MAX_DEGER);
    }
  }
}

void motorHesapla() {
  onsagust_deger = 1500 - (X1 - 1500) - (X2 - 1500) + (Y2 - 1500) + (Y1 - 1500);
  onsolust_deger = 1500 + (X1 - 1500) + (X2 - 1500) + (Y2 - 1500) + (Y1 - 1500);
  arsolust_deger = 1500 - (X1 - 1500) + (X2 - 1500) - (Y2 - 1500) + (Y1 - 1500);
  arsagust_deger = 1500 + (X1 - 1500) - (X2 - 1500) - (Y2 - 1500) + (Y1 - 1500);
  onsagalt_deger = 1500 - (X1 - 1500) - (X2 - 1500) + (Y2 - 1500) - (Y1 - 1500);
  onsolalt_deger = 1500 + (X1 - 1500) + (X2 - 1500) + (Y2 - 1500) - (Y1 - 1500);
  arsolalt_deger = 1500 - (X1 - 1500) + (X2 - 1500) - (Y2 - 1500) - (Y1 - 1500);
  arsagalt_deger = 1500 + (X1 - 1500) - (X2 - 1500) - (Y2 - 1500) - (Y1 - 1500);

  // Değerleri sınırla
  onsagust_deger = constrain(onsagust_deger, MIN_DEGER, MAX_DEGER);
  onsolust_deger = constrain(onsolust_deger, MIN_DEGER, MAX_DEGER);
  arsolust_deger = constrain(arsolust_deger, MIN_DEGER, MAX_DEGER);
  arsagust_deger = constrain(arsagust_deger, MIN_DEGER, MAX_DEGER);
  onsagalt_deger = constrain(onsagalt_deger, MIN_DEGER, MAX_DEGER);
  onsolalt_deger = constrain(onsolalt_deger, MIN_DEGER, MAX_DEGER);
  arsolalt_deger = constrain(arsolalt_deger, MIN_DEGER, MAX_DEGER);
  arsagalt_deger = constrain(arsagalt_deger, MIN_DEGER, MAX_DEGER);
}

void motorUygula() {
  onsagust.writeMicroseconds(onsagust_deger);
  onsolust.writeMicroseconds(onsolust_deger);
  arsolust.writeMicroseconds(arsolust_deger);
  arsagust.writeMicroseconds(arsagust_deger);
  onsagalt.writeMicroseconds(onsagalt_deger);
  onsolalt.writeMicroseconds(onsolalt_deger);
  arsolalt.writeMicroseconds(arsolalt_deger);
  arsagalt.writeMicroseconds(arsagalt_deger);
}

void geriBildirimGonder() {
  Serial.print("M1:"); Serial.print(onsagust_deger); Serial.print(",");
  Serial.print("M2:"); Serial.print(onsolust_deger); Serial.print(",");
  Serial.print("M3:"); Serial.print(arsolust_deger); Serial.print(",");
  Serial.print("M4:"); Serial.print(arsagust_deger); Serial.print(",");
  Serial.print("M5:"); Serial.print(onsagalt_deger); Serial.print(",");
  Serial.print("M6:"); Serial.print(onsolalt_deger); Serial.print(",");
  Serial.print("M7:"); Serial.print(arsolalt_deger); Serial.print(",");
  Serial.print("M8:"); Serial.println(arsagalt_deger);
}
