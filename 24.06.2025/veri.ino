#include <ESP32Servo.h>

#define maxdeger 1940
#define mindeger 1060

float hizBoleni = 1.0;

Servo onsagalt, onsolalt, onsolust, onsagust, arsagalt, arsagust, arsolalt, arsolust;

int onsagalt_deger, onsolalt_deger, arsagalt_deger, arsolalt_deger;
int onsagust_deger, onsolust_deger, arsagust_deger, arsolust_deger;

// Gelen Python verisini ayrıştırmak için fonksiyon prototipi
int parseVeri(String data, String key);

void setup() {
  Serial.begin(9600);
  Serial.println("AKINTAY MANUEL SÜRÜŞ MODU AKTİF");
  Serial.println("............Başlıyor..............");

  // Servo pinlerini kendi bağlantına göre ayarla
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
}

void loop() {
  mod1();  // Sadece mod1 aktif, Python verisi entegre
}

void mod1() {
  float X1, Y1, X2, Y2;

  // Seri porttan veri var mı kontrol et
  if (Serial.available()) {
    String veri = Serial.readStringUntil('\n');  // Satır sonuna kadar oku
    // Örnek veri formatı: X1:1500,Y1:1600,X2:1490,Y2:1400
    X1 = parseVeri(veri, "X1");
    Y1 = parseVeri(veri, "Y1");
    X2 = parseVeri(veri, "X2");
    Y2 = parseVeri(veri, "Y2");
  } else {
    // Veri yoksa analog joystickten oku
    X1 = analogRead(A0) / 4.0 + 1000;
    Y1 = analogRead(A2) / 4.0 + 1000;
    X2 = analogRead(A1) / 4.0 + 1000;
    Y2 = analogRead(A3) / 4.0 + 1000;
  }

  // Hız bölme oranı uygulanıyor
  X1 = 1500 + (X1 - 1500) / hizBoleni;
  Y1 = 1500 + (Y1 - 1500) / hizBoleni;
  X2 = 1500 + (X2 - 1500) / hizBoleni;
  Y2 = 1500 + (Y2 - 1500) / hizBoleni;

  // PWM sınırlandırması
  X1 = constrain(X1, mindeger, maxdeger);
  Y1 = constrain(Y1, mindeger, maxdeger);
  X2 = constrain(X2, mindeger, maxdeger);
  Y2 = constrain(Y2, mindeger, maxdeger);

  // Seri porttan okunan joystick verileri
  Serial.print("X1 "); Serial.print(X1);
  Serial.print("  Y1 "); Serial.print(Y1);
  Serial.print("  X2 "); Serial.print(X2);
  Serial.print("  Y2 "); Serial.println(Y2);

  // Motor pwm hesaplamaları
  onsagust_deger = 1500 - (X1 - 1500) - (X2 - 1500) + (Y2 - 1500) + (Y1 - 1500);
  onsolust_deger = 1500 + (X1 - 1500) + (X2 - 1500) + (Y2 - 1500) + (Y1 - 1500);
  arsolust_deger = 1500 - (X1 - 1500) + (X2 - 1500) - (Y2 - 1500) + (Y1 - 1500);
  arsagust_deger = 1500 + (X1 - 1500) - (X2 - 1500) - (Y2 - 1500) + (Y1 - 1500);
  onsagalt_deger = 1500 - (X1 - 1500) - (X2 - 1500) + (Y2 - 1500) - (Y1 - 1500);
  onsolalt_deger = 1500 + (X1 - 1500) + (X2 - 1500) + (Y2 - 1500) - (Y1 - 1500);
  arsolalt_deger = 1500 - (X1 - 1500) + (X2 - 1500) - (Y2 - 1500) - (Y1 - 1500);
  arsagalt_deger = 1500 + (X1 - 1500) - (X2 - 1500) - (Y2 - 1500) - (Y1 - 1500);

  // Motorlara PWM gönder
  onsagalt.writeMicroseconds(onsagalt_deger);
  onsolalt.writeMicroseconds(onsolalt_deger);
  onsolust.writeMicroseconds(onsolust_deger);
  onsagust.writeMicroseconds(onsagust_deger);
  arsagalt.writeMicroseconds(arsagalt_deger);
  arsagust.writeMicroseconds(arsagust_deger);
  arsolalt.writeMicroseconds(arsolalt_deger);
  arsolust.writeMicroseconds(arsolust_deger);

  // Motor değerlerini seri monitöre yaz
  Serial.print("M1: "); Serial.print(onsagust_deger); Serial.print("  ");
  Serial.print("M2: "); Serial.print(onsolust_deger); Serial.print("  ");
  Serial.print("M3: "); Serial.print(arsolust_deger); Serial.print("  ");
  Serial.print("M4: "); Serial.print(arsagust_deger); Serial.print("  ");
  Serial.print("M5: "); Serial.print(onsagalt_deger); Serial.print("  ");
  Serial.print("M6: "); Serial.print(onsolalt_deger); Serial.print("  ");
  Serial.print("M7: "); Serial.print(arsolalt_deger); Serial.print("  ");
  Serial.print("M8: "); Serial.println(arsagalt_deger);
}

// Gelen stringten ilgili key'in değerini ayıklayan fonksiyon
int parseVeri(String data, String key) {
  int index = data.indexOf(key + ":");
  if (index == -1) return 1500;  // Bulunamazsa varsayılan 1500 döner

  int endIndex = data.indexOf(",", index);
  if (endIndex == -1) endIndex = data.length();

  String value = data.substring(index + key.length() + 1, endIndex);
  return value.toInt();
}

