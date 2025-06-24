#include <ESP32Servo.h>

#define maxdeger 1940
#define mindeger 1060

float hizBoleni = 1.0;

Servo onsagalt, onsolalt, onsolust, onsagust, arsagalt, arsagust, arsolalt, arsolust;

int onsagalt_deger, onsolalt_deger, arsagalt_deger, arsolalt_deger;
int onsagust_deger, onsolust_deger, arsagust_deger, arsolust_deger;

int parseVeri(String data, String key);

void setup() {
  Serial.begin(115200); // Python ile baudrate uyumlu
  Serial.println("AKINTAY MANUEL SÜRÜŞ MODU AKTİF");
  Serial.println("............Başlıyor..............");

  onsagust.attach(D1);
  onsolust.attach(D2);
  arsolust.attach(D3);
  arsagust.attach(D4);
  onsagalt.attach(D5);
  onsolalt.attach(D6);
  arsolalt.attach(D7);
  arsagalt.attach(D8);
}

void loop() {
  if (Serial.available()) {
    String veri = Serial.readStringUntil('\n');
    float X1 = parseVeri(veri, "x1");
    float Y1 = parseVeri(veri, "y1");
    float X2 = parseVeri(veri, "x2");
    float Y2 = parseVeri(veri, "y2");

    // PWM sınırlandırması ve hız bölme
    X1 = constrain(1500 + (X1 - 1500) / hizBoleni, mindeger, maxdeger);
    Y1 = constrain(1500 + (Y1 - 1500) / hizBoleni, mindeger, maxdeger);
    X2 = constrain(1500 + (X2 - 1500) / hizBoleni, mindeger, maxdeger);
    Y2 = constrain(1500 + (Y2 - 1500) / hizBoleni, mindeger, maxdeger);

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

    // Motor değerlerini seri monitöre yaz (Python için geri bildirim)
    Serial.print("motors:");
    Serial.print(onsagust_deger); Serial.print(",");
    Serial.print(onsolust_deger); Serial.print(",");
    Serial.print(arsolust_deger); Serial.print(",");
    Serial.print(arsagust_deger); Serial.print(",");
    Serial.print(onsagalt_deger); Serial.print(",");
    Serial.print(onsolalt_deger); Serial.print(",");
    Serial.print(arsolalt_deger); Serial.print(",");
    Serial.println(arsagalt_deger);
  }
}

int parseVeri(String data, String key) {
  int index = data.indexOf(key + ":");
  if (index == -1) return 1500;

  int endIndex = data.indexOf(",", index);
  if (endIndex == -1) endIndex = data.length();

  String value = data.substring(index + key.length() + 1, endIndex);
  return value.toInt();
}

