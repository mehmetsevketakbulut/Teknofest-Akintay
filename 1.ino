#include <ESP32Servo.h>

#define maxdeger 1940
#define mindeger 1060

float valueJoyStick_X_1 = 1500;
float valueJoyStick_Y_1 = 1500;
float valueJoyStick_X_2 = 1500;
float valueJoyStick_Y_2 = 1500;
float hizBoleni = 1.0;
int sabitleme_toleransi = 60;

Servo onsagalt,onsolalt,onsolust,onsagust,arsagalt,arsagust,arsolalt,arsolust;

int onsagalt_deger, onsolalt_deger, arsagalt_deger, arsolalt_deger;
int onsagust_deger, onsolust_deger, arsagust_deger, arsolust_deger;

void setup () {
  Serial.begin(9600);
  Serial.println("AKINTAY OTONOM MODU (Python'dan Kontrol)");

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

void loop () {
  if (Serial.available()) {
    String veri = Serial.readStringUntil('\n');
    veri.trim();

    int i1 = veri.indexOf("x1:");
    int i2 = veri.indexOf("x2:");
    int i3 = veri.indexOf("y1:");
    int i4 = veri.indexOf("y2:");

    if (i1 >= 0 && i2 >= 0 && i3 >= 0 && i4 >= 0) {
      valueJoyStick_X_1 = veri.substring(i1 + 3, veri.indexOf(",", i1)).toFloat();
      valueJoyStick_X_2 = veri.substring(i2 + 3, veri.indexOf(",", i2)).toFloat();
      valueJoyStick_Y_1 = veri.substring(i3 + 3, veri.indexOf(",", i3)).toFloat();
      valueJoyStick_Y_2 = veri.substring(i4 + 3).toFloat();
    }
  }

  // Değer sınırlandırmaları
  valueJoyStick_X_1 = constrain(valueJoyStick_X_1, mindeger, maxdeger);
  valueJoyStick_Y_1 = constrain(valueJoyStick_Y_1, mindeger, maxdeger);
  valueJoyStick_X_2 = constrain(valueJoyStick_X_2, mindeger, maxdeger);
  valueJoyStick_Y_2 = constrain(valueJoyStick_Y_2, mindeger, maxdeger);

  // --- Motor Hesaplamaları (senin kodun aynen korunur) ---
  // [Buradan sonra senin orijinal motor değerlerini hesaplarken kullandığın kod aynen devam eder...]
  // onsagalt_deger = ...
  // motorlara writeMicroseconds(...) gönderilir
}