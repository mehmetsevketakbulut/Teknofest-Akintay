#include <ESP32Servo.h>

#define maxdeger 1940
#define mindeger 1060

float valueJoyStick_X_1 = A0;
float valueJoyStick_Y_1 = A2;
float valueJoyStick_X_2 = A1;
float valueJoyStick_Y_2 = A3;
float hizBoleni = 1.0;
int sabitleme_toleransi = 60;

Servo onsagalt, onsolalt, onsolust, onsagust, arsagalt, arsagust, arsolalt, arsolust;
int onsagalt_deger, onsolalt_deger, arsagalt_deger, arsolalt_deger;
int onsagust_deger, onsolust_deger, arsagust_deger, arsolust_deger;

// Butonlar
#define FOTO_BUTON D9
#define VIDEO_BUTON D0
bool buton1Prev = false;
bool buton2Prev = false;

void setup() {
  Serial.begin(115200);
  Serial.println("AKINTAY MANUEL SÜRÜŞ MODU AKTİF");

  // Motor pinleri
  onsagust.attach(D1);
  onsolust.attach(D2);
  arsolust.attach(D3);
  arsagust.attach(D4);
  onsagalt.attach(D5);
  onsolalt.attach(D6);
  arsolalt.attach(D7);
  arsagalt.attach(D8);

  // Buton pinleri
  pinMode(FOTO_BUTON, INPUT_PULLUP);
  pinMode(VIDEO_BUTON, INPUT_PULLUP);
}

void loop() {
  // === FOTO & VİDEO KOMUTLARI ===
  bool buton1 = digitalRead(FOTO_BUTON) == LOW;
  bool buton2 = digitalRead(VIDEO_BUTON) == LOW;

  if (buton1 && !buton1Prev) {
    Serial.println("PHOTO");
  }
  buton1Prev = buton1;

  if (buton2 && !buton2Prev) {
    Serial.println("VIDEO");
  }
  buton2Prev = buton2;

  // === JOYSTICK OKUMA ===
  valueJoyStick_X_1 = analogRead(A0) / 4 + 1000;
  valueJoyStick_Y_1 = analogRead(A2) / 4 + 1000;
  valueJoyStick_X_2 = analogRead(A1) / 4 + 1000;
  valueJoyStick_Y_2 = analogRead(A3) / 4 + 1000;

  valueJoyStick_X_1 = 1500 + (valueJoyStick_X_1 - 1500) / hizBoleni;
  valueJoyStick_Y_1 = 1500 + (valueJoyStick_Y_1 - 1500) / hizBoleni;
  valueJoyStick_X_2 = 1500 + (valueJoyStick_X_2 - 1500) / hizBoleni;
  valueJoyStick_Y_2 = 1500 + (valueJoyStick_Y_2 - 1500) / hizBoleni;

  valueJoyStick_X_1 = constrain(valueJoyStick_X_1, mindeger, maxdeger);
  valueJoyStick_Y_1 = constrain(valueJoyStick_Y_1, mindeger, maxdeger);
  valueJoyStick_X_2 = constrain(valueJoyStick_X_2, mindeger, maxdeger);
  valueJoyStick_Y_2 = constrain(valueJoyStick_Y_2, mindeger, maxdeger);

  onsagust_deger = 1500 - (valueJoyStick_X_1 - 1500) - (valueJoyStick_X_2 - 1500) + (valueJoyStick_Y_2 - 1500) + (valueJoyStick_Y_1 - 1500);
  onsolust_deger = 1500 + (valueJoyStick_X_1 - 1500) + (valueJoyStick_X_2 - 1500) + (valueJoyStick_Y_2 - 1500) + (valueJoyStick_Y_1 - 1500);
  arsolust_deger = 1500 - (valueJoyStick_X_1 - 1500) + (valueJoyStick_X_2 - 1500) - (valueJoyStick_Y_2 - 1500) + (valueJoyStick_Y_1 - 1500);
  arsagust_deger = 1500 + (valueJoyStick_X_1 - 1500) - (valueJoyStick_X_2 - 1500) - (valueJoyStick_Y_2 - 1500) + (valueJoyStick_Y_1 - 1500);
  onsagalt_deger = 1500 - (valueJoyStick_X_1 - 1500) - (valueJoyStick_X_2 - 1500) + (valueJoyStick_Y_2 - 1500) - (valueJoyStick_Y_1 - 1500);
  onsolalt_deger = 1500 + (valueJoyStick_X_1 - 1500) + (valueJoyStick_X_2 - 1500) + (valueJoyStick_Y_2 - 1500) - (valueJoyStick_Y_1 - 1500);
  arsolalt_deger = 1500 - (valueJoyStick_X_1 - 1500) + (valueJoyStick_X_2 - 1500) - (valueJoyStick_Y_2 - 1500) - (valueJoyStick_Y_1 - 1500);
  arsagalt_deger = 1500 + (valueJoyStick_X_1 - 1500) - (valueJoyStick_X_2 - 1500) - (valueJoyStick_Y_2 - 1500) - (valueJoyStick_Y_1 - 1500);

  onsagalt.writeMicroseconds(onsagalt_deger);
  onsolalt.writeMicroseconds(onsolalt_deger);
  onsolust.writeMicroseconds(onsolust_deger);
  onsagust.writeMicroseconds(onsagust_deger);
  arsagalt.writeMicroseconds(arsagalt_deger);
  arsagust.writeMicroseconds(arsagust_deger);
  arsolalt.writeMicroseconds(arsolalt_deger);
  arsolust.writeMicroseconds(arsolust_deger);

  delay(20);
}
