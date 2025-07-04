#include <ESP32Servo.h>

#define maxdeger 1940
#define mindeger 1060

float hizBoleni = 1.0;

Servo onsagalt, onsolalt, onsolust, onsagust, arsagalt, arsagust, arsolalt, arsolust;

// Motor PWM değerleri
int onsagalt_deger, onsolalt_deger, arsagalt_deger, arsolalt_deger;
int onsagust_deger, onsolust_deger, arsagust_deger, arsolust_deger;

// Gelen joystick verileri (Python’dan seri port ile gelecek)
int X1 = 1500;
int Y1 = 1500;
int X2 = 1500;
int Y2 = 1500;

void setup() {
  Serial.begin(115200);  // Hız arttırıldı
  Serial.println("AKINTAY SERİ PORTTAN JOYSTICK VERİSİ BEKLİYOR");

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
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    parseJoystick(input);
    hesaplaVeUygulaMotorPWM();
    motorDegerleriniYaz();
  }
  // Dilersen PID için ayrı zamanlama yapabilirsin
}

// Python’dan gelen joystick verisini ayrıştır (format: "X1,Y1,X2,Y2")
void parseJoystick(String data) {
  int values[4] = {1500,1500,1500,1500}; // Default nötr değerler
  int index = 0;
  int lastPos = 0;
  for (int i=0; i<data.length(); i++) {
    if (data.charAt(i) == ',') {
      if (index < 4) {
        values[index] = data.substring(lastPos, i).toInt();
        index++;
        lastPos = i + 1;
      }
    }
  }
  if (index < 4) {
    values[index] = data.substring(lastPos).toInt();
  }
  X1 = constrain(values[0], mindeger, maxdeger);
  Y1 = constrain(values[1], mindeger, maxdeger);
  X2 = constrain(values[2], mindeger, maxdeger);
  Y2 = constrain(values[3], mindeger, maxdeger);
}

// Joystick verilerine göre motor PWM değerlerini hesapla ve servo motorlara uygula
void hesaplaVeUygulaMotorPWM() {
  // Burada PID entegrasyonunu da yapabilirsin, şimdilik joystick değerlerine göre basit hesap
  onsagust_deger = 1500 - (X1 - 1500) - (X2 - 1500) + (Y2 - 1500) + (Y1 - 1500);
  onsolust_deger = 1500 + (X1 - 1500) + (X2 - 1500) + (Y2 - 1500) + (Y1 - 1500);
  arsolust_deger = 1500 - (X1 - 1500) + (X2 - 1500) - (Y2 - 1500) + (Y1 - 1500);
  arsagust_deger = 1500 + (X1 - 1500) - (X2 - 1500) - (Y2 - 1500) + (Y1 - 1500);
  onsagalt_deger = 1500 - (X1 - 1500) - (X2 - 1500) + (Y2 - 1500) - (Y1 - 1500);
  onsolalt_deger = 1500 + (X1 - 1500) + (X2 - 1500) + (Y2 - 1500) - (Y1 - 1500);
  arsolalt_deger = 1500 - (X1 - 1500) + (X2 - 1500) - (Y2 - 1500) - (Y1 - 1500);
  arsagalt_deger = 1500 + (X1 - 1500) - (X2 - 1500) - (Y2 - 1500) - (Y1 - 1500);

  // Limitleri uygula
  onsagust_deger = constrain(onsagust_deger, mindeger, maxdeger);
  onsolust_deger = constrain(onsolust_deger, mindeger, maxdeger);
  arsolust_deger = constrain(arsolust_deger, mindeger, maxdeger);
  arsagust_deger = constrain(arsagust_deger, mindeger, maxdeger);
  onsagalt_deger = constrain(onsagalt_deger, mindeger, maxdeger);
  onsolalt_deger = constrain(onsolalt_deger, mindeger, maxdeger);
  arsolalt_deger = constrain(arsolalt_deger, mindeger, maxdeger);
  arsagalt_deger = constrain(arsagalt_deger, mindeger, maxdeger);

  // Motorlara yaz
  onsagalt.writeMicroseconds(onsagalt_deger);
  onsolalt.writeMicroseconds(onsolalt_deger);
  onsolust.writeMicroseconds(onsolust_deger);
  onsagust.writeMicroseconds(onsagust_deger);
  arsagalt.writeMicroseconds(arsagalt_deger);
  arsagust.writeMicroseconds(arsagust_deger);
  arsolalt.writeMicroseconds(arsolalt_deger);
  arsolust.writeMicroseconds(arsolust_deger);
}

// Motor PWM değerlerini seri porttan Python’a yaz
void motorDegerleriniYaz() {
  Serial.print("M:");
  Serial.print(onsagust_deger); Serial.print(",");
  Serial.print(onsolust_deger); Serial.print(",");
  Serial.print(arsolust_deger); Serial.print(",");
  Serial.print(arsagust_deger); Serial.print(",");
  Serial.print(onsagalt_deger); Serial.print(",");
  Serial.print(onsolalt_deger); Serial.print(",");
  Serial.print(arsolalt_deger); Serial.print(",");
  Serial.println(arsagalt_deger);
}
