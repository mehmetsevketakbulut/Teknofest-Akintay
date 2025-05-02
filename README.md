HABERLEŞME:
  Deneyap Karttan Pythona veri aktarma, Deneyap kartın seri portu üzerinden yapılır. 
  Örnek çalışmada mesafe sensöründen (ilerde sonar olacak inşallah) gelen veri seri porta yazdırılır. 
  Pythonda deneyap kart, port ve haberleşme hızı bilgileriyle birlikte tamınlanır: deneyap=serial.Serial("COM3",9600) 
  veri=deneyap.readline() ve print(veri) kodlarıyla deneyapın seri portundaki veriler okunup pythona aktarılmış olur.
  Eğer pythondan deneyapa veri göndermek istersek de arduino.write((deger + '\n').encode()) kodunu kullanıyoruz.
  Eğer gelen verinin boş olup olmadığını kontrol etmek istersek if (new_data.length() > 0) sorgulaması yapıyoruz.

PYTHONDAN DENEYAPA VERİ GÖNDERİMİ ÖRNEK KOD - DENEYAP TARAFI:
(Python terminalinden uyandır yazınca D12'ye bağlı led yanar, dinlendir dediğimizde söner)

String received_data = "";  // Gelen veriyi tutmak için değişken
#define yesil D12
void setup() {
  Serial.begin(9600);  // Seri haberleşmeyi başlat
  Serial.println("Hazir");
  pinMode(yesil,  OUTPUT);
  digitalWrite(yesil,  LOW);
}

void loop() {
  // Seri porttan gelen veriyi kontrol et
  if (Serial.available() > 0) {
    // Gelen veriyi oku ve '\n' karakterine kadar bekle    
    String new_data = Serial.readStringUntil('\n')
    if (new_data == "uyandır") {
      digitalWrite(yesil,  HIGH);
    }
    if (new_data == "dinlendir") {
      digitalWrite(yesil,  LOW);
    }
  }
}


PYTHONDAN DENEYAPA VERİ GÖNDERİMİ ÖRNEK KOD - PYTHON TARAFI:

import serial
import time
arduino = serial.Serial('COM3', 9600)  # Port numarasını sistemine göre ayarla
time.sleep(2)  # Bağlantı kurulmasını bekle
while True:
    deger=input("Ledin istikbali nedir? ")
    arduino.write((deger + '\n').())  # Arduino'ya veri gonder
    print(f"Veri gönderildi: {deger}")  # Konsola yaz
