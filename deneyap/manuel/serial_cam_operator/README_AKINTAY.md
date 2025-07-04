# 🎮📷 AKINTAY - Joystick Kontrollü Foto/Video Kayıt Sistemi

Bu proje, **Deneyap Kart** ile joystick üzerinden kontrol edilen ve **Raspberry Pi** üzerinde bağlı bir kamerayla **fotoğraf ve video çekebilen** bir sistemdir.  
ROV gibi robotik projelerde uzaktan kontrol ile medya yakalamak isteyenler için tasarlanmıştır.

---

## 🔧 Sistem Mimarisi

**1. Donanım:**
- 🕹️ Joystick (2 eksen + 2 buton)
- ⚙️ 8 motor kontrolü (Servo - ESC)
- 🎥 Raspberry Pi (Picamera2 ile)
- 📦 Seri haberleşme (USB - UART)

**2. Bileşenler:**
| Bileşen | Görev |
|--------|-------|
| Deneyap Kart | Joystick verisi ve butonları okur, motorları kontrol eder, Raspberry Pi'ye komut gönderir |
| Raspberry Pi | Gelen komutlara göre kamera ile fotoğraf çeker veya video kaydeder |

---

## 📁 Dosya Yapısı

```
AKINTAY/
├── control/
│   └── deneyap_joycam_controller.ino        # Deneyap joystick ve motor kontrol kodu
├── camera/
│   └── cam_receiver_rpi.py                  # Raspberry Pi kamera kontrol kodu (Python)
├── media/
│   ├── photo_*.jpg                          # Çekilen fotoğraflar buraya gelir
│   └── video_*.mp4                          # Kaydedilen videolar buraya gelir
├── README.md                                # Bu döküman
```

---

## 🚀 Kurulum

### 🔌 Donanım Bağlantısı

- Joystick X1 → A0  
- Joystick Y1 → A2  
- Joystick X2 → A1  
- Joystick Y2 → A3  
- Buton1 (FOTO) → D9  
- Buton2 (VIDEO) → D0  
- 8 ESC ya da servo → D1–D8

### 💻 Yazılım

#### 1. Deneyap Tarafı

- `deneyap_joycam_controller.ino` dosyasını Arduino IDE ile Deneyap Kart’a yükle.

#### 2. Raspberry Pi Tarafı

- Gerekli Python paketlerini yükle:
  ```bash
  pip install picamera2
  sudo apt install ffmpeg
  ```

- `cam_receiver_rpi.py` dosyasını çalıştır:
  ```bash
  python3 cam_receiver_rpi.py
  ```

> **Not:** USB portun `/dev/ttyUSB0` değilse Python kodunda onu değiştirmen gerekebilir.

---

## 🕹️ Kullanım

| Eylem | Buton | İşlem |
|------|-------|--------|
| 📸 Fotoğraf çek | D9 | Raspberry `.jpg` çeker |
| 🎥 Video başlat/durdur | D0 | Raspberry `.h264` kaydeder ve `.mp4`'e dönüştürür |

---

## 📌 Geliştirme Önerileri

- 🌐 Videoları ağ üzerinden canlı aktar
- 🔁 Medyaları otomatik olarak buluta yükle
- 🔦 LED ile kayıt göstergesi
- 🧠 Joystick yönüne göre kamera hareketi

---

## 🧠 Yazılımcı Notu

Bu proje, gerçek zamanlı robotik kontrol ile medya kaydını birleştirir.  
Kullanımı kolay, genişletmeye açık ve Deneyap + Raspberry Pi gibi yerli-yaygın sistemleri bir araya getirir.

> Geliştiren: **Bahattin Yunus Çetin**  
> Proje adı: **AKINTAY** – Akıllı Kontrollü İnteraktif Tabanlı Aygıt Yönetimi

---

## 📬 İletişim

Bu sistemle ilgili destek, geliştirme fikri ya da katkı için bana yazabilirsin.