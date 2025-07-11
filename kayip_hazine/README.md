Görev Tanımı

**Kayıp Hazine Avı Görevi**'nde amaç:
- Görev başında verilen **hedef koordinata** (X, Y) ulaşmak.
- **Başlangıç konumuna göre** yön ve mesafe hesaplayarak hedefe ilerlemek.
- Tüm yolculuk boyunca sadece **IMU (LSM6DSM)** verileriyle yönelim sağlamak.
- Görüntü işleme, sonar, GPS ya da DVL verisi kullanılmaz.

---

##  Seri Port Haberleşmesi

Arduino tabanlı mikrodenetleyici, üst sistemle (örneğin Raspberry Pi veya bir bilgisayar) **seri port üzerinden haberleşir**:

- Python tarafında joystick benzeri yönlendirme verileri gönderilebilir.
- Arduino bu verileri alır ve **8 motor için PWM sinyallerini üretir**.
- Geri bildirim amacıyla Arduino'dan veri gönderimi de yapılabilir.

---

##  Kullanılan Algoritma

Algoritma üç temel adımdan oluşur:

### 1. **Koordinat Tabanlı Yön ve Mesafe Hesabı**
- `gps_to_xy()` fonksiyonu, başlangıç ve hedef koordinatları arasındaki farkı metre cinsine dönüştürür.
- Bu farktan hedef yön açısı (`θ`) ve toplam mesafe (`d`) hesaplanır.

### 2. **IMU Tabanlı Yön Tespiti**
- Dahili **LSM6DSM IMU** sensörü, aracın anlık `yaw` açısını verir.
- Sistem, yönelimi sürekli takip ederek hedef açıya olan farkı ölçer.

### 3. **PID Kontrollü Yön Düzeltme**
- Yön farkı, PID algoritmasıyla işlenir.
- Sapma açısına göre motorlara verilen PWM sinyalleri değiştirilerek yön düzeltmesi sağlanır.
- Araç hedef doğrultusuna hizalanınca düz ilerleme başlar.

---
