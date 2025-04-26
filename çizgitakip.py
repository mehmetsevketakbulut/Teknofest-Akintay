import cv2
import numpy as np

cap = cv2.VideoCapture(0)  # 0, varsayılan kamerayı kullanır

# Kontrast artırma (CLAHE) için parametreler
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

# Kamera kalibrasyonu için distorsiyon matrisleri (bunları gerçek kalibrasyonla elde edebilirsin)
# Aşağıdaki örnek, kamera distorsiyonunu düzeltmek için kullanılabilir.
# dist_coeffs = np.zeros((5, 1))  # Distorsiyon katsayıları
# camera_matrix = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])  # Kamera matrisi

while True:
    ret, frame = cap.read()  # Kameradan görüntü al
    if not ret:
        break

    # Görüntüyü gri tonlamaya çevir
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # CLAHE ile kontrastı artır
    gray = clahe.apply(gray)

    # Kamera distorsiyon düzeltmesi (Kamera matrisi ve distorsiyon katsayılarını kullan)
    # frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

    # HSV renk uzayına dönüştür
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Siyah renkleri filtrele (Siyah çizgiler için)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 50])  # Siyah için uygun aralık
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # Maskeyi uygula
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Görüntüyü bulanıklaştır (yaklaşırken kenarları netleştirmek için)
    result = cv2.GaussianBlur(result, (5, 5), 0)

    # Canny Edge Detection
    edges = cv2.Canny(result, 50, 150)

    # Morfolojik işlemler (gürültü temizleme)
    kernel = np.ones((5, 5), np.uint8)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    # Konturları bul
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # En büyük konturu (çizgiyi) bul
        largest_contour = max(contours, key=cv2.contourArea)

        # Momentleri hesapla (Merkez noktasını bulmak için)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])  # X koordinatı
            cy = int(M["m01"] / M["m00"])  # Y koordinatı

            # Çizginin merkezine bir daire çiz
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

            # Çizginin merkezine göre yön belirle
            frame_center = frame.shape[1] // 2  # Ekranın ortası
            if cx < frame_center - 50:
                direction = "SOL"  # Çizgi sola kaydı, sola dön
            elif cx > frame_center + 50:
                direction = "SAG"  # Çizgi sağa kaydı, sağa dön
            else:
                direction = "DUZ İLERLE"

            cv2.putText(frame, f"Yon: {direction}", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Görüntüyü ekranda göster
    cv2.imshow("Underwater Line Following", frame)
    cv2.imshow("Threshold", mask)

    # 'q' tuşuna basılırsa çık


    if cv2.waitKey(1) & 0xFF == 27:
        break
    # 'ESC' tuşuna basılırsa çık

cap.release()
cv2.destroyAllWindows()
