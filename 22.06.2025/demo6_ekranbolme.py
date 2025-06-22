import cv2
import numpy as np
import tensorflow as tf
import threading
import time
from collections import deque, Counter


# Kategoriler ve model yükleme
categories = ['circle', 'hexagon', 'parallelogram', 'pentagon', 'rectangle', 'rhombus', 'square', 'triangle']
model = tf.keras.models.load_model("shape_model_final1.h5")

# Kamera nesnesi
cap = cv2.VideoCapture(0)

# PID parametreleri
Kp = 0.4
Ki = 0.0
Kd = 0.15
previous_error = 0
integral = 0

# Karar sabitleme (moving mode)
direction_history = deque(maxlen=5)
last_applied_direction = "DUZ ILERLE"
last_direction_time = time.time()

# Karar üretme fonksiyonu
def get_stable_direction(current_direction):
    direction_history.append(current_direction)
    direction_mode = Counter(direction_history).most_common(1)[0][0]
    return direction_mode

def line_following():
    global previous_error, integral
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        height, width, _ = frame.shape

        ### ROI TANIMI (Alt orta pencere)
        roi_h, roi_w = 150, 200
        roi_top = height - roi_h
        roi_bottom = height
        roi_left = (width - roi_w) // 2
        roi_right = (width + roi_w) // 2
        roi = frame[roi_top:roi_bottom, roi_left:roi_right]

        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_roi = cv2.inRange(hsv_roi, (0, 0, 0), (180, 255, 80))
        mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        contours_roi, _ = cv2.findContours(mask_roi, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        direction = "CIZGI YOK"
        found_in_roi = False

        ### ROI İÇİNDE ÇİZGİ VAR MI?
        if contours_roi:
            largest = max(contours_roi, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] > 0:
                cx_roi = int(M["m10"] / M["m00"])
                cy_roi = int(M["m01"] / M["m00"])
                roi_center_x = roi_w // 2

                # PID
                error = cx_roi - roi_center_x
                integral += error
                derivative = error - previous_error
                output = Kp * error + Ki * integral + Kd * derivative
                previous_error = error

                if abs(error) < 20:
                    direction = "DUZ ILERLE"
                elif error < -20:
                    direction = "SOL"
                elif error > 20:
                    direction = "SAG"

                found_in_roi = True
                # Görsel İşaret
                cv2.circle(roi, (cx_roi, cy_roi), 5, (0, 255, 0), -1)

        ### ROI DIŞINDA ARAMA
        if not found_in_roi:
            hsv_full = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_full = cv2.inRange(hsv_full, (0, 0, 0), (180, 255, 80))
            mask_full = cv2.morphologyEx(mask_full, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
            contours_full, _ = cv2.findContours(mask_full, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours_full:
                largest = max(contours_full, key=cv2.contourArea)
                M = cv2.moments(largest)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    frame_center = width // 2

                    if cx < frame_center - 50:
                        direction = "SAG (ROI'ye dön)"
                    elif cx > frame_center + 50:
                        direction = "SOL (ROI'ye dön)"
                    else:
                        direction = "DUZ ROI'ye"
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        ### ROI Görseli ve Ana Çerçeve
        stable_direction = get_stable_direction(direction)
        cv2.putText(frame, f"Yon: {stable_direction}", (40, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # ROI kutusu çiz
        cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (255, 0, 0), 2)

        # Gösterimler
        cv2.imshow("Line Following", frame)
        cv2.imshow("ROI", mask_roi)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    cap.release()
    cv2.destroyAllWindows()


    cap.release()
    cv2.destroyAllWindows()


    cap.release()
    cv2.destroyAllWindows()

    cap.release()
    cv2.destroyAllWindows()

        
def shape_detection():
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        roi = cv2.resize(frame, (64, 64))
        img_array = roi.astype('float32') / 255.0
        img_array = np.expand_dims(img_array, axis=0)

        prediction = model.predict(img_array)
        predicted_class = categories[np.argmax(prediction)]

        if predicted_class:
            frame_copy = frame.copy()
            cv2.putText(frame_copy, f"Tahmin: {predicted_class}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Canli Tahmin", frame_copy)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Thread'leri başlat
thread1 = threading.Thread(target=shape_detection)
thread2 = threading.Thread(target=line_following)

thread1.start()
thread2.start()

thread1.join()
thread2.join()

cap.release()
cv2.destroyAllWindows()