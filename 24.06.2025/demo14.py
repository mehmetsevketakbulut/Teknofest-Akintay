import cv2
import numpy as np
import serial
from collections import deque, Counter
import threading

# Serial port tanımı (ESP32 bağlantısı)
ser = serial.Serial('COM8', 115200, timeout=1)

# PID ayarları
Kp = 0.4
Ki = 0.0
Kd = 0.15
previous_error = 0
integral = 0

# Yön geçmişi
direction_history = deque(maxlen=5)

# Geri bildirim verisi
latest_motor_values = [1500] * 8  # 8 motorluk liste

# Thread: ESP32'den gelen motor verisini terminale basar
def read_serial_feedback():
    global latest_motor_values
    while True:
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("motors:"):
                    values = line.replace("motors:", "").split(",")
                    if len(values) == 8:
                        latest_motor_values = [int(v) for v in values]
                        print("\n[ESP32 Geri Bildirim]")
                        print(f"M1:{values[0]} M2:{values[1]} M3:{values[2]} M4:{values[3]}")
                        print(f"M5:{values[4]} M6:{values[5]} M7:{values[6]} M8:{values[7]}\n")
            except:
                pass  # veri bozuksa geç

# Motor PWM hesaplayıcı
def motor_pwm_from_direction(direction):
    pwm_values = {
        "DUZ ILERLE":     [1500, 1500, 1500, 1500],
        "SOL DON":        [1300, 1700, 1300, 1700],
        "SAG DON":        [1700, 1300, 1700, 1300],
        "SOL KAY":        [1300, 1500, 1300, 1500],
        "SAG KAY":        [1700, 1500, 1700, 1500],
        "DON (CIZGI ARANIYOR)": [1400, 1600, 1400, 1600],
    }
    return pwm_values.get(direction, [1500, 1500, 1500, 1500])

def get_orientation(contour):
    sz = len(contour)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(sz):
        data_pts[i, 0] = contour[i, 0, 0]
        data_pts[i, 1] = contour[i, 0, 1]
    mean, eigenvectors, _ = cv2.PCACompute2(data_pts, mean=np.empty((0)))
    angle = np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0])
    return np.degrees(angle)

def get_stable_direction(current_direction):
    direction_history.append(current_direction)
    return Counter(direction_history).most_common(1)[0][0]

def send_motor_values(values):
    msg = f"x1:{values[0]},x2:{values[1]},y1:{values[2]},y2:{values[3]}\n"
    ser.write(msg.encode())

def line_following():
    global previous_error, integral

    cap = cv2.VideoCapture(0)
    kernel = np.ones((5, 5), np.uint8)
    exploration_counter = 0
    exploration_mode = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        height, width, _ = frame.shape
        roi_top, roi_bottom = height - 150, height
        roi_left, roi_right = width // 2 - 100, width // 2 + 100
        roi_center_x = (roi_right - roi_left) // 2
        roi = frame[roi_top:roi_bottom, roi_left:roi_right]

        blurred = cv2.GaussianBlur(roi, (7, 7), 0)
        hsv_roi = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_roi, np.array([0, 0, 0]), np.array([180, 255, 90]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        min_area = 500
        filtered = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]

        direction = "CIZGI YOK"
        orientation_angle = None

        if filtered:
            largest = max(filtered, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                error = cx - roi_center_x
                integral += error
                derivative = error - previous_error
                output = Kp * error + Ki * integral + Kd * derivative
                previous_error = error

                orientation_angle = get_orientation(largest)
                angle_dev = abs(abs(orientation_angle) - 90)

                exploration_mode = False
                exploration_counter = 0

                if angle_dev < 15:
                    direction = "DUZ ILERLE" if abs(error) < 30 else ("SOL KAY" if error < 0 else "SAG KAY")
                else:
                    direction = "SOL DON" if orientation_angle < 0 else "SAG DON"
        else:
            exploration_counter += 1
            if exploration_counter > 10:
                exploration_mode = True

        if exploration_mode:
            direction = "DON (CIZGI ARANIYOR)"

        stable_direction = get_stable_direction(direction)
        motor_values = motor_pwm_from_direction(stable_direction)
        send_motor_values(motor_values)

        # ESC çıkışını terminale yaz
        print(f"[Komut] {stable_direction} | x1:{motor_values[0]} x2:{motor_values[1]} y1:{motor_values[2]} y2:{motor_values[3]}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    ser.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Seri geri bildirim thread'ini başlat
    threading.Thread(target=read_serial_feedback, daemon=True).start()
    line_following()
