import time
import serial
from math import atan2, degrees, sqrt, cos, radians

# Seri port açılışı - timeout mutlaka koy
ser = serial.Serial("COM8", 115200, timeout=1)
time.sleep(2)

def clamp(val, minn=1300, maxx=1700):
    return max(min(val, maxx), minn)

def gps_to_xy(lat0, lon0, lat1, lon1):
    meters_per_deg_lat = 111320
    meters_per_deg_lon = 111320 * cos(radians(lat0))
    delta_lat = lat1 - lat0
    delta_lon = lon1 - lon0
    x = delta_lon * meters_per_deg_lon
    y = delta_lat * meters_per_deg_lat
    return x, y

def compute_distance_and_bearing(x, y):
    distance = sqrt(x**2 + y**2)
    angle = degrees(atan2(y, x))
    if angle < 0:
        angle += 360
    return distance, angle

def send_joystick(x1, y1, x2, y2):
    msg = f"X1:{x1},Y1:{y1},X2:{x2},Y2:{y2}\n"
    ser.write(msg.encode())

def read_data():
    if ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        try:
            parts = dict(part.split(":") for part in line.split(","))
            return {
                "yaw": float(parts.get("YAW", 0.0)),
                "m1": int(parts.get("M1", 1500)),
                "m2": int(parts.get("M2", 1500)),
                "m3": int(parts.get("M3", 1500)),
                "m4": int(parts.get("M4", 1500)),
                "m5": int(parts.get("M5", 1500)),
                "m6": int(parts.get("M6", 1500)),
                "m7": int(parts.get("M7", 1500)),
                "m8": int(parts.get("M8", 1500)),
            }
        
        except Exception as e:
            print(f"[Hata] Veri ayrıştırma hatası: {e} Satır: {line}")
            return None
    return None

def calculate_pid(error, integral, previous_error, dt, kp=2.0, ki=0.0, kd=1.2):
    integral += error * dt
    derivative = (error - previous_error) / dt if dt > 0 else 0
    output = kp * error + ki * integral + kd * derivative
    output = max(min(output, 200), -200)
    return output, integral, error

def smooth_transition(current, target, step=10):
    if current < target:
        current += step
        if current > target:
            current = target
    elif current > target:
        current -= step
        if current < target:
            current = target
    return current

def execute_navigation(lat0, lon0, lat1, lon1):
    x, y = gps_to_xy(lat0, lon0, lat1, lon1)
    distance, target_yaw = compute_distance_and_bearing(x, y)

    print(f"📍 Mesafe: {distance:.2f} m")
    print(f"🧭 Hedef yönü: {target_yaw:.2f}°")

    speed = 0.2  # m/s (düzenlenebilir)
    duration = distance / speed if speed > 0 else 0
    print(f"⏳ Tahmini süre: {duration:.1f} s")

    x1_current = 1500
    y1_current = 1500
    x2_current = 1500
    y2_current = 1500

    integral = 0.0
    previous_error = 0.0
    last_time = time.time()

    start_time = last_time
    while time.time() - start_time < duration:
        now = time.time()
        dt = now - last_time
        last_time = now

        data = read_data()
        if data:
            yaw = data["yaw"]

            # Yaw error hesaplama, -180..180 aralığında normalize et
            yaw_error = target_yaw - yaw
            if yaw_error > 180:
                yaw_error -= 360
            elif yaw_error < -180:
                yaw_error += 360

            pid_output, integral, previous_error = calculate_pid(yaw_error, integral, previous_error, dt)

            x1_target = clamp(1500 + pid_output)
            y2_target = clamp(1600)  # İleri hız sabit, isteğe göre değiştirilebilir

            x1_current = smooth_transition(x1_current, x1_target, step=10)
            y2_current = smooth_transition(y2_current, y2_target, step=10)

            send_joystick(int(x1_current), 1500, 1500, int(y2_current))

            motor_status = ", ".join([f"M{i+1}: {data[f'm{i+1}']}" for i in range(8)])

            print(f"🔁 Yön: {yaw:.2f}°, Hata: {yaw_error:.2f}, X1: {int(x1_current)}, Y2: {int(y2_current)}")
            print(f"🛠 Motor PWM → {motor_status}")

        else:
            print("[Uyarı] Seri porttan veri gelmedi.")

        time.sleep(0.1)

    print("🎯 Görev tamamlandı.")
    send_joystick(1500, 1500, 1500, 1500)
    ser.close()

if __name__ == "__main__":
    # Örnek koordinatlar, gerçek konumlarla değiştir
    lat0, lon0 = 36.123456, 33.123456
    lat1, lon1 = 36.124000, 33.124100
    execute_navigation(lat0, lon0, lat1, lon1)