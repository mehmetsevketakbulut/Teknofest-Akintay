import time
import serial
from math import cos, radians, atan2, degrees, sqrt

# --- GPS dönüşüm ve hesaplama fonksiyonları ---
def gps_to_xy(lat0, lon0, lat1, lon1):
    meters_per_deg_lat = 111320
    meters_per_deg_lon = 111320 * cos(radians(lat0))
    delta_lat = lat1 - lat0
    delta_lon = lon1 - lon0
    x = delta_lon * meters_per_deg_lon  # doğu-batı
    y = delta_lat * meters_per_deg_lat  # kuzey-güney
    return (0, 0), (x, y)

def compute_distance_and_bearing(x, y):
    distance = sqrt(x**2 + y**2)
    angle_rad = atan2(y, x)
    angle_deg = degrees(angle_rad)
    if angle_deg < 0:
        angle_deg += 360
    return distance, angle_deg

# --- Basit PID sınıfı ---
class PID:
    def __init__(self, kp, ki, kd, output_limits=(-400, 400)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output, self.max_output = output_limits
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(self.min_output, min(self.max_output, output))
        self.prev_error = error
        return output

# --- Seri iletişim ve kontrol sınıfı ---
class Controller:
    def __init__(self, port="COM8", baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2)
        self.NEUTRAL = 1500
        self.MIN_PWM = 1060
        self.MAX_PWM = 1940
        self.yaw_pid = PID(kp=2.5, ki=0.05, kd=1.5)
        self.throttle_pid = PID(kp=1.2, ki=0.01, kd=0.8)

    def send_motor_commands(self, m_values):
        # m_values listesi 8 motor PWM değerini içermeli
        # Format: M:m1,m2,...,m8\n
        msg = "M:" + ",".join(str(int(m)) for m in m_values) + "\n"
        self.ser.write(msg.encode())

    def read_telemetry(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line.startswith("M:"):
                vals = line[2:].split(",")
                if len(vals) == 8:
                    return [int(v) for v in vals]
        return None

    def close(self):
        self.ser.close()

# --- Ana fonksiyon ---
def main():
    # Başlangıç ve hedef koordinatlar
    lat0, lon0 = 36.123456, 33.123456
    lat1, lon1 = 36.124000, 33.124100

    controller = Controller()
    last_time = time.time()

    while True:
        _, (x_target, y_target) = gps_to_xy(lat0, lon0, lat1, lon1)
        distance, target_bearing = compute_distance_and_bearing(x_target, y_target)

        # Güncel zaman ve dt
        now = time.time()
        dt = now - last_time
        last_time = now

        # Burada sensor/veri güncellemesi yapmalısın (örneğin güncel yaw)
        # Şimdilik simülasyon olarak sabit 0° kabul edelim
        current_yaw = 0.0

        # Yaw hata hesabı (-180,180 aralığında)
        yaw_error = (target_bearing - current_yaw + 180) % 360 - 180

        # PID hesapla
        yaw_output = controller.yaw_pid.compute(yaw_error, dt)
        throttle_error = distance - 2  # 2m hedef aralığı bufferı
        throttle_output = controller.throttle_pid.compute(throttle_error, dt)

        # PWM hesapla
        base_throttle = controller.NEUTRAL + int(throttle_output)
        yaw_pwm = controller.NEUTRAL + int(yaw_output)

        # PWM değerlerini sınırla
        base_throttle = max(controller.MIN_PWM, min(controller.MAX_PWM, base_throttle))
        yaw_pwm = max(controller.MIN_PWM, min(controller.MAX_PWM, yaw_pwm))

        # Motorlara dağıt (örnek)
        motor_values = [
            yaw_pwm,        # M1
            yaw_pwm,        # M2
            base_throttle,  # M3
            base_throttle,  # M4
            yaw_pwm,        # M5
            base_throttle,  # M6
            yaw_pwm,        # M7
            base_throttle,  # M8
        ]

        # Gönder
        controller.send_motor_commands(motor_values)

        # Telemetri oku
        motors = controller.read_telemetry()
        if motors:
            print(f"Mesafe: {distance:.2f} m, Yön: {target_bearing:.2f}°, Motor PWM: {motors}")

        if distance < 1.0:
            print("Hedefe ulaşıldı.")
            break

        time.sleep(0.05)

    controller.close()

if __name__ == "__main__":
    main()
