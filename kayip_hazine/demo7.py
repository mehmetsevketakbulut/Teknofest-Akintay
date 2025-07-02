import time
import serial
from math import atan2, degrees, sqrt, cos, radians

class PIDController:
    def __init__(self, kp, ki, kd, max_output=400):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0.0
        self.previous_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0

    def calculate(self, error, dt):
        self.integral += error * dt
        self.integral = max(min(self.integral, 100), -100)
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        self.previous_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(min(output, self.max_output), -self.max_output)

class DroneController:
    def __init__(self, port="COM8", baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Bağlantı için bekleme süresi
        self.PWM_MIN = 1000
        self.PWM_MAX = 2000
        self.current_position = (36.123000, 33.123000)  # Dummy başlangıç koordinatı

    def clamp_pwm(self, value):
        return max(min(int(value), self.PWM_MAX), self.PWM_MIN)

    def gps_to_xy(self, lat0, lon0, lat1, lon1):
        meters_per_deg_lat = 111320
        meters_per_deg_lon = 111320 * cos(radians(lat0))
        dx = (lon1 - lon0) * meters_per_deg_lon
        dy = (lat1 - lat0) * meters_per_deg_lat
        return dx, dy

    def compute_distance_and_bearing(self, dx, dy):
        distance = sqrt(dx ** 2 + dy ** 2)
        bearing = degrees(atan2(dy, dx)) % 360
        return distance, bearing

    def send_control_command(self, x1=1500, y1=1500, x2=1500, y2=1500):
        cmd = f"X1:{x1},Y1:{y1},X2:{x2},Y2:{y2}\n"
        self.ser.write(cmd.encode())

    def read_telemetry(self):
        try:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                parts = dict(item.split(":") for item in line.split(",") if ":" in item)
                return {
                    'roll': float(parts.get('ROLL', 0.0)),
                    'pitch': float(parts.get('PITCH', 0.0)),
                    'motors': [int(parts.get(f'M{i+1}', 1500)) for i in range(8)]
                }
        except Exception as e:
            print(f"[TELEMETRİ HATASI] {e}")
        return None

    def navigate_to_waypoint(self, target_lat, target_lon, speed=0.5):
        lat0, lon0 = self.current_position
        dx, dy = self.gps_to_xy(lat0, lon0, target_lat, target_lon)
        distance, target_yaw = self.compute_distance_and_bearing(dx, dy)
        print(f"\n🎯 Hedef: {distance:.1f} m | Açı: {target_yaw:.1f}°")

        yaw_pid = PIDController(3.0, 0.05, 1.2)
        forward_pid = PIDController(2.0, 0.0, 0.5)

        remaining = distance
        last_time = time.time()

        while remaining > 1.0:
            telemetry = self.read_telemetry()
            if not telemetry:
                continue

            dt = time.time() - last_time
            last_time = time.time()

            current_yaw = telemetry['roll']  # Geçici olarak roll = yaw
            yaw_error = (target_yaw - current_yaw + 180) % 360 - 180
            forward_error = remaining

            yaw_output = yaw_pid.calculate(yaw_error, dt)
            forward_output = forward_pid.calculate(forward_error, dt)

            x2 = self.clamp_pwm(1500 + yaw_output)
            y1 = self.clamp_pwm(1500 + forward_output)

            self.send_control_command(x1=1500, y1=y1, x2=x2, y2=1500)

            # Motor PWM değerlerini yazdır
            motors = telemetry.get('motors', [1500]*8)
            motor_str = " | ".join(f"M{i+1}:{val}" for i, val in enumerate(motors))

            print(f"🧭 Sapma: {yaw_error:>5.1f}° | Mesafe: {remaining:>5.1f} m | {motor_str}")

            remaining -= speed * dt
            time.sleep(0.05)

        print("✅ Noktaya ulaşıldı!")
        self.send_control_command()  # Nötr konum

    def execute_mission(self, waypoints):
        for i, (lat, lon) in enumerate(waypoints):
            print(f"\n🚩 Nokta {i+1}/{len(waypoints)}")
            self.navigate_to_waypoint(lat, lon)
            time.sleep(1)

if __name__ == "__main__":
    mission = [
        (36.123456, 33.123456),
        (36.123500, 33.123500),
        (36.123600, 33.123600)
    ]
    controller = DroneController()
    controller.execute_mission(mission)
