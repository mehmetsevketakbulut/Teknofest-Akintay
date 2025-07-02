import time
import serial
from math import atan2, degrees, sqrt, cos, radians
from collections import deque

class DroneController:
    def __init__(self, port="COM8", baudrate=115200):
        # 🧷 Seri port bağlantısı
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Bağlantı için bekleme süresi
        
        # 🎮 Kontrol parametreleri
        self.MIN_PWM = 1060
        self.MAX_PWM = 1940
        self.NEUTRAL = 1500
        
        # 📊 Navigasyon verileri
        self.current_yaw = 0.0
        self.current_position = (0.0, 0.0)
        self.battery_voltage = 12.6  # Varsayılan değer
        
        # ⚙️ PID ayarları
        self.pid_params = {
            'yaw': {'kp': 2.5, 'ki': 0.05, 'kd': 1.5, 'max_output': 300},
            'throttle': {'kp': 1.2, 'ki': 0.01, 'kd': 0.8, 'max_output': 200}
        }
        
        # 📈 Veri filtreleme
        self.yaw_history = deque(maxlen=5)
        self.distance_history = deque(maxlen=3)

    def __del__(self):
        self.ser.close()

    def clamp(self, val, minn=None, maxx=None):
        minn = minn or self.MIN_PWM
        maxx = maxx or self.MAX_PWM
        return max(min(val, maxx), minn)

    def gps_to_xy(self, lat0, lon0, lat1, lon1):
        meters_per_deg_lat = 111320
        meters_per_deg_lon = 111320 * cos(radians(lat0))
        return (lon1 - lon0) * meters_per_deg_lon, (lat1 - lat0) * meters_per_deg_lat

    def compute_distance_and_bearing(self, x, y):
        distance = sqrt(x**2 + y**2)
        angle = degrees(atan2(y, x)) % 360
        return distance, angle

    def send_control_command(self, roll=1500, pitch=1500, yaw=1500, throttle=1500):
        cmd = f"R:{roll},P:{pitch},Y:{yaw},T:{throttle}\n"
        self.ser.write(cmd.encode())

    def read_telemetry(self):
        try:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                
                if not line.startswith("TELEMETRY:"):
                    raise ValueError("Geçersiz veri formatı")
                
                parts = dict(item.split(":") for item in line[10:].split(","))
                
                # Sensör verilerini güncelle
                self.current_yaw = float(parts.get('YAW', 0.0))
                self.battery_voltage = float(parts.get('BAT', 12.6))
                
                return {
                    'yaw': self.current_yaw,
                    'pitch': float(parts.get('PITCH', 0.0)),
                    'roll': float(parts.get('ROLL', 0.0)),
                    'motors': [int(parts.get(f'M{i}', 1500)) for i in range(1, 9)],
                    'battery': self.battery_voltage
                }
        except Exception as e:
            print(f"[TELEMETRY ERROR] {str(e)}")
            return None

    def calculate_pid(self, error, integral, previous_error, dt, control_type='yaw'):
        params = self.pid_params[control_type]
        
        # Integral term with anti-windup
        integral += error * dt
        integral = self.clamp(integral, -100, 100)
        
        # Derivative term with smoothing
        derivative = (error - previous_error) / dt if dt > 0 else 0
        
        # PID output
        output = (params['kp'] * error + 
                 params['ki'] * integral + 
                 params['kd'] * derivative)
        
        return (
            self.clamp(output, -params['max_output'], params['max_output']),
            integral,
            error
        )

    def emergency_stop(self):
        print("[EMERGENCY] Stopping all motors!")
        self.send_control_command(throttle=1000)
        time.sleep(1)
        self.ser.close()
        exit(1)

    def check_system_health(self):
        telemetry = self.read_telemetry()
        if not telemetry:
            print("[WARNING] No telemetry data received!")
            return False
            
        if telemetry['battery'] < 10.5:
            print(f"[CRITICAL] Low battery: {telemetry['battery']}V")
            self.emergency_stop()
            
        return True

    def navigate_to_waypoint(self, target_lat, target_lon, speed=0.5):
        current_lat, current_lon = self.current_position
        x, y = self.gps_to_xy(current_lat, current_lon, target_lat, target_lon)
        distance, target_yaw = self.compute_distance_and_bearing(x, y)
        
        print(f"🎯 Target: {distance:.2f}m @ {target_yaw:.1f}°")
        
        # PID variables
        yaw_integral = 0.0
        yaw_last_error = 0.0
        
        throttle_integral = 0.0
        throttle_last_error = 0.0
        
        last_time = time.time()
        remaining_distance = distance
        
        while remaining_distance > 1.0:  # Stop within 1m
            if not self.check_system_health():
                time.sleep(0.1)
                continue
                
            # Timing control
            now = time.time()
            dt = now - last_time
            last_time = now
            
            # Get current state
            telemetry = self.read_telemetry()
            current_yaw = telemetry['yaw']
            
            # Yaw control
            yaw_error = (target_yaw - current_yaw + 180) % 360 - 180
            yaw_output, yaw_integral, yaw_last_error = self.calculate_pid(
                yaw_error, yaw_integral, yaw_last_error, dt, 'yaw')
            
            # Throttle control based on distance
            throttle_error = min(10, remaining_distance) - 2  # Target 2m buffer
            throttle_output, throttle_integral, throttle_last_error = self.calculate_pid(
                throttle_error, throttle_integral, throttle_last_error, dt, 'throttle')
            
            # Calculate control values
            base_throttle = self.NEUTRAL + int(speed * 100)
            yaw_control = self.NEUTRAL + int(yaw_output)
            throttle_control = self.clamp(base_throttle + int(throttle_output))
            
            # Send commands
            self.send_control_command(
                yaw=yaw_control,
                throttle=throttle_control
            )
            
            # Update remaining distance
            traveled = speed * (time.time() - last_time)
            remaining_distance = max(0, distance - traveled)
            
            # Display status
            print(f"🧭 Yaw: {current_yaw:.1f}° | Error: {yaw_error:.1f}° | "
                  f"Throttle: {throttle_control} | Remaining: {remaining_distance:.1f}m")
            
            time.sleep(0.05)
        
        print("✅ Waypoint reached!")
        self.send_control_command()  # Return to neutral

    def execute_mission(self, waypoints):
        try:
            for i, (lat, lon) in enumerate(waypoints):
                print(f"\n🚀 Navigating to waypoint {i+1}/{len(waypoints)}")
                self.navigate_to_waypoint(lat, lon)
                time.sleep(1)  # Pause at each waypoint
                
            print("\n🎉 Mission complete!")
            self.send_control_command(throttle=1000)  # Land
            
        except KeyboardInterrupt:
            self.emergency_stop()


if __name__ == "__main__":
    # Test waypoints (latitude, longitude)
    MISSION_WAYPOINTS = [
        (36.123456, 33.123456),
        (36.123500, 33.123500),
        (36.123600, 33.123600)
    ]
    
    controller = DroneController()
    controller.execute_mission(MISSION_WAYPOINTS)