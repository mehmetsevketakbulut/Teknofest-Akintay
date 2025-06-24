import cv2
import numpy as np
from collections import deque, Counter

# PID parametreleri (ayarlanabilir)
Kp = 0.4
Ki = 0.0
Kd = 0.15
previous_error = 0
integral = 0

direction_history = deque(maxlen=5)
cap = cv2.VideoCapture(0)

# Motorların "güç" veya PWM değerleri (örnek aralık 1000-2000)
# 1500 = durma noktası, <1500 geri, >1500 ileri
# Burada 8 motoru 1500 (stop) ile başlatıyoruz
motors = {
    "onsagust": 1500,
    "onsolust": 1500,
    "arsagust": 1500,
    "arsolust": 1500,
    "onsagalt": 1500,
    "onsolalt": 1500,
    "arsagalt": 1500,
    "arsolalt": 1500,
}

def print_motor_status():
    print(">>> Motor Durumları:")
    for m, val in motors.items():
        print(f"  {m}: {val} µs")
    print("")

def motor_hareketi(yön):
    # Temel güç ayarları (örnek)
    ileri = 1700
    geri = 1300
    dur = 1500
    yavas_ileri = 1600
    yavas_geri = 1400

    # Her yön için motor değerlerini belirle (örnek mantık)
    if yön == "DUZ ILERLE":
        # Tüm motorlar ileri
        for m in motors:
            motors[m] = ileri
        print("[Motor] Düz İlerle")
    elif yön == "SOL KAY":
        # Sağ motorlar yavaş, sol motorlar hızlı (örnek)
        motors["onsagust"] = yavas_ileri
        motors["arsagust"] = yavas_ileri
        motors["onsolust"] = ileri
        motors["arsolust"] = ileri
        motors["onsagalt"] = yavas_ileri
        motors["arsagalt"] = yavas_ileri
        motors["onsolalt"] = ileri
        motors["arsolalt"] = ileri
        print("[Motor] Sola Kay")
    elif yön == "SAG KAY":
        # Sol motorlar yavaş, sağ motorlar hızlı (örnek)
        motors["onsagust"] = ileri
        motors["arsagust"] = ileri
        motors["onsolust"] = yavas_ileri
        motors["arsolust"] = yavas_ileri
        motors["onsagalt"] = ileri
        motors["arsagalt"] = il
