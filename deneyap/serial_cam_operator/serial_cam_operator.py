from picamera2 import Picamera2
import serial
import subprocess
import time

camera = Picamera2()
camera.start()
time.sleep(1)

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

recording = False
video_filename_h264 = ""
video_filename_mp4 = ""

try:
    print("Deneyap'tan komut bekleniyor...")

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            print(f"Gelen komut: {line}")

            if line == "PHOTO":
                timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
                filename = f"photo_{timestamp}.jpg"
                camera.capture_file(filename)
                print(f"📸 Fotoğraf çekildi: {filename}")

            elif line == "VIDEO":
                timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
                if not recording:
                    video_filename_h264 = f"video_{timestamp}.h264"
                    video_filename_mp4 = f"video_{timestamp}.mp4"
                    camera.start_recording(video_filename_h264)
                    recording = True
                    print("🎥 Video kaydı BAŞLADI")
                else:
                    camera.stop_recording()
                    recording = False
                    print("⏹️ Video kaydı DURDU, mp4'e dönüştürülüyor...")
                    subprocess.run([
                        "ffmpeg", "-y", "-i", video_filename_h264,
                        "-c:v", "copy", video_filename_mp4
                    ])
                    print(f"✅ MP4 oluşturuldu: {video_filename_mp4}")

except KeyboardInterrupt:
    if recording:
        camera.stop_recording()
        subprocess.run([
            "ffmpeg", "-y", "-i", video_filename_h264,
            "-c:v", "copy", video_filename_mp4
        ])
        print(f"Acil çıkışta MP4 oluşturuldu: {video_filename_mp4}")

    camera.close()
    ser.close()
    print("Program sonlandırıldı.")
    