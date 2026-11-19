import socket
import threading
import sys

ESP32_IP = "192.168.0.103"
UDP_PORT = 8888

# Tạo 1 Socket duy nhất cho cả Nhận và Gửi
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
sock.settimeout(1.0)

# 1. THREAD LẮNG NGHE VÀ IN TELEMETRY / LOG
def receive_thread():
    print(f"[INFO] Đã kết nối UDP Port {UDP_PORT}. Đang nhận log...")
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode('utf-8', errors='ignore').strip()
            
            if "[ALERT_LOG]" in msg or ("Err:" in msg and "Err:USER_DISARM_CMD" not in msg and "Err:NONE" not in msg):
                print(f"\033[91m[CẢNH BÁO SỰ CỐ] {msg}\033[0m")
            elif "[SYSTEM_LOG]" in msg:
                print(f"\033[93m[HỆ THỐNG] {msg}\033[0m")
            else:
                print(f"\033[92m[TELEMETRY] {msg}\033[0m")
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[LỖI RECEIVE]: {e}")
            break

# Kích hoạt Thread nhận Log chạy ngầm
t = threading.Thread(target=receive_thread, daemon=True)
t.start()

# 2. VÒNG LẶP NHẬP LỆNH TỪ BÀN PHÍM (CONSOLE COMMAND)
print("=== BÀN ĐIỀU KHIỂN ESP32 DRONE ===")
print("Nhập lệnh: ARM | DISARM | TAKEOFF | LAND | CALIB | EMERGENCY | exit")

try:
    while True:
        cmd = input().strip()
        if cmd.lower() == 'exit':
            break
        if cmd:
            # Gửi lệnh trực tiếp qua UDP từ chính Socket lắng nghe
            sock.sendto(cmd.encode('utf-8'), (ESP32_IP, UDP_PORT))
            print(f"--> Đã gửi lệnh: {cmd}")
except KeyboardInterrupt:
    print("\nĐã thoát chương trình.")