import socket
import threading
import sys
import time

ESP32_IP = "192.168.0.103"
UDP_PORT = 8888

# 1. Khởi tạo 1 Socket UDP duy nhất cho cả Gửi & Nhận
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5)

# Gửi 1 gói tin kết nối ban đầu để ESP32 nhận diện địa chỉ máy tính
sock.sendto(b"PING", (ESP32_IP, UDP_PORT))

running = True

def receive_logs():
    """Thread chạy ngầm lắng nghe Telemetry và Log hệ thống từ ESP32"""
    while running:
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode('utf-8', errors='ignore').strip()
            
            # Phân loại và tô màu các chuỗi Log
            if "[ALERT_LOG]" in msg or ("Err:" in msg and "Err:USER_DISARM_CMD" not in msg and "Err:NONE" not in msg):
                print(f"\n\033[91m[CẢNH BÁO SỰ CỐ] {msg}\033[0m\n> ", end="")
            elif "[SYSTEM_LOG]" in msg:
                print(f"\n\033[93m[HỆ THỐNG] {msg}\033[0m\n> ", end="")
            else:
                # In Telemetry liên tục trên từng dòng
                print(f"\r\033[92m[TELEMETRY] {msg}\033[0m", end="")
        except socket.timeout:
            continue
        except Exception as e:
            if running:
                print(f"\n[LỖI RECEIVE]: {e}")
            break

# 2. Khởi chạy Thread lắng nghe dữ liệu ngầm
t = threading.Thread(target=receive_logs, daemon=True)
t.start()

print("==================================================")
print("      BÀN ĐIỀU KHIỂN & THEO DÕI LOG ESP32 DRONE    ")
print("==================================================")
print("Danh sách lệnh:")
print("  - ARM          : Khởi động 4 động cơ rốt-đa (10% ga)")
print("  - CALIB        : Calib Gyro & Reset Yaw về 0 (Drone nằm yên)")
print("  - TAKEOFF      : Cất cánh tự động mượt mà lên 0.5m")
print("  - LAND         : Hạ cánh tự động & ngắt động cơ")
print("  - DISARM       : Tắt động cơ lập tức")
print("  - EMERGENCY    : Ngắt khẩn cấp toàn bộ hệ thống")
print("  - MOVE r p y z : Lệnh bay (VD: MOVE 0 0 0 0.5)")
print("  - exit         : Thoát chương trình")
print("==================================================")

# 3. Vòng lặp nhận lệnh từ bàn phím và gửi qua UDP
try:
    while True:
        cmd = input("\n> ").strip()
        if not cmd:
            continue
        if cmd.lower() == 'exit':
            running = False
            break
        
        # Gửi lệnh trực tiếp qua UDP socket hiện tại
        sock.sendto(cmd.encode('utf-8'), (ESP32_IP, UDP_PORT))
        print(f"\033[94m--> Đã gửi lệnh: {cmd}\033[0m")
        
except KeyboardInterrupt:
    print("\nĐã ngắt chương trình.")
finally:
    running = False
    sock.close()