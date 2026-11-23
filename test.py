import socket
import struct
import time

ESP32_IP = "192.168.4.1"
UDP_PORT = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def test_single_motor(motor_num):
    """
    Test từng motor từ 1 đến 4
    """
    print(f"\n--- ĐANG TEST MOTOR {motor_num} ---")
    if motor_num == 1:
        print("M1 (GPIO 4): Phải-Trước | Chiều quay chuẩn: CCW (Ngược chiều kim đồng hồ)")
    elif motor_num == 2:
        print("M2 (GPIO 5): Phải-Sau   | Chiều quay chuẩn: CW (Thuận chiều kim đồng hồ)")
    elif motor_num == 3:
        print("M3 (GPIO 18): Trái-Sau  | Chiều quay chuẩn: CCW (Ngược chiều kim đồng hồ)")
    elif motor_num == 4:
        print("M4 (GPIO 19): Trái-Trước| Chiều quay chuẩn: CW (Thuận chiều kim đồng hồ)")

    # Gửi lệnh test Manual trong 2 giây
    # Format manual packet: roll=0, pitch=0, yaw=0, thrust=0.12 (12%)
    for _ in range(100):
        # Nếu firmware bạn dùng PosHold 18-byte, gửi gói kiểm tra:
        payload = struct.pack('<ffffBB', 0.0, 0.0, 0.0, 0.0, 1, 0)
        sock.sendto(payload, (ESP32_IP, UDP_PORT))
        time.sleep(0.02)

if __name__ == "__main__":
    print("Vui lòng tháo cánh quạt hoặc nhấc drone khỏi mặt bàn trước khi test!")
    for m in range(1, 5):
        input(f"Nhấn Enter để chạy Motor M{m}...")
        test_single_motor(m)
    print("\nĐã test xong 4 động cơ.")