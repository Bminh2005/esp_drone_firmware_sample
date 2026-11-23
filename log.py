import socket

# Cấu hình UDP Listener
UDP_IP = "0.0.0.0"  # Lắng nghe trên tất cả card mạng
UDP_PORT = 8889

def listen_telemetry():
    # Tạo Socket UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    print(f"[*] Đang lắng nghe Telemetry từ ESP32 tại cổng UDP {UDP_PORT}...")

    while True:
        try:
            # Nhận dữ liệu thô từ socket
            data, addr = sock.recvfrom(1024)
            line = data.decode('utf-8', errors='ignore').strip()

            # Kiểm tra khung gói tin chuẩn /* ... */
            if line.startswith("/*") and line.endswith("*/"):
                # Cắt bỏ 2 ký tự đầu '/*' và 2 ký tự cuối '*/'
                raw_str = line[2:-2]
                items = raw_str.split(',')

                # Đảm bảo đủ 10 thông số
                if len(items) == 10:
                    roll  = float(items[0])
                    pitch = float(items[1])
                    yaw   = float(items[2])
                    x     = float(items[3])
                    y     = float(items[4])
                    z     = float(items[5])
                    vx    = float(items[6])
                    vy    = float(items[7])
                    vz    = float(items[8])
                    squal = int(float(items[9]))

                    # === TRUỶ XUẤT VÀ SỬ DỤNG BIẾN THÁI DỰ ÁN TẠI ĐÂY ===
                    print(f"[{addr[0]}]X={x:.3f}m Y={y:.3f}m Z={z:.3f}m | Roll={roll:.1f}° Pitch={pitch:.1f}° Yaw={yaw:.1f}° | Flow Quality={squal}")

        except KeyboardInterrupt:
            print("\nĐã dừng lắng nghe Telemetry.")
            break
        except Exception as e:
            print(f"Lỗi parse dữ liệu: {e}")

    sock.close()

if __name__ == "__main__":
    listen_telemetry()