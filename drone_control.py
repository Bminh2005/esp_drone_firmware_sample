import socket
import struct
import time
import threading
import sys

# === CẤU HÌNH UDP ===
ESP32_IP = "192.168.4.1"
UDP_PORT = 8888

# === BIẾN TRẠNG THÁI TOÀN CỤC ===
g_vx = 0.0           # Vận tốc tiến/lùi (m/s)
g_vy = 0.0           # Vận tốc sang trái/phải (m/s)
g_yaw_rate = 0.0     # Tốc độ xoay (deg/s)
g_z_target = 0.0     # Độ cao mục tiêu (m)
g_armed = False      # Trạng thái mở/khóa động cơ
g_calib_req = 0      # Cờ yêu cầu Calib Gyro (1 = Có, 0 = Không)
g_running = True     # Cờ duy trì chương trình

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def udp_sender_thread():
    """
    Thread chạy ngầm ở nhịp 50Hz (20ms/gói) gửi liên tục gói tin 18-byte
    giúp ESP32 không bị ngắt Failsafe Watchdog (500ms).
    """
    global g_running, g_calib_req
    while g_running:
        # Gói tin 18-byte: 4 floats + 2 uint8
        payload = struct.pack(
            '<ffffBB',
            float(g_vx),
            float(g_vy),
            float(g_yaw_rate),
            float(g_z_target),
            1 if g_armed else 0,
            int(g_calib_req)
        )
        try:
            sock.sendto(payload, (ESP32_IP, UDP_PORT))
        except Exception as e:
            print(f"\n[LỖI GỬI UDP]: {e}")

        # Tự động hạ cờ Calib ngay sau khi đã gửi đi 1 lần
        if g_calib_req == 1:
            g_calib_req = 0

        time.sleep(0.02)  # 50Hz

def cmd_calib():
    global g_calib_req, g_armed
    if g_armed:
        print("[LỖI] Không thể Calib khi drone đang ARMED (đang quay motor)! Hãy nhập 'disarm' trước.")
        return
    
    print("[CALIB] Đã gửi lệnh Calib Gyro... Vui lòng GIỮ ĐỨNG YÊN DRONE trong 1.5 giây!")
    g_calib_req = 1
    time.sleep(1.5)
    print("[CALIB] Hoàn tất quá trình Calib Gyro!")

def cmd_arm():
    global g_armed, g_z_target, g_vx, g_vy, g_yaw_rate
    g_vx = 0.0
    g_vy = 0.0
    g_yaw_rate = 0.0
    g_z_target = 0.0
    g_armed = True
    print("[ARM] Đã mở động cơ! Motors đang chạy ở mức Idle.")

def cmd_disarm():
    global g_armed, g_z_target, g_vx, g_vy, g_yaw_rate
    g_armed = False
    g_z_target = 0.0
    g_vx = 0.0
    g_vy = 0.0
    g_yaw_rate = 0.0
    print("[DISARM] Đã ngắt toàn bộ động cơ an toàn!")

def cmd_takeoff(target_alt=0.4):
    global g_armed, g_z_target
    if not g_armed:
        print("[TAKEOFF] Drone chưa ARM! Tự động ARM động cơ...")
        cmd_arm()
        time.sleep(0.3)

    print(f"[TAKEOFF] Bắn thẳng (Pop-Up) lên độ cao {target_alt:.2f}m...")
    
    # BẮT BỘC: Đặt thẳng target_alt để PID dồn ga bứt nhanh khỏi sàn nhà trong 0.3s
    # Tránh bị mắc kẹt tích lũy sai số trong vùng Ground Effect (< 20cm)
    g_z_target = target_alt

    print(f"[TAKEOFF] Đã phát lệnh Pop-Up Z = {g_z_target:.2f}m. Đang lơ lửng PosHold.")
    
def cmd_land():
    global g_armed, g_z_target, g_vx, g_vy, g_yaw_rate
    if not g_armed or g_z_target <= 0.05:
        cmd_disarm()
        return

    print("[LAND] Đang tự động hạ cánh dịu nhẹ...")
    g_vx = 0.0
    g_vy = 0.0
    g_yaw_rate = 0.0

    # Giảm độ cao mượt từ độ cao hiện tại xuống 0.05m
    start_alt = g_z_target
    steps = 30
    for i in range(1, steps + 1):
        g_z_target = start_alt * (1.0 - (i / steps))
        if g_z_target < 0.05:
            g_z_target = 0.05
        time.sleep(0.1)

    time.sleep(0.5)
    cmd_disarm()
    print("[LAND] Đã hạ cánh chạm sàn thành công!")

def show_status():
    print("--------------------------------------------------")
    print(f" TRẠNG THÁI DRONE:")
    print(f"  • Armed     : {'ARMED (BẬT)' if g_armed else 'DISARMED (TẮT)'}")
    print(f"  • Z Target  : {g_z_target:.2f} m")
    print(f"  • Vx Target : {g_vx:.2f} m/s")
    print(f"  • Vy Target : {g_vy:.2f} m/s")
    print(f"  • Yaw Rate  : {g_yaw_rate:.2f} deg/s")
    print("--------------------------------------------------")

def print_help():
    print("""
==================================================
        BẢNG LỆNH ĐIỀU KHIỂN DRONE (CLI)
==================================================
  calib          : Thực hiện Calib Gyro động (Khi Disarm)
  arm            : Kích hoạt động cơ (Chạy mức Idle)
  disarm         : Khóa động cơ khẩn cấp (Ngắt ga ngay)
  takeoff [alt]  : Tự động cất cánh (Ví dụ: 'takeoff 0.5')
  land           : Tự động hạ cánh dịu nhẹ & Disarm
  
  move <vx> <vy> : Đặt vận tốc di chuyển (m/s)
                   Ví dụ: 'move 0.2 0' (Tiến), 'move 0 -0.2' (Trái)
  yaw <rate>     : Xoay góc Yaw (deg/s) (Ví dụ: 'yaw 30', 'yaw -30')
  alt <z>        : Thay đổi độ cao giữ trực tiếp (Ví dụ: 'alt 0.8')
  
  status         : Xem thông số cài đặt hiện tại
  help           : Hiển thị bảng hướng dẫn này
  exit / quit    : Hạ cánh và thoát chương trình
==================================================
""")

def main():
    global g_running, g_vx, g_vy, g_yaw_rate, g_z_target

    # Khai báo thread gửi UDP 50Hz
    t = threading.Thread(target=udp_sender_thread, daemon=True)
    t.start()

    print_help()

    try:
        while g_running:
            raw_input = input("drone_cli> ").strip().lower()
            if not raw_input:
                continue

            parts = raw_input.split()
            cmd = parts[0]

            if cmd == "calib":
                cmd_calib()

            elif cmd == "arm":
                cmd_arm()

            elif cmd in ["disarm", "stop"]:
                cmd_disarm()

            elif cmd == "takeoff":
                alt = 0.5
                if len(parts) > 1:
                    try:
                        alt = float(parts[1])
                    except ValueError:
                        print("[LỖI] Độ cao không hợp lệ! Dùng mặc định 0.5m.")
                cmd_takeoff(alt)

            elif cmd == "land":
                cmd_land()

            elif cmd == "move":
                if len(parts) >= 3:
                    try:
                        g_vx = float(parts[1])
                        g_vy = float(parts[2])
                        print(f"[MOVE] Đã đặt Vx = {g_vx:.2f} m/s, Vy = {g_vy:.2f} m/s")
                    except ValueError:
                        print("[LỖI] Cú pháp sai! Dùng: move <vx> <vy>")
                else:
                    print("[LỖI] Thiếu tham số! Dùng: move <vx> <vy>")

            elif cmd == "yaw":
                if len(parts) >= 2:
                    try:
                        g_yaw_rate = float(parts[1])
                        print(f"[YAW] Đã đặt Yaw Rate = {g_yaw_rate:.2f} deg/s")
                    except ValueError:
                        print("[LỖI] Cú pháp sai! Dùng: yaw <rate>")

            elif cmd == "alt":
                if len(parts) >= 2:
                    try:
                        g_z_target = float(parts[1])
                        print(f"[ALT] Đã chuyển độ cao mục tiêu Z = {g_z_target:.2f} m")
                    except ValueError:
                        print("[LỖI] Cú pháp sai! Dùng: alt <z>")

            elif cmd == "status":
                show_status()

            elif cmd == "help":
                print_help()

            elif cmd in ["exit", "quit"]:
                print("[EXIT] Đang hạ cánh an toàn trước khi thoát...")
                cmd_land()
                g_running = False
                break

            else:
                print(f"[LỖI] Lệnh '{cmd}' không hợp lệ. Gõ 'help' để xem danh sách lệnh.")

    except KeyboardInterrupt:
        print("\n[NGẮT KHẨN CẤP] Người dùng nhấn Ctrl+C!")
        cmd_disarm()
        g_running = False

    finally:
        g_running = False
        time.sleep(0.1)
        sock.close()
        print("Đã thoát chương trình CLI.")

if __name__ == "__main__":
    main()