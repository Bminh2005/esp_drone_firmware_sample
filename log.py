import socket
import struct
import time

# Cấu hình địa chỉ IP và Cổng UDP
UDP_IP = "0.0.0.0"  # Lắng nghe tất cả các interface mạng
UDP_PORT = 8890

# Định dạng Unpack cho uav_state_t (64 Bytes):
# <  : Little-Endian
# 15f: 15 số thực float 32-bit (roll, pitch, yaw, roll_deg, pitch_deg, yaw_deg, gyro_x, gyro_y, gyro_z, x, y, z, vx, vy, vz)
# H  : 1 số nguyên uint16_t (dist_mm)
# B  : 1 số nguyên uint8_t (squal)
# ?  : 1 số boolean 8-bit (is_initialized)
STATE_STRUCT_FMT = "<15fHB?"
STRUCT_SIZE = struct.calcsize(STATE_STRUCT_FMT)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    print(f"[UDP RX] Đã mở cổng {UDP_PORT}, chờ dữ liệu uav_state_t (Size: {STRUCT_SIZE} bytes)...")

    try:
        while True:
            data, addr = sock.recvfrom(1024)
            
            if len(data) == STRUCT_SIZE:
                # Giải mã dữ liệu nhị phân
                unpacked = struct.unpack(STATE_STRUCT_FMT, data)
                
                state = {
                    "roll": unpacked[0],
                    "pitch": unpacked[1],
                    "yaw": unpacked[2],
                    "roll_deg": unpacked[3],
                    "pitch_deg": unpacked[4],
                    "yaw_deg": unpacked[5],
                    "gyro_x": unpacked[6],
                    "gyro_y": unpacked[7],
                    "gyro_z": unpacked[8],
                    "x": unpacked[9],
                    "y": unpacked[10],
                    "z": unpacked[11],
                    "vx": unpacked[12],
                    "vy": unpacked[13],
                    "vz": unpacked[14],
                    "dist_mm": unpacked[15],
                    "squal": unpacked[16],
                    "is_initialized": unpacked[17]
                }
                
                # In thông số ra màn hình console
                print(f"[{addr[0]}:{addr[1]}] Z:{state['z']:.2f}m | "
                      f"Att(P/R/Y): {state['pitch_deg']:.1f}°, {state['roll_deg']:.1f}°, {state['yaw_deg']:.1f}° | "
                      f"Vel(X/Y/Z): ({state['vx']:.2f}, {state['vy']:.2f}, {state['vz']:.2f})m/s | "
                      f"ToF: {state['dist_mm']}mm | SQUAL: {state['squal']}")
            else:
                print(f"[CẢNH BÁO] Nhận gói tin sai kích thước: {len(data)} bytes (kỳ vọng {STRUCT_SIZE} bytes)")

    except KeyboardInterrupt:
        print("\n[UDP RX] Đã dừng chương trình.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()