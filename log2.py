import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 8889

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Đang lắng nghe UDP Telemetry 5 Tầng tại Port {UDP_PORT}...\n")

while True:
    try:
        data, addr = sock.recvfrom(1024)
        raw_str = data.decode('utf-8', errors='ignore').strip()

        if raw_str.startswith("/*") and raw_str.endswith("*/"):
            clean_str = raw_str[2:-2]
            parts = clean_str.split(',')

            # ĐỊNH DẠNG 24 THAM SỐ (Bao gồm Roll, Pitch, Yaw gốc)
            if len(parts) == 24:
                # Thái độ Euler hiện tại từ Sensor/State Estimator
                st_roll, st_pitch, st_yaw = float(parts[0]), float(parts[1]), float(parts[2])

                # Tầng 1: Altitude Hold (Z Pos / Vel)
                z_tgt, z_st, u_z = float(parts[3]), float(parts[4]), float(parts[5])
                
                # Tầng 2: PosHold Velocity -> Target Angle
                vx, vy = float(parts[6]), float(parts[7])
                tgt_pitch, tgt_roll = float(parts[8]), float(parts[9])
                
                # Tầng 3: Angle -> Target Rate
                roll, pitch = float(parts[10]), float(parts[11])
                tgt_r_roll, tgt_r_pitch = float(parts[12]), float(parts[13])
                
                # Tầng 4: Gyro Rate -> Torque Command
                gx, gy, gz = float(parts[14]), float(parts[15]), float(parts[16])
                u_roll, u_pitch, u_yaw = float(parts[17]), float(parts[18]), float(parts[19])
                
                # Tầng 5: Motor PWM (%)
                m1, m2, m3, m4 = float(parts[20]), float(parts[21]), float(parts[22]), float(parts[23])

                log_msg = (
                    f"[{addr[0]}] "
                    f"ATT:[R:{st_roll:+6.2f}° P:{st_pitch:+6.2f}° Y:{st_yaw:+6.2f}°] | "
                    f"T1_Z:[{z_st:.2f}/{z_tgt:.2f}m uZ:{u_z:+.2f}] | "
                    f"T2_Vel:[V:({vx:+.2f},{vy:+.2f})->TgtA:(P:{tgt_pitch:+.1f}°,R:{tgt_roll:+.1f}°)] | "
                    f"T3_Ang:[Att:(P:{pitch:+.1f}°,R:{roll:+.1f}°)->TgtR:(P:{tgt_r_pitch:+.1f},R:{tgt_r_roll:+.1f})] | "
                    f"T4_Rate:[G:({gx:+.1f},{gy:+.1f},{gz:+.1f}) U:(P:{u_pitch:+.2f},R:{u_roll:+.2f},Y:{u_yaw:+.2f})] | "
                    f"MOTORS:[M1:{m1:4.1f}% M2:{m2:4.1f}% M3:{m3:4.1f}% M4:{m4:4.1f}%]"
                )
                print(log_msg)

            # 1. Định dạng AUDIT 5 TẦNG MỚI (Đủ 21 tham số)
            if len(parts) == 21:
                # Tầng 1: Altitude Hold (Z Pos / Vel)
                z_tgt, z_st, u_z = float(parts[0]), float(parts[1]), float(parts[2])
                
                # Tầng 2: PosHold Velocity -> Target Angle
                vx, vy = float(parts[3]), float(parts[4])
                tgt_pitch, tgt_roll = float(parts[5]), float(parts[6])
                
                # Tầng 3: Angle -> Target Rate
                roll, pitch = float(parts[7]), float(parts[8])
                tgt_r_roll, tgt_r_pitch = float(parts[9]), float(parts[10])
                
                # Tầng 4: Gyro Rate -> Torque Command
                gx, gy, gz = float(parts[11]), float(parts[12]), float(parts[13])
                u_roll, u_pitch, u_yaw = float(parts[14]), float(parts[15]), float(parts[16])
                
                # Tầng 5: Motor PWM (%)
                m1, m2, m3, m4 = float(parts[17]), float(parts[18]), float(parts[19]), float(parts[20])

                log_msg = (
                    f"[{addr[0]}] "
                    f"T1_Z:[{z_st:.2f}/{z_tgt:.2f}m uZ:{u_z:+.2f}] | "
                    f"T2_Vel:[V:({vx:+.2f},{vy:+.2f})->TgtA:(P:{tgt_pitch:+.1f}°,R:{tgt_roll:+.1f}°)] | "
                    f"T3_Ang:[Att:(P:{pitch:+.1f}°,R:{roll:+.1f}°)->TgtR:(P:{tgt_r_pitch:+.1f},R:{tgt_r_roll:+.1f})] | "
                    f"T4_Rate:[G:({gx:+.1f},{gy:+.1f},{gz:+.1f}) U:(P:{u_pitch:+.2f},R:{u_roll:+.2f},Y:{u_yaw:+.2f})] | "
                    f"MOTORS:[M1:{m1:4.1f}% M2:{m2:4.1f}% M3:{m3:4.1f}% M4:{m4:4.1f}%]"
                )
                print(log_msg)

            # 2. Định dạng cũ (14 tham số)
            elif len(parts) == 14:
                roll, pitch, yaw = float(parts[0]), float(parts[1]), float(parts[2])
                x, y, z          = float(parts[3]), float(parts[4]), float(parts[5])
                vx, vy, vz       = float(parts[6]), float(parts[7]), float(parts[8])
                squal            = int(parts[9])
                m1, m2, m3, m4   = float(parts[10]), float(parts[11]), float(parts[12]), float(parts[13])

                print(f"[{addr[0]}] ATT:[R:{roll:6.2f}° P:{pitch:6.2f}° Y:{yaw:6.2f}°] | POS:[Z:{z:5.3f}m] | MOTORS:[M1:{m1:.1f}% M2:{m2:.1f}% M3:{m3:.1f}% M4:{m4:.1f}%]")

            # 3. Định dạng cũ (10 tham số)
            elif len(parts) == 10:
                roll, pitch, yaw = float(parts[0]), float(parts[1]), float(parts[2])
                x, y, z          = float(parts[3]), float(parts[4]), float(parts[5])
                vx, vy, vz       = float(parts[6]), float(parts[7]), float(parts[8])
                squal            = int(parts[9])

                print(f"[{addr[0]}] ATT:[R:{roll:6.2f}° P:{pitch:6.2f}° Y:{yaw:6.2f}°] | POS:[Z:{z:5.3f}m] | Flow SQUAL={squal}")

    except KeyboardInterrupt:
        print("\n[INFO] Đã dừng chương trình nhận log.")
        break
    except Exception as e:
        # Bỏ qua các gói dữ liệu rác không đúng cấu trúc
        pass

sock.close()