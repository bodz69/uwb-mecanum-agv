# -*- coding: utf-8 -*-
# UWB + odometry localization and waypoint control for a Mecanum robot.

import socket
import json
import math
import time
import threading
import sys
import numpy as np
from PyQt6 import QtWidgets, QtCore
from gui_app import RobotGUI

# ==========================================================
# 1. CẤU HÌNH HỆ THỐNG VÀ KINEMATICS
# ==========================================================
try:
    from config import (
        ESP32_IP, UDP_PORT_CMD, UDP_PORT_ODOM, UDP_PORT_UWB,
        WHEEL_DIAMETER, SCALE_FACTOR, D_KINEMATICS, ANCHORS,
    )
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Thiếu software/config.py. Hãy sao chép config.example.py thành "
        "config.py rồi cập nhật IP và tọa độ anchor."
    ) from exc

CIRCUMFERENCE = math.pi * WHEEL_DIAMETER

# ==========================================================
# 2. BỘ LỌC KALMAN 2D VÀ BIẾN TOÀN CỤC 
# ==========================================================
kf_lock = threading.Lock()
kf_state = np.array([[0.8], [0.8]]) 
kf_P = np.eye(2) * 0.01

kf_Q_move  = np.eye(2) * 0.01   
kf_Q_still = np.eye(2) * 0.001   
kf_R_move  = np.eye(2) * 1.0  
kf_R_still = np.eye(2) * 0.1

kf_H = np.eye(2)                  

# CÁC BIẾN CHO 3 CHẾ ĐỘ ĐỊNH VỊ (0: Odom, 1: UWB, 2: Kalman)
current_mode = 2
pure_odom_x, pure_odom_y = 0.8, 0.8

current_yaw_imu = 0.0
last_odom_time = None
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

is_moving = False      
stop_start_time = 0
has_moved_ever = False   

uwb_lpf_state = None     
ALPHA_UWB = 0.5      
uwb_stop_array = []      
last_uwb_running_update = time.time()
last_uwb_stopped_update = time.time()

# =========================================================
# CÁC HÀM CƠ BẢN VÀ KALMAN FILTER
# =========================================================
def set_initial_position(x, y, yaw):
    global kf_state, kf_P, has_moved_ever, stop_start_time, current_yaw_imu
    global pure_odom_x, pure_odom_y, uwb_lpf_state
    with kf_lock:
        kf_state = np.array([[x], [y]])
        kf_P = np.eye(2) * 0.0000001
        has_moved_ever = False  
        stop_start_time = time.time()
        current_yaw_imu = yaw
        
        # Reset luôn tọa độ Odom thuần và UWB thuần
        pure_odom_x, pure_odom_y = x, y
        uwb_lpf_state = [x, y]

def set_operating_mode(idx):
    global current_mode
    current_mode = idx
    modes = ["Odometry Thuần", "UWB Thuần", "Kalman Filter"]
    print(f"PC: Chuyển sang chế độ định vị -> {modes[idx]}")

def get_kf_pose():
    # Trả về tọa độ tùy theo chế độ đang được chọn trên GUI
    with kf_lock:
        if current_mode == 0:   # Odometry thuần
            return pure_odom_x, pure_odom_y, current_yaw_imu
        elif current_mode == 1: # UWB thuần
            ux = uwb_lpf_state[0] if uwb_lpf_state else 0.8
            uy = uwb_lpf_state[1] if uwb_lpf_state else 0.8
            return float(ux), float(uy), current_yaw_imu
        else:                   # Kalman Filter (Mặc định)
            return float(kf_state[0, 0]), float(kf_state[1, 0]), current_yaw_imu

def kalman_predict(dx, dy, Q_matrix):
    global kf_state, kf_P
    u = np.array([[dx], [dy]])
    with kf_lock:
        kf_state = kf_state + u
        kf_P = kf_P + Q_matrix

def kalman_update(z, R_matrix):
    global kf_state, kf_P
    with kf_lock:
        y = z - kf_H @ kf_state
        S = kf_H @ kf_P @ kf_H.T + R_matrix
        K = kf_P @ kf_H.T @ np.linalg.inv(S)
        kf_state = kf_state + K @ y
        kf_P = (np.eye(2) - K @ kf_H) @ kf_P

def trilateration(distances):
    try:
        x1, y1 = ANCHORS["1782"]; x2, y2 = ANCHORS["1783"]; x3, y3 = ANCHORS["1784"]
        r1, r2, r3 = distances["1782"], distances["1783"], distances["1784"]
        A = 2*(x2-x1); B = 2*(y2-y1); C = r1**2-r2**2-x1**2+x2**2-y1**2+y2**2
        D = 2*(x3-x1); E = 2*(y3-y1); F = r1**2-r3**2-x1**2+x3**2-y1**2+y3**2
        denom = A*E - B*D
        if denom == 0: return None
        return (C*E - F*B)/denom, (A*F - D*C)/denom
    except: return None

def wrap180(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle

# =========================================================
# 3. LẮNG NGHE LỌC UWB VÀ TÍNH TRUNG VỊ
# =========================================================
def landmark_listener():
    global uwb_lpf_state, uwb_stop_array
    global last_uwb_running_update, last_uwb_stopped_update
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT_UWB))
    sock.setblocking(0) 
    
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            obj = json.loads(data.decode("utf-8").strip())
            raw_distances = {item["A"]: float(item["R"]) for item in obj.get("links", []) if item["A"] in ANCHORS}
            if len(raw_distances) == 3:
                pos = trilateration(raw_distances)
                if pos:
                    now = time.time()

                    if is_moving:
                        if now - last_uwb_running_update >= 1.0:
                            last_uwb_running_update = now
                            kalman_update(np.array([[pos[0]], [pos[1]]]), kf_R_move)
                        
                        uwb_lpf_state = list(pos)
                    else:
                        if uwb_lpf_state is None:
                            uwb_lpf_state = list(pos)
                        else:
                            uwb_lpf_state[0] = ALPHA_UWB * pos[0] + (1 - ALPHA_UWB) * uwb_lpf_state[0]
                            uwb_lpf_state[1] = ALPHA_UWB * pos[1] + (1 - ALPHA_UWB) * uwb_lpf_state[1]
                        
                        if not has_moved_ever:
                            uwb_stop_array.clear()
                        else:
                            time_stopped = now - stop_start_time
                            if time_stopped < 2.0:
                                uwb_stop_array.clear()
                            else:
                                uwb_stop_array.append(tuple(uwb_lpf_state))
                                if now - stop_start_time >= 3.0:
                                    if now - last_uwb_stopped_update >= 1.0:
                                        last_uwb_stopped_update = now
                                        
                                        if len(uwb_stop_array) > 0:
                                            xs = [p[0] for p in uwb_stop_array]
                                            ys = [p[1] for p in uwb_stop_array]
                                            
                                            med_x = np.median(xs)
                                            med_y = np.median(ys)
                                            
                                            kalman_update(np.array([[med_x], [med_y]]), kf_R_still)
                                            uwb_stop_array.clear()
                                    
        except BlockingIOError: pass
        except: pass
        time.sleep(0.005)

# =========================================================
# 4. LẮNG NGHE ODOMETRY VÀ XÁC ĐỊNH TRẠNG THÁI
# =========================================================
def odometry_listener():
    global last_odom_time, current_yaw_imu
    global is_moving, stop_start_time, uwb_stop_array, last_uwb_stopped_update, has_moved_ever
    global pure_odom_x, pure_odom_y
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT_ODOM))
    sock.setblocking(0) 
    
    while True:
        try:
            last_data = None
            while True:
                try:
                    data, _ = sock.recvfrom(1024)
                    last_data = data
                except BlockingIOError:
                    break
            
            if last_data:
                msg = last_data.decode("utf-8").strip()
                parts = msg.split(',')
                if len(parts) == 5:
                    rpm1, rpm2, rpm3, rpm4, yaw = map(float, parts)
                    
                    now = time.time()
                    if last_odom_time is None:
                        last_odom_time = now
                        continue

                    dt = now - last_odom_time
                    last_odom_time = now
                    if dt <= 0 or dt > 0.2: continue

                    current_yaw_imu = yaw 

                    v1 = rpm1 * CIRCUMFERENCE / 60.0
                    v2 = rpm2 * CIRCUMFERENCE / 60.0
                    v3 = rpm3 * CIRCUMFERENCE / 60.0
                    v4 = rpm4 * CIRCUMFERENCE / 60.0

                    Vx = ((v1 + v2 + v3 + v4) / 4.0) * SCALE_FACTOR
                    Vy = ((v1 - v2 + v3 - v4) / 4.0) * SCALE_FACTOR
                    
                    speed = math.hypot(Vx, Vy)
                    current_moving = speed > 0.02 
                    
                    if current_moving:
                        has_moved_ever = True 
                        
                    if is_moving and not current_moving:
                        stop_start_time = now
                        last_uwb_stopped_update = now + 2.0 
                        uwb_stop_array.clear()        
                        
                    is_moving = current_moving
                    
                    Q_matrix = kf_Q_move if is_moving else kf_Q_still

                    theta = math.pi / 2 - math.radians(yaw)
                    dx = (Vx * math.cos(theta) + Vy * math.sin(theta)) * dt
                    dy = (Vx * math.sin(theta) - Vy * math.cos(theta)) * dt
                    
                    # 1. Tích lũy Odometry thuần
                    pure_odom_x += dx
                    pure_odom_y += dy
                    
                    # 2. Cập nhật Kalman Predict
                    kalman_predict(dx, dy, Q_matrix)
            
            time.sleep(0.005)
        except: pass

# ==========================================================
# 5. HỆ THỐNG PID & TỰ HÀNH
# ==========================================================
class SpacePID:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral, self.last_error = 0.0, 0.0

    def compute(self, error, dt):
        if dt <= 0.0: return 0.0
        Pout = self.kp * error
        self.integral += error * dt
        Iout = self.ki * self.integral
        if self.ki > 0:
            if Iout > self.out_max: Iout, self.integral = self.out_max, self.out_max / self.ki
            elif Iout < self.out_min: Iout, self.integral = self.out_min, self.out_min / self.ki
        Dout = self.kd * (error - self.last_error) / dt
        self.last_error = error
        return max(self.out_min, min(self.out_max, Pout + Iout + Dout))

    def reset(self):
        self.integral, self.last_error = 0.0, 0.0

pid_x  = SpacePID(10.0, 4.0, 0.0, -0.25, 0.25)
pid_y  = SpacePID(15.0, 10.0, 0.0, -0.25, 0.25)
pid_th = SpacePID(8.0, 5.0, 0.0, -1.0, 1.0)

path_queue = []
is_auto_mode = False

def receive_path_from_gui(raw_waypoints):
    global path_queue, is_auto_mode
    path_queue.clear()
    pid_x.reset(); pid_y.reset(); pid_th.reset()
    
    for i in range(len(raw_waypoints)):
        pt = raw_waypoints[i]
        path_queue.append({"x": pt[0], "y": pt[1], "th": pt[2]})
        
    is_auto_mode = True
    print(f"PC: Bắt đầu Tự hành - Nhận {len(path_queue)} điểm!")

def send_manual_cmd(cmd_char):
    global is_auto_mode, path_queue
    is_auto_mode = False
    path_queue.clear()
    try: cmd_sock.sendto(cmd_char.encode('utf-8'), (ESP32_IP, UDP_PORT_CMD))
    except: pass

def agv_control_loop(gui_signal):
    last_pid_time = time.time()
    last_gui_update = time.time() # Biến quản lý tốc độ vẽ đồ thị
    
    while True:
        try:
            now = time.time()
            dt = now - last_pid_time
            x_kf, y_kf, yaw_imu = get_kf_pose()
            
            errX, errY = 0.0, 0.0 # Khởi tạo mặc định
            
            if is_auto_mode and len(path_queue) > 0 and dt >= 0.1:
                last_pid_time = now
                target = path_queue[0]
                
                errX = target["x"] - x_kf
                errY = target["y"] - y_kf
                
                dist = math.hypot(errX, errY)
                errTh_abs = abs(wrap180(target["th"] - yaw_imu))
                
                # --- ĐIỀU KIỆN DỪNG ---
                if len(path_queue) == 1:
                    # Điểm cuối, cần cực kì chính xác (<5cm)
                    if dist < 0.05 and errTh_abs < 2.0:
                        path_queue.pop(0)
                        try: cmd_sock.sendto(b"M,0.0,0.0,0.0,0.0", (ESP32_IP, UDP_PORT_CMD))
                        except: pass
                        
                        send_manual_cmd('S')
                        print("--> Tới đích cuối cùng. Dừng 0.5s...")
                        time.sleep(0.5) 
                        pid_x.reset(); pid_y.reset(); pid_th.reset()
                        last_pid_time = time.time()
                        continue
                else:
                    # Chấp nhận điểm (Với cấu hình cách nhau 5cm hiện tại)
                    if dist < 0.04:
                        path_queue.pop(0)
                        continue
                
                errTh = wrap180(target["th"] - yaw_imu)
                
                rad = math.radians(yaw_imu)
                r_errX = errX * math.sin(rad) + errY * math.cos(rad) 
                r_errY = errX * math.cos(rad) - errY * math.sin(rad) 
                
                vx = pid_x.compute(r_errX, dt)
                vy = pid_y.compute(r_errY, dt)
                w  = pid_th.compute(errTh*(math.pi/180.0), dt)
                
                v1 = vx + vy + w * D_KINEMATICS
                v2 = vx - vy + w * D_KINEMATICS
                v3 = vx + vy - w * D_KINEMATICS
                v4 = vx - vy - w * D_KINEMATICS
                
                cmd = f"M,{(v1/CIRCUMFERENCE)*60:.1f},{(v2/CIRCUMFERENCE)*60:.1f},{(v3/CIRCUMFERENCE)*60:.1f},{(v4/CIRCUMFERENCE)*60:.1f}"
                try: cmd_sock.sendto(cmd.encode('utf-8'), (ESP32_IP, UDP_PORT_CMD))
                except: pass

            # Gửi tín hiệu vẽ GUI giới hạn tốc độ ở mức ~20Hz (mỗi 0.05s)
            if now - last_gui_update >= 0.05:
                gui_signal.emit(float(x_kf), float(y_kf), float(yaw_imu), float(errX), float(errY))
                last_gui_update = now

            time.sleep(0.01)
        except Exception as e:
            print(f"Lỗi Thread PID: {e}")
            time.sleep(0.1)

# ==========================================================
# 6. KHỞI CHẠY GIAO DIỆN
# ==========================================================
class GuiUpdateSignal(QtCore.QObject):
    # Cập nhật nhận 5 biến
    update_sig = QtCore.pyqtSignal(object, object, object, object, object)

def main():
    app = QtWidgets.QApplication(sys.argv)
    
    win = RobotGUI(
        anchors=ANCHORS, 
        set_pos_cb=set_initial_position, 
        send_cmd_cb=send_manual_cmd,
        set_mode_cb=set_operating_mode 
    )
    
    win.path_generated_sig.connect(receive_path_from_gui)
    
    gui_notifier = GuiUpdateSignal()
    gui_notifier.update_sig.connect(win.update_robot_pose)
    
    threading.Thread(target=landmark_listener, daemon=True).start()
    threading.Thread(target=odometry_listener, daemon=True).start()
    threading.Thread(target=agv_control_loop, args=(gui_notifier.update_sig,), daemon=True).start()
    
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__": 
    main()