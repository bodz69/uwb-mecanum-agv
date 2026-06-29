# -*- coding: utf-8 -*-
import sys
import math
import time
import numpy as np
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg

# =========================================================
# WIDGET BẢN ĐỒ TÙY CHỈNH: HỖ TRỢ VẼ VÀ NỘI SUY
# =========================================================
class CustomPlotWidget(pg.PlotWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.is_drawing = False
        self.gui_parent = None 
        self.last_pt = None
        self.step = 0.1  # Nội suy vẽ tay: 10 cm mỗi điểm

    def mousePressEvent(self, ev):
        if self.gui_parent and self.gui_parent.draw_mode_cb.isChecked():
            if ev.button() == QtCore.Qt.MouseButton.LeftButton:
                self.is_drawing = True
                scene_pos = self.mapToScene(ev.pos())
                pos = self.plotItem.vb.mapSceneToView(scene_pos)
                x, y = round(pos.x(), 3), round(pos.y(), 3)
                if self.is_point_in_map(x, y):
                    self.last_pt = np.array([x, y])
                    self.gui_parent.add_point_to_list(x, y, 0.0) 
                ev.accept()
                return
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        if self.is_drawing and self.gui_parent and self.gui_parent.draw_mode_cb.isChecked():
            scene_pos = self.mapToScene(ev.pos())
            pos = self.plotItem.vb.mapSceneToView(scene_pos)
            curr_pt = np.array([pos.x(), pos.y()])
            
            if self.last_pt is not None:
                v = curr_pt - self.last_pt
                dist = np.hypot(v[0], v[1])
                if dist >= self.step:
                    n_pts = int(np.floor(dist / self.step))
                    direction = v / dist
                    for i in range(1, n_pts + 1):
                        q = self.last_pt + direction * (i * self.step)
                        self.gui_parent.add_point_to_list(round(q[0], 3), round(q[1], 3), 0.0)
                    self.last_pt = self.last_pt + direction * (n_pts * self.step)
            ev.accept()
            return
        super().mouseMoveEvent(ev)

    def mouseReleaseEvent(self, ev):
        if self.is_drawing and ev.button() == QtCore.Qt.MouseButton.LeftButton:
            self.is_drawing = False
            self.last_pt = None
            ev.accept()
            return
        super().mouseReleaseEvent(ev)


class RobotGUI(QtWidgets.QMainWindow):
    path_generated_sig = QtCore.pyqtSignal(list)

    def __init__(self, anchors, set_pos_cb, send_cmd_cb, set_mode_cb=None, window_title="AGV Mecanum - Tuyến Tính Hóa N Điểm"):
        super().__init__()
        self.anchors = anchors
        xs = [p[0] for p in anchors.values()] if anchors else [0.0, 3.2]
        ys = [p[1] for p in anchors.values()] if anchors else [0.0, 3.2]
        margin = 0.2
        self.map_x_min = min(0.0, min(xs)) - margin
        self.map_x_max = max(xs) + margin
        self.map_y_min = min(0.0, min(ys)) - margin
        self.map_y_max = max(ys) + margin
        self.set_pos_cb = set_pos_cb
        self.send_cmd_cb = send_cmd_cb
        self.set_mode_cb = set_mode_cb 
        self.default_pid = (10.0, 4.0, 0.0) 
        
        self.raw_waypoints = []
        self.robot_trace_data = [] 
        self.interp_step = 0.1  # Bước nội suy: 10 cm
        self.last_robot_x = 0.8
        self.last_robot_y = 0.8
        
        # Dữ liệu cho đồ thị sai số
        self.start_time = time.time()
        self.time_data = []
        self.err_x_data = []
        self.err_y_data = []

        self.setWindowTitle(window_title)
        self.resize(1500, 850) # Mở rộng bề ngang để chứa đồ thị
        self.init_ui()

    def init_ui(self):
        central = QtWidgets.QWidget()
        main_layout = QtWidgets.QHBoxLayout() 
        
        # ================= CỘT 1: BẢNG ĐIỀU KHIỂN =================
        control_panel = QtWidgets.QWidget()
        control_layout = QtWidgets.QVBoxLayout()
        control_panel.setFixedWidth(320)

        # 0. CHỌN CHẾ ĐỘ
        mode_box = QtWidgets.QGroupBox("0. Chọn Chế Độ Định Vị")
        mode_layout = QtWidgets.QVBoxLayout()
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["1. Chạy hoàn toàn Odometry", "2. Chạy hoàn toàn UWB", "3. Chạy chuẩn Kalman Filter"])
        self.mode_combo.setCurrentIndex(2) 
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)
        mode_layout.addWidget(self.mode_combo)
        mode_box.setLayout(mode_layout)
        control_layout.addWidget(mode_box)
        
        # 1. KHỞI TẠO VỊ TRÍ
        start_box = QtWidgets.QGroupBox("1. Khởi tạo vị trí (KF)")
        start_layout = QtWidgets.QGridLayout()
        self.x_start = QtWidgets.QLineEdit("0.8")
        self.y_start = QtWidgets.QLineEdit("0.8")
        self.yaw_start = QtWidgets.QLineEdit("0.0")
        self.set_start_btn = QtWidgets.QPushButton("Xác Nhận Vị Trí")
        self.set_start_btn.clicked.connect(self.on_set_pos_clicked)
        start_layout.addWidget(QtWidgets.QLabel("X (m):"), 0, 0)
        start_layout.addWidget(self.x_start, 0, 1)
        start_layout.addWidget(QtWidgets.QLabel("Y (m):"), 1, 0)
        start_layout.addWidget(self.y_start, 1, 1)
        start_layout.addWidget(QtWidgets.QLabel("Yaw (°):"), 2, 0)
        start_layout.addWidget(self.yaw_start, 2, 1)
        start_layout.addWidget(self.set_start_btn, 3, 0, 1, 2)
        start_box.setLayout(start_layout)
        control_layout.addWidget(start_box)
        
        # 2. QUẢN LÝ QUỸ ĐẠO
        path_box = QtWidgets.QGroupBox("2. Quản lý quỹ đạo (nội suy 10 cm)")
        path_layout = QtWidgets.QVBoxLayout()
        
        # NÚT SINH HÌNH VUÔNG
        square_layout = QtWidgets.QHBoxLayout()
        self.L_input = QtWidgets.QLineEdit("1.0") 
        self.btn_square = QtWidgets.QPushButton("TẠO HÌNH VUÔNG")
        self.btn_square.setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold;")
        self.btn_square.clicked.connect(self.generate_square_path)
        square_layout.addWidget(QtWidgets.QLabel("Cạnh a(m):"))
        square_layout.addWidget(self.L_input)
        square_layout.addWidget(self.btn_square)
        path_layout.addLayout(square_layout)

        # NÚT SINH HÌNH TRÒN (THÊM MỚI)
        circle_layout = QtWidgets.QHBoxLayout()
        self.R_input = QtWidgets.QLineEdit("0.8") 
        self.btn_circle = QtWidgets.QPushButton("TẠO HÌNH TRÒN")
        self.btn_circle.setStyleSheet("background-color: #E91E63; color: white; font-weight: bold;")
        self.btn_circle.clicked.connect(self.generate_circle_path)
        circle_layout.addWidget(QtWidgets.QLabel("Bán kính R(m):"))
        circle_layout.addWidget(self.R_input)
        circle_layout.addWidget(self.btn_circle)
        path_layout.addLayout(circle_layout)

        self.draw_mode_cb = QtWidgets.QCheckBox("Bật Chế Độ Vẽ Tay (Giữ Chuột)")
        self.draw_mode_cb.stateChanged.connect(self.on_draw_mode_toggled)
        self.waypoint_list_widget = QtWidgets.QListWidget()
        self.waypoint_list_widget.setFixedHeight(90) # Thu nhỏ list một xíu cho vừa 2 nút
        self.clear_path_btn = QtWidgets.QPushButton("Xóa Quỹ Đạo")
        self.clear_path_btn.clicked.connect(self.clear_path)
        self.send_path_btn = QtWidgets.QPushButton("CHẠY TỰ HÀNH N-ĐIỂM")
        self.send_path_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        self.send_path_btn.clicked.connect(self.send_path_to_main)
        
        path_layout.addWidget(self.draw_mode_cb)
        path_layout.addWidget(QtWidgets.QLabel("Danh sách N điểm:"))
        path_layout.addWidget(self.waypoint_list_widget)
        path_layout.addWidget(self.clear_path_btn)
        path_layout.addWidget(self.send_path_btn)
        path_box.setLayout(path_layout)
        control_layout.addWidget(path_box)

        # 3. ĐIỂM ĐƠN 
        num_box = QtWidgets.QGroupBox("3. Tạo Đường Thẳng Bằng Số")
        num_layout = QtWidgets.QGridLayout()
        self.x_in = QtWidgets.QLineEdit("1.5")
        self.y_in = QtWidgets.QLineEdit("1.5")
        self.yaw_in = QtWidgets.QLineEdit("0.0")
        self.add_btn = QtWidgets.QPushButton("TẠO N-ĐIỂM TỚI ĐÍCH NÀY")
        self.add_btn.setStyleSheet("background-color: #2196F3; color: white;")
        self.add_btn.clicked.connect(self.on_add_numeric_interp)
        num_layout.addWidget(QtWidgets.QLabel("X đích:"), 0, 0)
        num_layout.addWidget(self.x_in, 0, 1)
        num_layout.addWidget(QtWidgets.QLabel("Y đích:"), 1, 0)
        num_layout.addWidget(self.y_in, 1, 1)
        num_layout.addWidget(QtWidgets.QLabel("Yaw đích:"), 2, 0)
        num_layout.addWidget(self.yaw_in, 2, 1)
        num_layout.addWidget(self.add_btn, 3, 0, 1, 2)
        num_box.setLayout(num_layout)
        control_layout.addWidget(num_box)

        # DỪNG KHẨN CẤP
        self.estop_btn = QtWidgets.QPushButton("DỪNG KHẨN CẤP")
        self.estop_btn.setStyleSheet("background-color: #F44336; color: white; font-weight: bold; padding: 10px;")
        self.estop_btn.clicked.connect(self.on_emergency_stop)
        control_layout.addWidget(self.estop_btn)
        
        self.lbl_status = QtWidgets.QLabel("X: -- | Y: -- | Yaw: --")
        control_layout.addWidget(self.lbl_status)
        control_layout.addStretch()
        control_panel.setLayout(control_layout)
        main_layout.addWidget(control_panel, stretch=1)

        # ================= CỘT 2: BẢN ĐỒ =================
        self.plot = CustomPlotWidget()
        self.plot.gui_parent = self
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.5)
        self.plot.setXRange(self.map_x_min, self.map_x_max, padding=0)
        self.plot.setYRange(self.map_y_min, self.map_y_max, padding=0)
        
        for aid, (ax, ay) in self.anchors.items():
            self.plot.plot([ax], [ay], pen=None, symbol='t', symbolBrush='r', symbolSize=15)
            text = pg.TextItem(aid, anchor=(0.5, 2), color='r')
            text.setPos(ax, ay)
            self.plot.addItem(text)
            
        # Vẽ nét liền màu đen có mũi tên ngầm định (Dots)
        self.draw_path_line = self.plot.plot([], [], pen=pg.mkPen('k', width=2, style=QtCore.Qt.PenStyle.SolidLine))
        self.draw_path_dots = self.plot.plot([], [], pen=None, symbol='o', symbolBrush=(0, 0, 0, 150), symbolSize=5)
        
        # Đánh dấu Start (Xanh) và End (Đỏ)
        self.path_start_marker = pg.ScatterPlotItem(size=14, brush='g', symbol='star')
        self.path_end_marker = pg.ScatterPlotItem(size=14, brush='r', symbol='x')
        self.plot.addItem(self.path_start_marker)
        self.plot.addItem(self.path_end_marker)

        self.robot_trace = self.plot.plot([], [], pen=pg.mkPen('r', width=1))
        self.robot_current_dot = pg.ScatterPlotItem(size=15, brush='r', symbol='o')
        self.plot.addItem(self.robot_current_dot)
        
        self.plot.scene().sigMouseClicked.connect(self.on_map_clicked)
        main_layout.addWidget(self.plot, stretch=2)

        # ================= CỘT 3: ĐỒ THỊ SAI SỐ =================
        right_panel = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout()
        
        self.plot_err_x = pg.PlotWidget(title="Sai số trục X (m)")
        self.plot_err_x.setBackground('w') 
        self.plot_err_x.showGrid(x=True, y=True, alpha=0.3)
        self.plot_err_x.setLabel('left', 'Error X', units='m')
        self.line_err_x = self.plot_err_x.plot([], [], pen=pg.mkPen('k', width=1.5)) 
        
        self.plot_err_y = pg.PlotWidget(title="Sai số trục Y (m)")
        self.plot_err_y.setBackground('w') 
        self.plot_err_y.showGrid(x=True, y=True, alpha=0.3)
        self.plot_err_y.setLabel('left', 'Error Y', units='m')
        self.plot_err_y.setLabel('bottom', 'Time', units='s')
        self.line_err_y = self.plot_err_y.plot([], [], pen=pg.mkPen('k', width=1.5)) 

        right_layout.addWidget(self.plot_err_x)
        right_layout.addWidget(self.plot_err_y)
        right_panel.setLayout(right_layout)
        main_layout.addWidget(right_panel, stretch=1)

        central.setLayout(main_layout)
        self.setCentralWidget(central)

    # ================= CÁC HÀM XỬ LÝ =================
    
    # --- THÊM HÀM TẠO HÌNH TRÒN ---
    def generate_circle_path(self):
        self.clear_path()
        try:
            R = float(self.R_input.text())
        except:
            R = 0.8

        # 1. Tự động tính tâm Map từ Anchor
        if self.anchors:
            xs = [pos[0] for pos in self.anchors.values()]
            ys = [pos[1] for pos in self.anchors.values()]
            cx = (min(xs) + max(xs)) / 2.0
            cy = (min(ys) + max(ys)) / 2.0
        else:
            cx, cy = 1.6, 1.6
            
        # 2. Tìm góc hướng về Anchor 1 (0,0) làm điểm bắt đầu
        start_angle = math.atan2(0 - cy, 0 - cx)
        
        # 3. Tính số điểm dựa trên chu vi và bước nội suy 20cm
        circumference = 2 * math.pi * R
        num_points = int(circumference / self.interp_step)
        
        if num_points <= 0: return

        # Tạo mảng góc chạy 1 vòng tròn
        angles = np.linspace(start_angle, start_angle + 2 * math.pi, num_points, endpoint=False)
        
        for theta in angles:
            x = cx + R * math.cos(theta)
            y = cy + R * math.sin(theta)
            # Yaw luôn là 0.0 theo yêu cầu tịnh tiến của Mecanum
            self.add_point_to_list(round(x, 3), round(y, 3), 0.0)

        # Chốt điểm cuối cùng để khép kín hình tròn hoàn hảo
        if len(self.raw_waypoints) > 0:
            first_pt = self.raw_waypoints[0]
            self.add_point_to_list(first_pt[0], first_pt[1], first_pt[2])
            
        QtWidgets.QMessageBox.information(self, "Thành công", f"Đã tạo Hình Tròn (Tâm: {cx},{cy}, R={R}m)\nXe sẽ giữ góc 0° và tịnh tiến theo vòng tròn.\nTổng: {len(self.raw_waypoints)} điểm.")

    def generate_square_path(self):
        self.clear_path()
        try:
            L = float(self.L_input.text())
        except:
            L = 1.0

        # 1. Tự động tính tâm Map từ Anchor
        if self.anchors:
            xs = [pos[0] for pos in self.anchors.values()]
            ys = [pos[1] for pos in self.anchors.values()]
            cx = (min(xs) + max(xs)) / 2.0
            cy = (min(ys) + max(ys)) / 2.0
        else:
            cx, cy = 1.6, 1.6
            
        # 2. Tính tọa độ 4 góc của hình vuông (Tâm là cx, cy)
        half_L = L / 2.0
        BL = (cx - half_L, cy - half_L) # Điểm Dưới-Trái (Bắt đầu)
        BR = (cx + half_L, cy - half_L) # Điểm Dưới-Phải
        TR = (cx + half_L, cy + half_L) # Điểm Trên-Phải
        TL = (cx - half_L, cy + half_L) # Điểm Trên-Trái
        
        # 3. Khai báo 4 đoạn đường (GÓC YAW LUÔN LÀ 0.0 CHO XE MECANUM TỊNH TIẾN)
        segments = [
            (BL, BR, 0.0),    
            (BR, TR, 0.0),   
            (TR, TL, 0.0),  
            (TL, BL, 0.0)   
        ]
        
        # 4. Nội suy sinh các điểm cách nhau (self.interp_step)
        for start_pt, end_pt, yaw in segments:
            x1, y1 = start_pt
            x2, y2 = end_pt
            
            dist = math.hypot(x2 - x1, y2 - y1)
            num_points = int(dist / self.interp_step)
            
            if num_points <= 0:
                continue
                
            for i in range(num_points):
                curr_x = x1 + (x2 - x1) * (i / num_points)
                curr_y = y1 + (y2 - y1) * (i / num_points)
                self.add_point_to_list(round(curr_x, 3), round(curr_y, 3), yaw)

        if len(self.raw_waypoints) > 0:
            first_pt = self.raw_waypoints[0]
            self.add_point_to_list(first_pt[0], first_pt[1], first_pt[2])
            
        QtWidgets.QMessageBox.information(self, "Thành công", f"Đã tạo Hình Vuông (Tâm: {cx},{cy}, Cạnh={L}m)\nXe sẽ giữ góc 0° và tịnh tiến theo quỹ đạo.\nTổng: {len(self.raw_waypoints)} điểm.")

    def is_point_in_map(self, x, y):
        return (self.map_x_min <= x <= self.map_x_max and
                self.map_y_min <= y <= self.map_y_max)

    def on_mode_changed(self, idx):
        if self.set_mode_cb:
            self.set_mode_cb(idx)

    def on_draw_mode_toggled(self, state):
        if state == QtCore.Qt.CheckState.Checked.value:
            self.plot.setMouseEnabled(x=False, y=False)
        else:
            self.plot.setMouseEnabled(x=True, y=True)

    def add_point_to_list(self, x, y, yaw):
        self.raw_waypoints.append((x, y, yaw))
        idx = len(self.raw_waypoints)
        self.waypoint_list_widget.addItem(f"[{idx}] X:{x:.2f} Y:{y:.2f} Yaw:{yaw}°")
        self.waypoint_list_widget.scrollToBottom()
        self.update_path_drawing()

    def on_map_clicked(self, event):
        if not self.draw_mode_cb.isChecked() and event.button() == QtCore.Qt.MouseButton.LeftButton:
            if self.plot.plotItem.vb.sceneBoundingRect().contains(event.scenePos()):
                pos = self.plot.plotItem.vb.mapSceneToView(event.scenePos())
                x, y = round(pos.x(), 3), round(pos.y(), 3)
                if self.is_point_in_map(x, y):
                    self.interp_to_point(x, y, 0.0)

    def on_add_numeric_interp(self):
        try:
            tx = float(self.x_in.text())
            ty = float(self.y_in.text())
            tyaw = float(self.yaw_in.text()) 
            if self.is_point_in_map(tx, ty):
                self.interp_to_point(tx, ty, tyaw)
            else:
                QtWidgets.QMessageBox.warning(self, "Lỗi", "Tọa độ nằm ngoài bản đồ")
        except:
            QtWidgets.QMessageBox.critical(self, "Lỗi", "Số liệu không hợp lệ")

    def interp_to_point(self, tx, ty, tyaw):
        if not self.raw_waypoints:
            start_x = self.last_robot_x
            start_y = self.last_robot_y
            self.add_point_to_list(start_x, start_y, tyaw)
        
        last_x, last_y, _ = self.raw_waypoints[-1]
        dist = math.hypot(tx - last_x, ty - last_y)
        
        if dist < self.interp_step:
            self.add_point_to_list(round(tx, 3), round(ty, 3), tyaw)
            return

        n_steps = int(dist // self.interp_step)
        for i in range(1, n_steps + 1):
            curr_x = last_x + (tx - last_x) * (i * self.interp_step / dist)
            curr_y = last_y + (ty - last_y) * (i * self.interp_step / dist)
            self.add_point_to_list(round(curr_x, 3), round(curr_y, 3), tyaw)
        
        last_added_x = self.raw_waypoints[-1][0]
        last_added_y = self.raw_waypoints[-1][1]
        if math.hypot(tx - last_added_x, ty - last_added_y) > 0.01:
            self.add_point_to_list(round(tx, 3), round(ty, 3), tyaw)

    def update_path_drawing(self):
        if not self.raw_waypoints:
            self.draw_path_line.setData([], [])
            self.draw_path_dots.setData([], [])
            self.path_start_marker.setData([], [])
            self.path_end_marker.setData([], [])
            return
            
        xs = [p[0] for p in self.raw_waypoints]
        ys = [p[1] for p in self.raw_waypoints]
        self.draw_path_line.setData(xs, ys)
        self.draw_path_dots.setData(xs, ys)
        self.path_start_marker.setData([xs[0]], [ys[0]])
        self.path_end_marker.setData([xs[-1]], [ys[-1]])

    def clear_path(self):
        self.raw_waypoints.clear()
        self.waypoint_list_widget.clear()
        self.update_path_drawing()
        self.robot_trace_data.clear() 
        self.robot_trace.setData([], [])
        
        self.time_data.clear()
        self.err_x_data.clear()
        self.err_y_data.clear()
        self.line_err_x.setData([], [])
        self.line_err_y.setData([], [])
        self.start_time = time.time()

    def send_path_to_main(self):
        if len(self.raw_waypoints) < 1: return
        self.path_generated_sig.emit(self.raw_waypoints)

    def on_set_pos_clicked(self):
        try:
            x, y, yaw = float(self.x_start.text()), float(self.y_start.text()), float(self.yaw_start.text())
            self.robot_trace_data.clear() 
            self.set_pos_cb(x, y, yaw)
        except: pass

    def on_emergency_stop(self):
        self.clear_path()         
        self.send_cmd_cb('S')     

    def update_robot_pose(self, x_kf, y_kf, yaw_imu, errX, errY):
        self.last_robot_x = x_kf
        self.last_robot_y = y_kf
        self.lbl_status.setText(f"<b>X:</b> {x_kf:.2f} m | <b>Y:</b> {y_kf:.2f} m\n<b>Yaw:</b> {yaw_imu:.1f}°")
        
        # Vẽ vết robot
        self.robot_trace_data.append((x_kf, y_kf))
        if len(self.robot_trace_data) > 500: self.robot_trace_data.pop(0)
        trace_xs, trace_ys = zip(*self.robot_trace_data)
        self.robot_trace.setData(trace_xs, trace_ys)
        self.robot_current_dot.setData([x_kf], [y_kf])

        # Vẽ đồ thị sai số (Giới hạn mảng lưu trữ để không lag)
        current_t = time.time() - self.start_time
        self.time_data.append(current_t)
        self.err_x_data.append(errX)
        self.err_y_data.append(errY)
        
        if len(self.time_data) > 400:
            self.time_data.pop(0)
            self.err_x_data.pop(0)
            self.err_y_data.pop(0)
            
        self.line_err_x.setData(self.time_data, self.err_x_data)
        self.line_err_y.setData(self.time_data, self.err_y_data)

    def keyPressEvent(self, event):
        key = event.key()
        if key == QtCore.Qt.Key.Key_W: self.send_cmd_cb('F')
        elif key == QtCore.Qt.Key.Key_S: self.send_cmd_cb('B')
        elif key == QtCore.Qt.Key.Key_A: self.send_cmd_cb('L')
        elif key == QtCore.Qt.Key.Key_D: self.send_cmd_cb('R')
        elif key == QtCore.Qt.Key.Key_Space: self.send_cmd_cb('S')
        else: super().keyPressEvent(event)