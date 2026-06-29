# Sao chép tệp này thành config.py trước khi chạy chương trình.

ESP32_IP = "192.168.1.100"  # IP của ESP32 điều khiển Mecanum

UDP_PORT_CMD = 4210
UDP_PORT_ODOM = 4210
UDP_PORT_UWB = 4121

WHEEL_DIAMETER = 0.097
SCALE_FACTOR = 1.0
D_KINEMATICS = 0.25  # lx + ly, đơn vị mét

# Đo lại tọa độ anchor tại nơi triển khai. ID phải khớp dữ liệu UWB.
ANCHORS = {
    "1782": (0.0, 0.0),
    "1783": (5.0, 0.0),
    "1784": (0.0, 5.0),
}
