# Hướng dẫn triển khai chi tiết

Tài liệu này bổ sung cho phần cài đặt nhanh trong README.

## 1. Kiểm tra cơ khí trước khi cấp nguồn

- Nâng bánh khỏi mặt sàn trong lần chạy đầu tiên.
- Xác nhận thứ tự bánh trong firmware trùng với lắp đặt thực tế.
- Quay tay từng bánh và kiểm tra encoder tăng/giảm đúng chiều.
- Kiểm tra driver động cơ có giới hạn dòng phù hợp.
- Chuẩn bị công tắc ngắt nguồn động cơ độc lập với Wi-Fi.

## 2. Bố trí UWB

Đặt ba anchor tại vị trí cố định, không thẳng hàng và có đường nhìn tốt đến vùng robot hoạt động. Đo tọa độ theo cùng một hệ trục, nhập đúng các giá trị này vào `software/config.py`.

Antenna delay trong các sketch anchor là giá trị đã dùng ở mô hình gốc. Với module khác, cần hiệu chuẩn lại thay vì sao chép nguyên giá trị.

## 3. Kết nối Tag và Gateway

- TX của Tag nối vào RX của Gateway.
- RX chỉ cần nối khi có giao tiếp hai chiều.
- Hai board phải chung GND.
- Baud rate ở Tag và Gateway phải giống nhau.

## 4. Kiểm tra từng tầng

### UWB

Serial của Tag phải có đủ ba ID và khoảng cách thay đổi hợp lý khi di chuyển.

### Gateway

Gateway phải nhận được mỗi dòng JSON hoàn chỉnh và phát UDP qua cổng 4121.

### Encoder và IMU

ESP32 điều khiển phải gửi chuỗi gồm bốn RPM và một góc yaw qua cổng 4210. Khi robot đứng yên, RPM nên gần 0 và yaw không thay đổi nhanh.

### Điều khiển

Chạy lệnh tiến/lùi ở tốc độ thấp khi bánh đang được nâng. Nếu một bánh quay sai chiều, sửa ánh xạ hoặc dấu của bánh đó trước khi chạy tự hành.

## 5. Hiệu chuẩn cơ bản

- Đo lại đường kính lăn thực tế của bánh, không chỉ dùng kích thước danh nghĩa.
- Cho robot chạy thẳng một quãng đã biết để chỉnh `SCALE_FACTOR`.
- Đo khoảng cách tâm robot theo hai phương để chỉnh `D_KINEMATICS`.
- Hiệu chuẩn antenna delay cho từng UWB module.
- Chỉnh PID tốc độ từng bánh trước, sau đó mới chỉnh PID vị trí trên máy tính.

## 6. Thứ tự chạy an toàn

1. Bật Anchor.
2. Bật Tag và Gateway.
3. Bật ESP32 điều khiển, giữ robot đứng yên trong thời gian hiệu chuẩn gyro.
4. Chạy `python main.py`.
5. Kiểm tra vị trí trên GUI trước khi cấp nguồn công suất cho động cơ.
6. Thử lệnh dừng và watchdog.
7. Bắt đầu bằng quãng đường ngắn, tốc độ thấp.
