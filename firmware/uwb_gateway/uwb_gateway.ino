// Gateway_ESP32_UART_to_UDP_Broadcast_Step.ino

#include <WiFi.h>
#include <WiFiUdp.h>
#include "wifi_config.h"

// --- Wi-Fi Configuration ---
const int udp_port_pc = 4121;                // Cổng UDP để gửi đến PC

WiFiUDP udp_to_pc;

// --- UART Pins cho giao tiếp với Tag (Serial2) ---
const int GATEWAY_RX_FROM_TAG_PIN = 16;  // Nối với TX2 của Tag (ví dụ GPIO17 trên Tag)
const int GATEWAY_TX_TO_TAG_PIN = 17;    // Nối với RX2 của Tag (ví dụ GPIO16 trên Tag, nếu cần)
const long UART_TAG_BAUD_RATE = 9600;    // <<< GIỮ BAUD RATE 9600 (PHẢI KHỚP VỚI TAG)

String received_data_buffer = "";  // Buffer để lưu dữ liệu nhận được từ Tag

void setup() {
  Serial.begin(115200);  // Serial USB để debug Gateway
  delay(500);
  Serial.println("--- ESP32 Gateway (UART from Tag, UDP Broadcast to PC) ---");

  // Khởi tạo Serial2 để nhận dữ liệu từ Tag
  Serial2.begin(UART_TAG_BAUD_RATE, SERIAL_8N1, GATEWAY_RX_FROM_TAG_PIN, GATEWAY_TX_TO_TAG_PIN);
  Serial.print("Serial2 for Tag initialized at ");
  Serial.print(UART_TAG_BAUD_RATE);
  Serial.println(" baud. Waiting for data from Tag...");

  // Kết nối Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi '");
  Serial.print(WIFI_SSID);
  Serial.print("' ...");
  unsigned long wifi_connect_timeout = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifi_connect_timeout < 15000) {  // Timeout 15 giây
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("Gateway IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Will broadcast UDP to port: ");
    Serial.println(udp_port_pc);
  } else {
    Serial.println("\nWiFi Connection Failed. Data from Tag will still be printed to Serial Monitor (if USB connected).");
  }
  Serial.println("--------------------------------------------------");
}

void loop() {
  if (Serial2.available()) {
    char receivedChar = Serial2.read();
    received_data_buffer += receivedChar;

    if (receivedChar == '\n') {     // Đã nhận đủ một dòng (chuỗi JSON)
      received_data_buffer.trim();  // Xóa ký tự \r hoặc khoảng trắng thừa

      if (received_data_buffer.length() > 0) {
        // Luôn in ra Serial USB của Gateway để debug
        Serial.print("Gateway Received (S2): [");
        Serial.print(received_data_buffer);
        Serial.println("]");

        // Gửi qua UDP NẾU Wi-Fi đã kết nối
        if (WiFi.status() == WL_CONNECTED) {
          udp_to_pc.beginPacket("255.255.255.255", udp_port_pc);  // Gửi broadcast
          udp_to_pc.print(received_data_buffer);
          if (udp_to_pc.endPacket()) {
            Serial.println("  -> UDP broadcast packet sent.");
            } else {
            Serial.println("  -> [Warning] UDP broadcast packet send FAILED.");
          }
        } else {
          Serial.println("  -> WiFi not connected, UDP not sent.");
        }
      }
      received_data_buffer = "";  // Reset buffer để nhận chuỗi tiếp theo
    }
  }
}