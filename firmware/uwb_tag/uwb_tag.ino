#include <SPI.h>
#include <DW1000Ranging.h>
#include "link.h"

// --- Pinout ---
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
const uint8_t PIN_RST = 27, PIN_IRQ = 34, PIN_SS = 4;

#define TAG_TX_TO_GATEWAY_PIN 17
#define TAG_RX_FROM_GATEWAY_PIN 16
#define UART_GATEWAY_BAUD_RATE 9600

#define MAX_ANCHORS 10
#define MAX_SAMPLES 7
#define SEND_INTERVAL 100UL 
#define MAX_VALID_RANGE 100.0f

struct RangeBuffer {
  uint16_t addr;
  float samples[MAX_SAMPLES];
  int count;
  float last_rx_dbm;
};

MyLink* uwb_data;
RangeBuffer anchorBuffers[MAX_ANCHORS];
int anchorCount = 0;
portMUX_TYPE bufferMutex = portMUX_INITIALIZER_UNLOCKED;
String json_to_gateway = "";

// === UTIL ===
int findAnchorIndex(uint16_t addr) {
  for (int i = 0; i < anchorCount; i++) {
    if (anchorBuffers[i].addr == addr) return i;
  }
  if (anchorCount < MAX_ANCHORS) {
    anchorBuffers[anchorCount].addr = addr;
    anchorBuffers[anchorCount].count = 0;
    return anchorCount++;
  }
  return -1;
}

//TRẢ GIÁ TRỊ TRUNG VỊ
float medianN(float* data, int count) {
  float sorted[MAX_SAMPLES];
  memcpy(sorted, data, count * sizeof(float));

  // sort
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (sorted[i] > sorted[j]) {
        float temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }

  return sorted[count / 2];
}


void newRange() {
  auto dev = DW1000Ranging.getDistantDevice();
  uint16_t addr = dev->getShortAddress();
  float raw = dev->getRange();
  float rx_dbm = dev->getRXPower();

  if (!isfinite(raw) || raw < 0.0f || raw > MAX_VALID_RANGE) return;

  int idx = findAnchorIndex(addr);
  if (idx == -1) return;

  portENTER_CRITICAL(&bufferMutex);
  if (anchorBuffers[idx].count < MAX_SAMPLES) {
    anchorBuffers[idx].samples[anchorBuffers[idx].count++] = raw;
    anchorBuffers[idx].last_rx_dbm = rx_dbm;
  }
  portEXIT_CRITICAL(&bufferMutex);
}

void newDevice(DW1000Device *device) {
  add_link(uwb_data, device->getShortAddress());
}

void inactiveDevice(DW1000Device *device) {
  delete_link(uwb_data, device->getShortAddress());
}


void dataProcessingTask(void *parameter) {
  for (;;) {
    delay(SEND_INTERVAL);

    portENTER_CRITICAL(&bufferMutex);

    for (int i = 0; i < anchorCount; i++) {
      if (anchorBuffers[i].count == 0) continue;

      float medianValue = medianN(anchorBuffers[i].samples, anchorBuffers[i].count);

      float finalRange = medianValue + 0.05f; // offset nếu có
      fresh_link(uwb_data, anchorBuffers[i].addr, finalRange, anchorBuffers[i].last_rx_dbm);
      anchorBuffers[i].count = 0;
    }

    portEXIT_CRITICAL(&bufferMutex);

    make_link_json(uwb_data, &json_to_gateway);
    Serial2.println(json_to_gateway);
    Serial.println(json_to_gateway); 
  }
}


void setup() {
  Serial.begin(115200);
  delay(500);

  Serial2.begin(UART_GATEWAY_BAUD_RATE, SERIAL_8N1, TAG_RX_FROM_GATEWAY_PIN, TAG_TX_TO_GATEWAY_PIN);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_LOWPOWER);

  uwb_data = init_link();

  xTaskCreatePinnedToCore(
    dataProcessingTask,
    "DataProcessor",
    4096,
    NULL,
    1,
    NULL,
    0  
  );
}

void loop() {
  DW1000Ranging.loop(); 
}
