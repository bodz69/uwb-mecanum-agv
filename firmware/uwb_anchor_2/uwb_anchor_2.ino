#include <SPI.h>
#include "DW1000Ranging.h"

#define ANCHOR_ADD "83:17:5B:D5:A9:9A:E2:9C"  // Anchor2 short address
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS  = 4;

// --- CALLBACKS ---
void newRange() {
  DW1000Device* dev = DW1000Ranging.getDistantDevice();
  Serial.print("from: ");
  Serial.print(dev->getShortAddress(), HEX);
  Serial.print("\t Range: ");
  Serial.print(dev->getRange());
  Serial.print(" m\t RX power: ");
  Serial.print(dev->getRXPower());
  Serial.println(" dBm");
}

void newBlink(DW1000Device* device) {
  Serial.print("blink; device added -> short: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
// --------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1) SPI + DW1000 init
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);

  // 2) Antenna delay đã calibrate
  DW1000.setAntennaDelay(16590);//16590

  // 3) Đăng ký callback
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachBlinkDevice(newBlink);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // 4) Start as anchor
  DW1000Ranging.startAsAnchor(
    ANCHOR_ADD,
    DW1000.MODE_LONGDATA_RANGE_LOWPOWER,
    false
  );
}

void loop() {
  DW1000Ranging.loop();
  delay(1);
}