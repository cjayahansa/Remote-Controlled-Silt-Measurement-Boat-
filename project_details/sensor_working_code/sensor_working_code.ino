#include <ModbusMaster.h>
#include <HardwareSerial.h>

#define DE_RE 4      // RS485 direction control

HardwareSerial RS485Serial(2);
ModbusMaster node;

void preTransmission() { digitalWrite(DE_RE, HIGH); } // TX
void postTransmission() { digitalWrite(DE_RE, LOW); }  // RX

void setup() {
  Serial.begin(115200);
  RS485Serial.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17
  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW);

  node.begin(0x01, RS485Serial); // Default slave address 0x01
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial.println("RS485 Modbus DYP-L042M4W Ready");
}

void loop() {
  uint8_t result;

  // --- Read real-time distance (register 0x0101) ---
  result = node.readHoldingRegisters(0x0101, 1);
  if (result == node.ku8MBSuccess) {
    uint16_t distance = node.getResponseBuffer(0);
    Serial.print("Real-time Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  } else {
    Serial.println("Error reading distance");
  }

  // --- Read temperature (register 0x0102) ---
  result = node.readHoldingRegisters(0x0102, 1);
  if (result == node.ku8MBSuccess) {
    int16_t tempRaw = node.getResponseBuffer(0);
    float temperature = tempRaw / 10.0; // 0.1°C units
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
  } else {
    Serial.println("Error reading temperature");
  }

  delay(200); // Read every 200ms
}
