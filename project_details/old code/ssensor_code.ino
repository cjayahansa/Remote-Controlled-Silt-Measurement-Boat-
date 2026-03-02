#include <HardwareSerial.h>

// Use UART2 on ESP32
HardwareSerial sensorSerial(2); // UART2

void setup() {
  Serial.begin(115200);         // USB Serial Monitor
  sensorSerial.begin(115200, SERIAL_8N1, 16, 17); 
  // rxPin=16, txPin=17 (UART2)
  Serial.println("Underwater ultrasonic UART sensor ready");
}

void loop() {
  // We want 4 bytes: header + H_DATA + L_DATA + checksum
  if (sensorSerial.available() >= 4) {
    byte header = sensorSerial.read();

    // Only process if header is 0xFF
    if (header == 0xFF) {
      byte h = sensorSerial.read();  // high byte
      byte l = sensorSerial.read();  // low byte
      byte sum = sensorSerial.read(); // checksum

      // Checksum = (header + h + l) & 0xFF
      byte calc = (header + h + l) & 0xFF;

      if (sum == calc) {
        int dist = (h << 8) + l;  // final distance
        Serial.print("Distance: ");
        Serial.print(dist);
        Serial.println(" cm");
      } else {
        Serial.println("Checksum error");
      }
    }
  }
}

