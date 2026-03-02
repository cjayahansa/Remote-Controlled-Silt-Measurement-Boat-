#define COM 0x55

unsigned char buffer_RTT[4] = {0};
uint8_t CS;
int Distance = 0;

// Use hardware Serial2 on ESP32
// RX2 = GPIO16, TX2 = GPIO17 (you can change pins as needed)
HardwareSerial mySerial(2);

void setup() {
  Serial.begin(115200);       // For debug output
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // RX, TX pins
}

void loop() {
  mySerial.write(COM);       // Send command to sensor
  delay(100);

  if (mySerial.available() > 0) {
    delay(4); // small delay to ensure all bytes arrive

    if (mySerial.read() == 0xFF) {    
      buffer_RTT[0] = 0xFF;
      for (int i = 1; i < 4; i++) {
        while(!mySerial.available()); // wait for the byte
        buffer_RTT[i] = mySerial.read();
      }

      // Calculate checksum
      CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];

      if (buffer_RTT[3] == CS) {
        Distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
        Serial.print("Distance: ");
        Serial.print(Distance);
        Serial.println(" mm");
      }
    }
  }
}

