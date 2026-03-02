#include <BluetoothSerial.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ==========================
// Bluetooth
// ==========================
BluetoothSerial SerialBT;

// ==========================
// Motor Pins
// ==========================
#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25
#define ENA 12
#define ENB 33

#define CH_A 0
#define CH_B 1

int speedA = 200;
int speedB = 200;

// ==========================
// WiFi Credentials
// ==========================
const char* ssid = "iPhone";
const char* password = "cha1761@";

// WebSocket Server
WebSocketsServer webSocket(8080);

// ==========================
// Ultrasonic Sensor (UART2)
// ==========================
#define COM 0x55
HardwareSerial sensorSerial(2); // RX=16, TX=17
unsigned long lastSensorRequest = 0;
const unsigned long sensorInterval = 100; // 100ms
int distance_mm = 0;

// ==========================
// WebSocket send timing
// ==========================
unsigned long lastSend = 0;
const unsigned long sendInterval = 100; // 100ms

// ==========================
// WebSocket Event Handler
// ==========================
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) Serial.println("WebSocket: Client connected");
  else if (type == WStype_DISCONNECTED) Serial.println("WebSocket: Client disconnected");
}

// ==========================
// SETUP
// ==========================
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  ledcAttachPin(ENA, CH_A);
  ledcAttachPin(ENB, CH_B);
  ledcSetup(CH_A, 10000, 8);
  ledcSetup(CH_B, 10000, 8);

  // Bluetooth
  SerialBT.begin("ESP32_Motor");
  Serial.println("Bluetooth ready. Pair with ESP32_Motor");

  // WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected! IP:");
  Serial.println(WiFi.localIP());

  // WebSocket
  webSocket.begin();
  webSocket.onEvent(onWsEvent);

  // Ultrasonic Sensor UART
  sensorSerial.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("Ultrasonic sensor ready");
}

// ==========================
// LOOP
// ==========================
void loop() {
  webSocket.loop();      // update WebSocket
  handleBluetooth();     // check BT commands
  readUltrasonic();      // read sensor data
  sendDistanceWS();      // send distance via WebSocket
}

// ===================================================
//  BLUETOOTH CONTROL HANDLER
// ===================================================
void handleBluetooth() {
  while (SerialBT.available()) {
    char c = SerialBT.read();

    // Debug
    Serial.print("BT char: "); Serial.println(c);

    if (c == 'F') forward();
    else if (c == 'B') backward();
    else if (c == 'L') left();
    else if (c == 'R') right();
    else if (c == 'S') stopMotors();

    // Speed command s000-s255
    static String buffer = "";
    if (c == 's') buffer = "s";
    else if (buffer.startsWith("s")) {
      buffer += c;
      if (buffer.length() == 4) {
        int val = buffer.substring(1).toInt();
        speedA = constrain(val, 0, 255);
        speedB = speedA;
        Serial.print("Speed Updated: "); Serial.println(speedA);
        buffer = "";
      }
    }
  }
}

// ===================================================
//  READ ULTRASONIC SENSOR (UART2)
// ===================================================
void readUltrasonic() {
  if (millis() - lastSensorRequest >= sensorInterval) {
    sensorSerial.write(COM);  // send request
    lastSensorRequest = millis();
  }

  while (sensorSerial.available() >= 4) {
    byte header = sensorSerial.read();
    if (header == 0xFF) {
      byte h = sensorSerial.read();
      byte l = sensorSerial.read();
      byte sum = sensorSerial.read();
      if (sum == ((header + h + l) & 0xFF)) {
        distance_mm = (h << 8) + l; // store distance
      } else {
        Serial.println("Checksum error");
      }
    }
  }
}

// ===================================================
//  SEND DISTANCE VIA WEBSOCKET
// ===================================================
void sendDistanceWS() {
  if (millis() - lastSend >= sendInterval) {
    lastSend = millis();

    JsonDocument doc;
    doc["distance_mm"] = distance_mm;

    String json;
    serializeJson(doc, json);
    webSocket.broadcastTXT(json);
    Serial.println(json); // debug
  }
}

// ===================================================
//  MOTOR CONTROL FUNCTIONS
// ===================================================
void forward() {
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);
  ledcWrite(CH_A,speedA); ledcWrite(CH_B,speedB);
}

void backward() {
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  ledcWrite(CH_A,speedA); ledcWrite(CH_B,speedB);
}

void left() {
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  ledcWrite(CH_A,speedA); ledcWrite(CH_B,speedB);
}

void right() {
  digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
  ledcWrite(CH_A,speedA); ledcWrite(CH_B,speedB);
}

void stopMotors() {
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
  ledcWrite(CH_A,0); ledcWrite(CH_B,0);
}

