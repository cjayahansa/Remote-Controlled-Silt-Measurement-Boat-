// ===================================================
//  BLUETOOTH + WIFI + MOTOR + RS485 SENSOR
// ===================================================

#include <BluetoothSerial.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>

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
const char* ssid = "Chamika";
const char* password = "12345678";

// WebSocket Server
WebSocketsServer webSocket(8080);

// ==========================
// RS485 Ultrasonic Sensor
// ==========================
#define DE_RE 4      // GPIO to control RS485 direction
HardwareSerial RS485Serial(2); // UART2
ModbusMaster sensorNode;

void preTransmission() { digitalWrite(DE_RE, HIGH); } // TX
void postTransmission() { digitalWrite(DE_RE, LOW); }  // RX

// ==========================
// Setup
// ==========================
void setup() {
  Serial.begin(115200);

  // ---------- MOTOR SETUP ----------
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttachPin(ENA, CH_A);
  ledcAttachPin(ENB, CH_B);
  ledcSetup(CH_A, 10000, 8);
  ledcSetup(CH_B, 10000, 8);

  // ---------- BLUETOOTH ----------
  SerialBT.begin("ESP32_Motor");
  Serial.println("Bluetooth ready. Pair with ESP32_Motor");

  // ---------- WIFI ----------
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // ---------- WEBSOCKET ----------
  webSocket.begin();
  webSocket.onEvent(onWsEvent);

  // ---------- RS485 SENSOR ----------
  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW); // start in receive mode
  RS485Serial.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17
  sensorNode.begin(0x01, RS485Serial); // default sensor address
  sensorNode.preTransmission(preTransmission);
  sensorNode.postTransmission(postTransmission);

  Serial.println("RS485 Sensor Ready");
}

// ==========================
// WebSocket Event
// ==========================
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) Serial.println("WebSocket: Client connected");
  else if (type == WStype_DISCONNECTED) Serial.println("WebSocket: Client disconnected");
}

// ==========================
// Main Loop
// ==========================
unsigned long lastSend = 0;

void loop() {
  webSocket.loop();       // WebSocket service
  handleBluetooth();      // BT commands
  sendDistance();         // Read sensor + send JSON every 200ms
}

// ==========================
// Bluetooth Control Handler
// ==========================
void handleBluetooth() {
  static String buffer = "";
  while (SerialBT.available()) {
    char c = SerialBT.read();
    Serial.print("BT char: "); Serial.println(c);

    if (c == 'F') forward();
    else if (c == 'B') backward();
    else if (c == 'L') left();
    else if (c == 'R') right();
    else if (c == 'S') stopMotors();

    // SPEED COMMAND: s180
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

// ==========================
// Read Sensor & Send JSON
// ==========================
void sendDistance() {
  if (millis() - lastSend > 200) {  // update every 200ms
    lastSend = millis();

    uint16_t distance_mm = 0;
    uint8_t result = sensorNode.readHoldingRegisters(0x0101, 1); // real-time distance
    if (result == sensorNode.ku8MBSuccess) {
      distance_mm = sensorNode.getResponseBuffer(0); // value in mm
    }

    // Convert mm to cm as integer
    uint16_t distance_cm = distance_mm / 10; // decimal truncated

    // Build JSON
    StaticJsonDocument<200> doc;
    doc["d1"] = distance_cm;

    String json;
    serializeJson(doc, json);

    webSocket.broadcastTXT(json);
    Serial.println(json); // debug
  }
}



// ==========================
// Motor Functions
// ==========================
void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  ledcWrite(CH_A, speedA); ledcWrite(CH_B, speedB);
}

void backward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(CH_A, speedA); ledcWrite(CH_B, speedB);
}

void left() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(CH_A, speedA); ledcWrite(CH_B, speedB);
}

void right() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  ledcWrite(CH_A, speedA); ledcWrite(CH_B, speedB);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(CH_A, 0); ledcWrite(CH_B, 0);
}
