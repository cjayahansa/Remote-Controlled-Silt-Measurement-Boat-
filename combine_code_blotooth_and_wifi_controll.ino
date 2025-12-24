// ===================================================
//  BLUETOOTH + WIFI + MOTOR CONTROL COMBINED VERSION
// ===================================================

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
// WebSocket Event Handler
// ==========================
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.println("WebSocket: Client connected");
  } 
  else if (type == WStype_DISCONNECTED) {
    Serial.println("WebSocket: Client disconnected");
  }
}

// ==========================
// SETUP
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

  // ---------- BLUETOOTH START ----------
  SerialBT.begin("ESP32_Motor");
  Serial.println("Bluetooth ready. Pair with ESP32_Motor");

  // ---------- WIFI START ----------
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

  // ---------- WEBSOCKET START ----------
  webSocket.begin();
  webSocket.onEvent(onWsEvent);
}

// ==========================
// LOOP
// ==========================
unsigned long lastSend = 0;

void loop() {
  webSocket.loop();  // update websocket service
  handleBluetooth(); // check for BT commands
  sendFakeDistances(); // send JSON every 1 sec
}

// ===================================================
//  BLUETOOTH CONTROL HANDLER
// ===================================================
void handleBluetooth() {
  // Read ALL incoming BT characters without blocking
  while (SerialBT.available()) {
    
    char c = SerialBT.read();  // get one byte

    Serial.print("BT char: ");
    Serial.println(c); // debug

    // -----------------------
    // SINGLE-CHAR COMMANDS
    // -----------------------
    if (c == 'F') {
      forward();
    }
    else if (c == 'B') {
      backward();
    }
    else if (c == 'L') {
      left();
    }
    else if (c == 'R') {
      right();
    }
    else if (c == 'S') {
      stopMotors();
    }

    // -----------------------
    // SPEED COMMAND: s180
    // -----------------------
    static String buffer = "";
    if (c == 's') {
      buffer = "s";
    } 
    else if (buffer.startsWith("s")) {
      buffer += c;
      if (buffer.length() == 4) {  // s + 3 digits
        int val = buffer.substring(1).toInt();
        speedA = constrain(val, 0, 255);
        speedB = speedA;

        Serial.print("Speed Updated: ");
        Serial.println(speedA);

        buffer = "";  // clear
      }
    }

  } // end while
}


// ===================================================
//  SEND DISTANCE VIA WEBSOCKET
// ===================================================
void sendFakeDistances() {
  if (millis() - lastSend > 1000) {
    lastSend = millis();

    float d1 = random(30, 120); 

    JsonDocument doc;
    doc["d1"] = d1;

    String json;
    serializeJson(doc, json);

    webSocket.broadcastTXT(json);
    Serial.println(json);
  }
}

// ===================================================
//  MOTOR CONTROL FUNCTIONS
// ===================================================

void forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(CH_A, speedA);
  ledcWrite(CH_B, speedB);
}

void backward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(CH_A, speedA);
  ledcWrite(CH_B, speedB);
}

void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(CH_A, speedA);
  ledcWrite(CH_B, speedB);
}

void right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(CH_A, speedA);
  ledcWrite(CH_B, speedB);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(CH_A, 0);
  ledcWrite(CH_B, 0);
}
