#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Motor Pins
#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25
#define ENA 12
#define ENB 33

#define CH_A 0
#define CH_B 1

int speedA = 200; // Default speed
int speedB = 200;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  ledcAttachPin(ENA, CH_A);
  ledcAttachPin(ENB, CH_B);

  ledcSetup(CH_A, 10000, 8); // 10kHz, 8-bit resolution
  ledcSetup(CH_B, 10000, 8);

  Serial.begin(115200);
  SerialBT.begin("ESP32_Motor"); // Bluetooth device name
  Serial.println("Bluetooth started! Pair with 'ESP32_Motor'");
}

void loop() {
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim(); // Remove newline or spaces

    if (cmd == "F") {
      forward();
    } else if (cmd == "B") {
      backward();
    } else if (cmd == "L") {
      left();
    } else if (cmd == "R") {
      right();
    } else if (cmd == "S") {
      stopMotors();
    } else if (cmd.startsWith("speed:")) {
      int val = cmd.substring(6).toInt();
      speedA = constrain(val, 0, 255);
      speedB = constrain(val, 0, 255);
      SerialBT.printf("Speed set to %d\n", speedA);
    }
  }
}

// Motor functions
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
