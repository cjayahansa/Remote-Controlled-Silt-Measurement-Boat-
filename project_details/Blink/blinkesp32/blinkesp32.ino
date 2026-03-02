void setup() {
  pinMode(2, OUTPUT);  // GPIO 2 has the onboard LED on most ESP32 boards
}

void loop() {
  digitalWrite(2, HIGH);  
  delay(1000);            // wait 1 second
  digitalWrite(2, LOW);   
  delay(1000);            // wait 1 second
}
