// Simple Blink for STM32F103C8 (Blue Pill)
// LED is connected to PC13 (active LOW)

void setup() {
  pinMode(PA0, OUTPUT);
  pinMode(PA1, OUTPUT);
  pinMode(PA2, OUTPUT);
  pinMode(PA3, OUTPUT);
  pinMode(PA4, OUTPUT);
  pinMode(PA5, OUTPUT);



  pinMode(PA8, OUTPUT);
  pinMode(PA9, OUTPUT);
  pinMode(PA10, OUTPUT);
  pinMode(PA11, OUTPUT);
  pinMode(PA12, OUTPUT);
  pinMode(PA15, OUTPUT);
}

void loop() {
  digitalWrite(PA1, LOW);   
  digitalWrite(PA2, HIGH); 

  digitalWrite(PA3, LOW);   
  digitalWrite(PA4, HIGH);   

  digitalWrite(PA9, LOW);   
  digitalWrite(PA10, HIGH); 

  digitalWrite(PA11, LOW);   
  digitalWrite(PA12, HIGH); 

  analogWrite(PA0, 200);
  analogWrite(PA5, 200);
  analogWrite(PA8, 200);
  analogWrite(PA15, 200);
  
}
