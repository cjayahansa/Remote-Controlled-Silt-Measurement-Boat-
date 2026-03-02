#define TRIG 5
#define ECHO 18

void setup() {
  Serial.begin(9600);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);  // IMPORTANT: no pull-up
}

void loop() {

  // Ensure clean LOW trigger
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  // JSN-SR04T needs 30–50 µs trigger
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIG, LOW);

  // Longer timeout for JSN-SR04T (40ms)
  long duration = pulseIn(ECHO, HIGH, 26000);

  int distance = duration / 58;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(80);
}
