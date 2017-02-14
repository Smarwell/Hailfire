int width;
bool led;

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);
  width = 1000;
  Serial.begin(115200);
  Serial.setTimeout(10);
  led = false;
}

void loop() {
  if (Serial.available()) {
    width = Serial.parseInt();
    led = !led;
    digitalWrite(13, led);
  }
  digitalWrite(2, HIGH);
  delayMicroseconds(width);
  digitalWrite(2, LOW);
  delay(10);

}
