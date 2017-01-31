int width;
bool led;

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  pinMode(25, OUTPUT);
  width = 1000;
  Serial1.begin(9600);
  Serial1.setTimeout(10);
  led = false;
}

void loop() {
  if (Serial1.available()) {
    width = int(Serial1.parseInt()/255.0*1000+1000);
    led = !led;
    digitalWrite(13, led);
  }
  digitalWrite(25, HIGH);
  delayMicroseconds(width);
  digitalWrite(25, LOW);
  delay(10);

}
