void setup() {
  // put your setup code here, to run once:
pinMode(13,OUTPUT);
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
if(Serial.available()){
  digitalWrite(13,Serial.read()%2);
}
delay(100);
Serial.println("hello!");
}
