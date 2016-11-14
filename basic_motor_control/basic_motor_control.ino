int width;
bool led;

void setup() {
  // put your setup code here, to run once:
pinMode(2,OUTPUT);
pinMode(13,OUTPUT);
width=1100;
Serial.begin(115200);
led=false;
}

void loop() {
if(Serial.available()){
  width=Serial.parseInt();
  led=!led;
  digitalWrite(13,led);
}
digitalWrite(2,HIGH);
delayMicroseconds(width);
digitalWrite(2,LOW);
delay(10);

}
