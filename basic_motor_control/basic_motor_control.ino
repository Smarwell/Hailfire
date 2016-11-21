int width;
bool led;

void setup() {
  // put your setup code here, to run once:
pinMode(2,OUTPUT);
pinMode(22,OUTPUT);
width=1000;
Serial.begin(115200);
led=false;
}

void loop() {
if(Serial.available()){
  width=Serial.parseInt();
  led=!led;
  digitalWrite(2,led);
}
digitalWrite(22,HIGH);
delayMicroseconds(width);
digitalWrite(22,LOW);
delay(10);

}
