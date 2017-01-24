int width;
bool led;

void setup() {
  // put your setup code here, to run once:
pinMode(2,OUTPUT);
pinMode(3,OUTPUT);
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
digitalWrite(3,HIGH);
delayMicroseconds(width);
digitalWrite(3,LOW);
delay(10);

}
