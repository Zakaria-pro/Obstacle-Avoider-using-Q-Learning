// Pins
const int trig = A2;
const int echo = A1;

void setup(){
  pinMode(trig, OUTPUT);
  digitalWrite(echo, LOW);
  Serial.begin(9600);
}

void loop(){
  distance();
  delay(2000);
}

void distance(){
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  int duree = pulseIn(echo,HIGH);
  float dist = 0.034*duree/2;
  Serial.print("La distance est :");
  Serial.print(dist);
  Serial.print("\n");
}
