// Defining what each pin's task is

#define SpeedLeft 5
#define SpeedRight 6
#define DirectionLeft 7
#define DirectionRight 8
#define PowerAll 3

void setup() {
 
  // Setting the pins as outputs

pinMode(SpeedLeft, OUTPUT);
pinMode(SpeedRight, OUTPUT);
pinMode(DirectionLeft, OUTPUT);
pinMode(DirectionRight, OUTPUT);
pinMode(PowerAll, OUTPUT);
Serial.begin(9600);

}

void forward(int speed) {
  digitalWrite(DirectionRight, HIGH);
  analogWrite(SpeedRight, speed);
  digitalWrite(DirectionLeft, HIGH);
  analogWrite(SpeedLeft, speed);
  digitalWrite(PowerAll, HIGH);
  Serial.println("Forward");

}

void left(int speed) {
  digitalWrite(DirectionRight, HIGH);
  analogWrite(SpeedRight, speed);
  digitalWrite(DirectionLeft, LOW);
  analogWrite(SpeedLeft, speed);
  digitalWrite(PowerAll, HIGH);
  Serial.println("Left");

}

void right(int speed) {
  digitalWrite(DirectionRight, LOW);
  analogWrite(SpeedRight, speed);
  digitalWrite(DirectionLeft, HIGH);
  analogWrite(SpeedLeft, speed);
  digitalWrite(PowerAll, HIGH);
  Serial.println("Right");

}

void back(int speed) {
  digitalWrite(DirectionRight, LOW);
  analogWrite(SpeedRight, speed);
  digitalWrite(DirectionLeft, LOW);
  analogWrite(SpeedLeft, speed);
  digitalWrite(PowerAll, HIGH);
  Serial.println("Forward");

}

void loop() {
  // put your main code here, to run repeatedly:

forward(255);
delay(2000);
back(255);
delay(2000);
forward(255);
delay(1000);
right(255);
delay(500);
back(255);
delay(4000);
left(255);
delay(3000);


}
