
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include "utility/Adafruit_PWMServoDriver.h"\

#define rightLine A11
#define midRightLine A12
#define midLine A14
#define midLeftLine A15
#define leftLine A13

#define leftLightSensor 6
#define rightLightSensor 7

//Color Sensor
#define colourSensor 27 //left
#define S0 23
#define S1 22
#define S2 25
#define S3 24

#define colourSensor0 49 //right
#define S00 52
#define S10 53
#define S20 51
#define S30 50

//Ultrasonic Sensor
#include "SR04.h"
#define TRIG_PIN A2
#define ECHO_PIN A1
long distance;
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *BackRight = AFMS.getMotor(3);
Adafruit_DCMotor *FrontRight = AFMS.getMotor(4);
Adafruit_DCMotor *BackLeft = AFMS.getMotor(1);
Adafruit_DCMotor *FrontLeft = AFMS.getMotor(2);

int frequencyRight, frequencyLeft;
int counterLeft = 0;
int counterRight = 0;
const byte leftGreen = 15;
const byte rightGreen = 15;
const byte leftBlack = 30;
const byte rightBlack = 60;
boolean isJunction = false;
String lastMove = "";

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

void setup() {
  AFMS.begin();
  BackRight ->setSpeed(0);
  BackRight ->run(RELEASE);
  FrontRight -> setSpeed(0);
  FrontRight ->run(RELEASE);
  BackLeft ->setSpeed(0);
  BackLeft ->run(RELEASE);
  FrontLeft -> setSpeed(0);
  FrontLeft ->run(RELEASE);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(colourSensor, INPUT);

  pinMode(S00, OUTPUT);
  pinMode(S10, OUTPUT);
  pinMode(S20, OUTPUT);
  pinMode(S30, OUTPUT);
  pinMode(colourSensor0, INPUT);

  pinMode(S00, OUTPUT);
  pinMode(S10, OUTPUT);
  pinMode(S20, OUTPUT);
  pinMode(S30, OUTPUT);
  pinMode(colourSensor0, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  digitalWrite(S00, HIGH);
  digitalWrite(S10, LOW);
  digitalWrite(S20, HIGH);
  digitalWrite(S30, LOW);
  Serial.begin(9600);

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(RCTime(leftLightSensor));
  Serial.print("    ");
  Serial.print(analogRead(midLeftLine));
  Serial.print("    ");
  Serial.print(analogRead(midLine));
  Serial.print("    ");
  Serial.print(analogRead(midRightLine));
  Serial.print("    ");
  Serial.println(RCTime(rightLightSensor));
}
