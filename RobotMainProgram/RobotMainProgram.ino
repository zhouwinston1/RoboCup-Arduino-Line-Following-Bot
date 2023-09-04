
/*
  Program that allows a robot to line follow. It looks for objects, has a gyroscope to align the robot to make precise turns, and is able to find what kind of intersection its in
  RoboCup Line Follow Assignment
  Winston Zhou, Mats Leis
  Last Updated 4/14/2022
*/

//Libraries for the motorshield, gyrosensor communication, and servo communication
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Servo.h>

//Line follow readings
#define rightLine A11
#define midRightLine A12
#define midLine A14
#define midLeftLine A15
#define leftLine A13
#define button 12

//Light sensor readings
#define leftLightSensor 6
#define rightLightSensor 7

//Gyro library
#include <MPU6050_tockn.h>
MPU6050 mpu6050(Wire);
int angle = 10;

//Servo setup
Servo arm;
Servo myarm;

//Left color sensor pins
const int S0 = 23;
const int S1 = 22;
const int S2 = 25;
const int S3 = 24;
const int colourSensor = 27;

//Right color sensor pins
const int S00 = 52;
const int S10 = 53;
const int S20 = 51;
const int S30 = 50;
const int colourSensor0 = 49;

//Ultrasonic Sensor
#include "SR04.h"
#define TRIG_PIN A0
#define ECHO_PIN A1
long distance;
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);

//Setting up motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *BackRight = AFMS.getMotor(3);
Adafruit_DCMotor *FrontRight = AFMS.getMotor(4);
Adafruit_DCMotor *BackLeft = AFMS.getMotor(1);
Adafruit_DCMotor *FrontLeft = AFMS.getMotor(2);

//Variables
int frequencyRight, frequencyLeft;
int counterLeft = 0;
int counterRight = 0;
boolean isJunction = false;
String lastMove = "left";
String turnType;

//Gyro readings
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float angleX, angleY, angleZ, startAngleZ;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

//Green readings
int red = 0;
int green = 0;
int blue = 0;
int red0 = 0;
int green0 = 0;
int blue0 = 0;

int newGreen = 4;

int leftGreen = 300;
int rightGreen = 500;
const int trigPin = 13;
const int echoPin = 12;
int MPU_addr = 0x68;

int cal_gyro = 1;  //set to zero to use gyro calibration offsets below.

// vvvvvvvvvvvvvvvvvv  VERY VERY IMPORTANT vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//These are the previously determined offsets and scale factors for accelerometer and gyro for
// a particular example of an MPU-6050. They are not correct for other examples.
//The AHRS will NOT work well or at all if these are not correct

float A_cal[6] = {0.0, 0.0, 0.0, 1.000, 1.000, 1.000}; // 0..2 offset xyz, 3..5 scale xyz
float G_off[3] = { 0., 0., 0.}; //raw offsets, determined for gyro at rest
#define gscale ((250./32768.0)*(PI/180.0))  //gyro default 250 LSB per d/s -> rad/s

// ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

// GLOBALLY DECLARED, required for Mahony filter
// vector to hold quaternion
float q[4] = {1.0, 0.0, 0.0, 0.0};

// Free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
float Kp = 30.0;
float Ki = 0.0;

// Notes: with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
// with MPU-6050, some instability observed at Kp=100, Kp=30 works well

// char s[60]; //snprintf buffer, if needed for debug

// globals for AHRS loop timing
unsigned long now_ms, last_ms = 0; //millis() timers

// print interval
unsigned long print_ms = 200; //print angles every "print_ms" milliseconds
float yaw, pitch, roll; //Euler angle output
void setup() {
  AFMS.begin();
  mpu6050.begin();
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

  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);
  digitalWrite(S00, LOW);
  digitalWrite(S10, HIGH);
  pinMode(colourSensor0, INPUT);
  pinMode(button, INPUT_PULLUP);

  Serial.begin(9600);
  Serial1.begin(9600);

  Wire.begin(); // Initialize comunication
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // Talk to the register 6B
  Wire.write(0x00); // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);

  //Making sure the claw and arm are in a set position at the start
  myarm.attach(13);  // (pin, min, max)
  myarm.write(180);
  arm.attach(2);
  arm.write(130);
  delay(500);

  //To avoid spiked readings when the robot turns on
  for (int i = 0; i < 200; i++) {
    mpu6050.getAngleX();
    mpu6050.update();
  }

}

//Moving forward
void forward(int speed) {
  BackRight ->setSpeed(speed);
  BackRight ->run(FORWARD);
  FrontRight -> setSpeed(speed);
  FrontRight ->run(FORWARD);
  BackLeft ->setSpeed(speed + 10);
  BackLeft ->run(FORWARD);
  FrontLeft -> setSpeed(speed + 10);
  FrontLeft ->run(FORWARD);
  Serial.println("Forward");
}

//Goes back
void back(int speed) {
  Serial.println("Left");

  BackRight ->setSpeed(speed);
  BackRight ->run(BACKWARD);
  FrontRight -> setSpeed(speed);
  FrontRight ->run(BACKWARD);
  BackLeft ->setSpeed(speed + 10);
  BackLeft ->run(BACKWARD);
  FrontLeft -> setSpeed(speed + 10);
  FrontLeft ->run(BACKWARD);
}

//Goes right
void right(int speedLeft, int speedRight) {
  Serial.println("Right");
  BackRight ->setSpeed(speedRight + 15);
  BackRight ->run(BACKWARD);
  FrontRight -> setSpeed(speedRight + 15);
  FrontRight ->run(BACKWARD);
  BackLeft ->setSpeed(speedLeft);
  BackLeft ->run(FORWARD);
  FrontLeft -> setSpeed(speedLeft);
  FrontLeft ->run(FORWARD);
  lastMove = "right";
}

//Goes left
void left(int speedLeft, int speedRight) {
  Serial.println("Left");
  BackRight ->setSpeed(speedRight);
  BackRight ->run(FORWARD);
  FrontRight -> setSpeed(speedRight);
  FrontRight ->run(FORWARD);
  BackLeft ->setSpeed(speedLeft + 15);
  BackLeft ->run(BACKWARD);
  FrontLeft -> setSpeed(speedLeft + 15);
  FrontLeft ->run(BACKWARD);
  lastMove = "left";
}
void slowTurn(int speedLeft, int speedRight) {
  BackRight ->setSpeed(speedRight);
  BackRight ->run(FORWARD);
  FrontRight -> setSpeed(speedRight);
  FrontRight ->run(FORWARD);
  BackLeft ->setSpeed(speedLeft);
  BackLeft ->run(FORWARD);
  FrontLeft -> setSpeed(speedLeft);
  FrontLeft ->run(FORWARD);
}

//Stops
void fullyStop() {
  BackRight ->run(RELEASE);
  FrontRight ->run(RELEASE);
  BackLeft ->run(RELEASE);
  FrontLeft ->run(RELEASE);
}

//Very complex stuff that I got off the internet for gyro readings
void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
  float recipNorm;
  float vx, vy, vz;
  float ex, ey, ez;  //error terms
  float qa, qb, qc;
  static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
  float tmp;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)

  //  tmp = ax * ax + ay * ay + az * az;
  tmp = 0.0; //IGNORE ACCELEROMETER

  // ignore accelerometer if false (tested OK, SJR)
  if (tmp > 0.0)
  {

    // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
    recipNorm = 1.0 / sqrt(tmp);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity in the body frame (factor of two divided out)
    vx = q[1] * q[3] - q[0] * q[2];
    vy = q[0] * q[1] + q[2] * q[3];
    vz = q[0] * q[0] - 0.5f + q[3] * q[3];

    // Error is cross product between estimated and measured direction of gravity in body frame
    // (half the actual magnitude)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Compute and apply to gyro term the integral feedback, if enabled
    if (Ki > 0.0f) {
      ix += Ki * ex * deltat;  // integral error scaled by Ki
      iy += Ki * ey * deltat;
      iz += Ki * ez * deltat;
      gx += ix;  // apply integral feedback
      gy += iy;
      gz += iz;
    }

    // Apply proportional feedback to gyro term
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;
  }

  // Integrate rate of change of quaternion, given by gyro term
  // rate of change = current orientation quaternion (qmult) gyro rate

  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply common factors
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];

  //add qmult*delta_t to current orientation
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}

//Returns the x-rotational angle of the robot
float gyroReading(int x) {
  static unsigned int i = 0; //loop counter
  static float deltat = 0;  //loop time in seconds
  static unsigned long now = 0, last = 0; //micros() timers
  static long gsum[3] = {0};
  //raw data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t Tmp; //temperature

  //scaled data as vector
  float Axyz[3];
  float Gxyz[3];


  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14); // request a total of 14 registers
  int t = Wire.read() << 8;
  ax = t | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  t = Wire.read() << 8;
  ay = t | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  t = Wire.read() << 8;
  az = t | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  t = Wire.read() << 8;
  Tmp = t | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  t = Wire.read() << 8;
  gx = t | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  t = Wire.read() << 8;
  gy = t | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  t = Wire.read() << 8;
  gz = t | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // calibrate gyro upon startup. SENSOR MUST BE HELD STILL (a few seconds)
  i++;
  if (cal_gyro) {

    gsum[0] += gx; gsum[1] += gy; gsum[2] += gz;
    if (i == 500) {
      cal_gyro -= 1;  //turn off calibration and print results

      for (char k = 0; k < 3; k++) G_off[k] = ((float) gsum[k]) / 500.0;

      Serial.print("G_Off: ");
      Serial.print(G_off[0]);
      Serial.print(", ");
      Serial.print(G_off[1]);
      Serial.print(", ");
      Serial.print(G_off[2]);
      Serial.println();
    }
  }

  // normal AHRS calculations

  else {
    Axyz[0] = (float) ax;
    Axyz[1] = (float) ay;
    Axyz[2] = (float) az;

    //apply offsets and scale factors from Magneto
    for (i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

    Gxyz[0] = ((float) gx - G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float) gy - G_off[1]) * gscale;
    Gxyz[2] = ((float) gz - G_off[2]) * gscale;

    //  snprintf(s,sizeof(s),"mpu raw %d,%d,%d,%d,%d,%d",ax,ay,az,gx,gy,gz);
    //  Serial.println(s);

    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;

    Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

    // Compute Tait-Bryan angles. Strictly valid only for approximately level movement

    // In this coordinate system, the positive z-axis is down toward Earth.
    // yaw1 is the angle between Sensor x-axis and Earth magnetic North
    // (or true North if corrected for local declination, looking down on the sensor
    // positive yaw1 is counterclockwise, which is not conventional for NED navigation.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
    // arise from the definition of the homogeneous rotation matrix constructed
    // from quaternions. Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw1,
    // pitch, and then roll.

    roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    //conventional yaw1 increases clockwise from North. Not that the MPU-6050 knows where North is.
    yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
    // to degrees
    yaw   *= 180.0 / PI;
    if (yaw < 0) yaw += 360.0; //compass circle
    //ccrrect for local magnetic declination here
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;

    if (x == 2) {
      Serial.print(yaw, 0);
      Serial.print(", ");
      Serial.print(pitch, 0);
      Serial.print(", ");
      Serial.println(roll, 0);
    }
    if (x == 0)return yaw;
    else if (x == 1) return roll;
  }
}


//Function that detects the light readings on the QTI Sensor
long RCTime(int sensorIn) {
  long duration = 0;
  pinMode(sensorIn, OUTPUT);
  digitalWrite(sensorIn, HIGH);
  delay(1);
  pinMode(sensorIn, INPUT);
  digitalWrite(sensorIn, LOW);
  while (digitalRead(sensorIn)) {
    duration++;

    if (duration > 200) {
      return duration;
    }
  }
  return duration;
}


void debugging() {
  //Printing the sensor readings for debugging
  Serial.print(RCTime(leftLightSensor));
  Serial.print(" ");
  Serial.print(analogRead(midLeftLine));
  Serial.print(" ");
  Serial.print(analogRead(midLine));
  Serial.print(" ");
  Serial.print(analogRead(midRightLine));
  Serial.print(" ");
  Serial.println(RCTime(rightLightSensor));
}

//Basic line follow
void movement() {
  //Right line
  if (analogRead(midRightLine) < 70 && RCTime(rightLightSensor) > 70 && analogRead(midLine) < 70) {
    right(80, 90);
  }

  //Left line
  else if (analogRead(midLeftLine) < 70 && analogRead(midLine) < 70 && RCTime(leftLightSensor) > 70) {
    left(90, 80);
  }

  //Right line
  else if ((analogRead(midRightLine) < 70)) {
    right(80, 90);
  }

  //Left line
  else if ((analogRead(midLeftLine) < 70)) {
    left(90, 80);
  }

  //No black readings other than the middle line
  else {
    forward(60);
  }
}

void checkSensors() {
  //Reading RGB values using the color sensor for as much information as possible
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  red = pulseIn(colourSensor, digitalRead(colourSensor) == HIGH ? LOW : HIGH);
  digitalWrite(S3, HIGH);
  blue = pulseIn(colourSensor, digitalRead(colourSensor) == HIGH ? LOW : HIGH);
  digitalWrite(S2, HIGH);
  green = pulseIn(colourSensor, digitalRead(colourSensor) == HIGH ? LOW : HIGH);
  digitalWrite(S20, LOW);
  digitalWrite(S30, LOW);
  red0 = pulseIn(colourSensor0, digitalRead(colourSensor0) == HIGH ? LOW : HIGH);
  digitalWrite(S30, HIGH);
  blue0 = pulseIn(colourSensor0, digitalRead(colourSensor0) == HIGH ? LOW : HIGH);
  digitalWrite(S20, HIGH);
  green0 = pulseIn(colourSensor0, digitalRead(colourSensor0) == HIGH ? LOW : HIGH);

  //Debugging sake
  Serial.print("R=");
  Serial.print(red);
  Serial.print(" G=");
  Serial.print(green);
  Serial.print(" B=");
  Serial.println(blue);
  Serial.print("R=");
  Serial.print(red0);
  Serial.print(" G=");
  Serial.print(green0);
  Serial.print(" B=");
  Serial.println(blue0);
  Serial.println(".........");
}

//Checks if robot detects green
boolean detectGreen() {
  checkSensors();
  checkSensors();

  //Right color sensor often glitches and gives really low readings but still can differentiate between green and white, must change green values if that occurs
  if (green0 < 100) {
    rightGreen = newGreen;
  }
  else {
    rightGreen = 500;
  }

  //If it finds green, it may break out of the loop
  if ((green > leftGreen && red > leftGreen && blue > leftGreen) || (green0 > rightGreen && red0 > rightGreen && blue0 > rightGreen)) {
    return true;
  }
  return false;
}

//Moving accordingly to green
void greenMovement() {
  Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");


  //Rechecking green frequencies
  checkSensors();
  checkSensors();

  if (green0 < 100) {
    rightGreen = newGreen;
  }
  else {
    rightGreen = 375;
  }

  //Two greens
  if ((green > leftGreen && red > leftGreen && blue > leftGreen) && (green0 > rightGreen && red0 > rightGreen && blue0 > rightGreen)) {

    Serial.println("Turn 180");
    right(100, 90);
    delay(3600);

    //Stop turning once it realigns itself
    while (analogRead(midLine) > 400) {
      right(100, 100);

    }
  }

  //Right Green
  else if (green0 > rightGreen && red0 > rightGreen && blue0 > rightGreen) {
    Serial.println("Turn Right");
    forward(70);
    delay(500);
    right(90, 90);
    delay(1200);
    //Stop turning once it realigns itself
    while (analogRead(midLeftLine) > 400) {
      Serial.println("Turning right GREEN");
      right(90, 90);
    }
  }
  else if ((green > leftGreen && red > leftGreen && blue > leftGreen)) {
    Serial.println("Turn Left");
    forward(70);
    delay(500);
    left(90, 70);
    delay(2000);
    //Stop turning once it realigns itself
    while (analogRead(midLine) > 400) {
      Serial.println("Turning Left");
      left(90, 70);
    }

    forward(80);
    delay(200);
  }
  else {
    Serial.println("NA");
  }
}

void adjustment(String lastTurn) {
  fullyStop();
  delay(200);

  double curAngle = gyroReading(0);
  double counter = 0.0;
  double refAngle = 1.0;
  int initialAngle = curAngle;

  //Finding the reference angle to realign the robot based on angular velocity, will align itself using angles that are multiples of 90 or 0
  boolean found = false;
  while (not found) {
    refAngle = 90.0 * counter;
    if (abs(curAngle - refAngle) < 45.0) {
      found = true;
      break;
    }
    else {

      curAngle = gyroReading(0);
      counter += 1.0;

      if (counter > 10) {
        counter = 0;
      }
    }
  }

  //After its found, the robot needs to turn to that angle to realign
  while (abs(curAngle - refAngle) > 0.2) {
    if (curAngle > 315 && refAngle == 0.0) {
      right(75, 75);
    }
    else if (curAngle < 45 && refAngle == 360) {
      left(75, 75);
    }

    else if (curAngle > refAngle) {
      left(75, 75);
    }
    else {
      right(75, 75);
    }
    curAngle = gyroReading(0);
    Serial.print(curAngle);
    Serial.print("      ");
    Serial.println(refAngle);
  }


  fullyStop();
  delay(200);
}

String movementJunction() {
  Serial.println("(((((((((((((((((((((((((((((((((((");


  //Aligns the robot using the angular velocity relative to starting position
  if (lastMove.equals("left")) {
    adjustment("left");
  }
  else if (lastMove.equals("right")) {
    adjustment("right");
  }

  //Checking what intersection its in, specifically looking for greens
  for (int i = 0; i < 30; i++) {
    Serial.println(i);

    //Only checks for green for a few iterations, otherwise it may read black as green
    if (i < 8) {

      //If it detected green
      if (detectGreen()) {
        fullyStop();
        delay(500);
        greenMovement();
        return "none";
      }
    }
    forward(50);
    delay(1);
  }

  fullyStop();
  delay(500);

  //If there is a black after the check, its a intersection that goes straight
  if (analogRead(midRightLine) < 50 || analogRead(midLine) < 50 || analogRead(midLeftLine) < 50 || RCTime(rightLightSensor) > 70 || RCTime(leftLightSensor) > 70) {
    debugging();
    return "none";
  }

  else {
    //Its a 90 degree turn
    //Goes back to original turn
    while (analogRead(midLine) > 50) {
      back(70);
    }
    forward(100);
    delay(100);

    Serial.println("22222222222222222222222222222222222222222222222");
    fullyStop();
    delay(500);
    return lastMove;
  }
}
void objectDetection() {
  distance = sr04.Distance();
  //Checking if there is an object in vicinity
  if (distance <= 5 && distance != 0) {
    objectAvoidence();
  }
}
void objectAvoidence() {
  //Rams into the object
  forward(100);
  delay(2000);
}


void checkJunction() {
  //This code is to check what kind of intersection we are in: 90 degree, green 180, straight through, etc.
  boolean isFound = false;
  mpu6050.update();
  //If far right light sensor detect black, its a right turn
  if (analogRead(midRightLine) < 70 && analogRead(midLeftLine) < 70 && analogRead(midLine) < 70 && RCTime(rightLightSensor) > 70) {
    turnType = "right";
    isFound = true;
  }

  //If far left light sensor detect black, its a left turn
  else if (analogRead(midRightLine) < 70 && analogRead(midLeftLine) < 70 && analogRead(midLine) < 70 && RCTime(leftLightSensor) > 70) {
    turnType = "left";
    isFound = true;
  }

  if (isFound) {

    //Check for green tiles, straight through, or just a regular 90 degree turn
    String temp = movementJunction();


    //If temp equals some kind of turn that means is a 90 degree turn, so it needs to go back to where it read the black and manually turn
    //Manual left turn
    if (!temp.equals("none")) {
      if (turnType.equals("left")) {
        forward(70);
        delay(700);
        left(80, 80);
        delay(2000);

        //Catching the black line
        while (analogRead(midLine) > 50) {
          Serial.println("Fixing left");
          left(90, 90);
        }
      }

      //Manual right turn
      else if (turnType.equals("right")) {
        forward(70);
        delay(700);
        right(80, 80);
        delay(2000);

        //Catching the black line
        while (analogRead(midLine) > 50) {
          Serial.println("Fixing right");
          right(90, 90);
        }
      }
    }
  }
}

void checkRamp() {
  //If robot is on an incline
  if (mpu6050.getAngleX() - 4.24 > 15) {

    //Will run while on ramp
    while (mpu6050.getAngleX() - 4.24 > 15) {
      mpu6050.update();

      //We slowturn instead of regular turn to avoid battling against gravity
      //The parameters in the slowturn are the speeds of the wheels spinning forward
      if (analogRead(midRightLine) < 70 && RCTime(rightLightSensor) > 70 && analogRead(midLine) < 70) {
        slowTurn(110, 70);
      }

      //Same idea but this is the left turn
      else if (analogRead(midLeftLine) < 70 && analogRead(midLine) < 70 && RCTime(leftLightSensor) > 70) {
        slowTurn(70, 110);
      }

      //Right line
      else if ((analogRead(midRightLine) < 70)) {
        slowTurn(110, 70);
      }

      //Left line
      else if ((analogRead(midLeftLine) < 70)) {
        slowTurn(70, 110);
      }

      //No black readings other than the middle line
      else {
        forward(80);
      }
    }

    //Get over ramp just in case if it stops right on the edge
    forward(100);
    delay(1000);
  }
}

void checkCamera() {
  int x, y, num_balls;
  String color;
  if (Serial1.available()) {
    // Receive number of balls
    while (Serial1.available() < 1) {}  // Wait for 1 byte
    num_balls = Serial1.read();
    
    // Receive information for each ball
    while (Serial1.available() < num_balls * 4) {}  // Wait for all data
    for (int i = 0; i < num_balls; i++) {
      x = Serial1.read();
      x |= Serial1.read() << 8;
      y = Serial1.read();
      y |= Serial1.read() << 8;

      Serial.print("Received coordinates for ball ");
      Serial.print(i);
      Serial.print(": (");
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.println(")");
    }

    //These thousand values determine what color the ball is since UART struggles to send chars and ints 
    //Each color has a thousand range which indicates color of ball, then we must reduce the x and y to actual coordinates
    if (999 < x && x < 2000) {
      color = "black";
      x -= 1000;
      y -= 1000;
    }
    else if (1999 < x && x < 3000) {
      color = "blue";
      x -= 2000;
      y -= 2000;
    }
    else if (2999 < x && x < 4000) {
      color = "silver";
      x -= 3000;
      y -= 3000;
    }
    else if (3999 < x && x < 5000) {
      color = "red";
      x -= 4000;
      y -= 4000;
    }
    //If the ball is close enough to pickup
    if (150 < x && x < 180 && y > 230) {
      while (sr04.Distance() > 10) {
        forward(100);
      }

      fullyStop();
      delay(500);

      //Drop the arm and scoop the ball
      arm.write(0);
      claw.write(180);
      delay(2000);

      //Lift it up into the basket
      arm.write(130);
      claw.write(0);
      delay(1000);
    }

    //Ball on left side of camera
    else if (150 > x) left(80, 80);
    //Right side of camera
    else if (180 < x) right(80, 80);
    //In case if received bytes incorrectly
    else fullyStop();
  }
  //If nothing is received, spin the robot to search for balls
  else left(60, 60);
}

void loop() {
  //  Line Follow
  
  movement();
  checkJunction();
  objectDetection();

  //  Didn't work well
  
//  checkRamp();
//  checkCamera();


  //  Debugging Code
  
  //  debugging();
  //  checkSensors();
  //  Serial.println(mpu6050.getAngleX() - 4.24);
  //  Serial.print("       ");
  //  Serial.println(mpu6050.getAccY());
  //  mpu6050.update();
  //  adjustment("n");
  //  Serial.println("test");
  //  testingarm();
  //  Serial.println(sr04.Distance());
}
