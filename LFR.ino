// variable to store sensor value 
int a, b, c, d, e;

// Variables for storing IR sensor
int svel, svl, svc, svr, sver;

// Pin definition
int svelPin = A1;
int svlPin = A2;
int svcPin = A3;
int svrPin = A4;
int sverPin = A5;

// PID variables
float Kp = 0.6;   // Proportional gain
float Ki = 0.1;   // Integral gain
float Kd = 0.2;   // Derivative gain

float error = 0, previousError = 0;
float integral = 0;
float derivative = 0;
float correction = 0;

// Base motor speed
int baseSpeed = 70;

// PWM pins for motor control
const int enA = 9; // Left motor
const int enB = 3; // Right motor

// Direction control pins
const int in1 = 10, in2 = 11; // Left motor
const int in3 = 5, in4 = 4;   // Right motor

void setup() {
  Serial.begin(9600);
  pinMode(svelPin, INPUT);
  pinMode(svlPin, INPUT);
  pinMode(svcPin, INPUT);
  pinMode(svrPin, INPUT);
  pinMode(sverPin, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enA, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
}

void loop() {
  // Read sensor values
  svel = analogRead(svelPin);
  svl = analogRead(svlPin);
  svc = analogRead(svcPin);
  svr = analogRead(svrPin);
  sver = analogRead(sverPin);

  // Convert sensor readings to binary (line detected or not)
  a = (svel < 50) ? 0 : 1;
  b = (svl < 50) ? 0 : 1;
  c = (svc < 50) ? 0 : 1;
  d = (svr < 50) ? 0 : 1;
  e = (sver < 50) ? 0 : 1;

  // Calculate error based on sensor values
  error = (-2 * a) + (-1 * b) + (0 * c) + (1 * d) + (2 * e);

  // Calculate PID terms
  integral += error; // Accumulate the error for the integral term
  derivative = error - previousError; // Change in error for the derivative term

  // PID correction
  correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Update previous error for the next loop
  previousError = error;

  // Adjust motor speeds
  int leftMotorSpeed = baseSpeed + correction;
  int rightMotorSpeed = baseSpeed - correction;

  // Constrain motor speeds to valid range (0-255)
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  // Move the robot
  moveMotors(leftMotorSpeed, rightMotorSpeed);
}

void moveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, leftSpeed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, -leftSpeed);
  }

  if (rightSpeed > 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, rightSpeed);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, -rightSpeed);
  }
}
