int a, b, c, d, e;


int svel, svl, svc, svr, sver;


int svelPin = A1;
int svlPin = A2;
int svcPin = A3;
int svrPin = A4;
int sverPin = A5;


float Kp = 0.6;  
float Ki = 0.1;   
float Kd = 0.2;   

float error = 0, previousError = 0;
float integral = 0;
float derivative = 0;
float correction = 0;


int baseSpeed = 70;


const int enA = 9; 
const int enB = 3; 


const int in1 = 10, in2 = 11; 
const int in3 = 5, in4 = 4;   

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
  
  svel = analogRead(svelPin);
  svl = analogRead(svlPin);
  svc = analogRead(svcPin);
  svr = analogRead(svrPin);
  sver = analogRead(sverPin);


  a = (svel < 50) ? 0 : 1;
  b = (svl < 50) ? 0 : 1;
  c = (svc < 50) ? 0 : 1;
  d = (svr < 50) ? 0 : 1;
  e = (sver < 50) ? 0 : 1;

  
  error = (-2 * a) + (-1 * b) + (0 * c) + (1 * d) + (2 * e);

  
  integral += error; 
  derivative = error - previousError; 

 
  correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  
  previousError = error;

  
  int leftMotorSpeed = baseSpeed + correction;
  int rightMotorSpeed = baseSpeed - correction;

  
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  
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
