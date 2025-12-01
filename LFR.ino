// Motor A (Left Motor)
#define IN1 9
#define IN2 8
#define ENA 10 
// Motor B (Right Motor)
#define IN3 7
#define IN4 6
#define ENB 5

// Ultraonic Sensor 
#define TRIG_PIN 3
#define ECHO_PIN 4
#define STOP_DISTANCE 10   

bool isBlackLine = 1;           
unsigned int numSensors = 8;      
//-----------------------------------------

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 110;
int currentSpeed = 100;
int sensorWeight[8] = { 7, 5, 3, 1, -1, -3, -5, -7 };

int extremeTurnBrakespeed = 100;
int extremeTurnSpeed = 150;

int activeSensors;
float Kp = 0.04;
float Kd = 3;
float Ki = 0;
int onLine = 1;

// Taking digital input from sensors A0 to A5b and analog for A6 to A7
// Only A6, A7
int minValues[2], maxValues[2], threshold[2];

int sensorValue[8], sensorArray[8];

// Ultrasonic sensor timing variables
unsigned long lastUltrasonicCheck = 0;
const unsigned int ULTRASONIC_INTERVAL = 50;
float lastDistance = 999;

void setup() {
  Serial.begin(9600);

  // Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Digital sensor pins (A0-A5)
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  
  // Ultrasonic Sensor Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // LED
  pinMode(13, OUTPUT); 

}

void loop() {
  calibrate();
  
  while (1) {

    unsigned long currentMillis = millis();
    if (currentMillis - lastUltrasonicCheck >= ULTRASONIC_INTERVAL) {
      lastUltrasonicCheck = currentMillis;
      lastDistance = getDistance();
    }
    
    if (lastDistance <= STOP_DISTANCE) {
      
      motorLeftRun(-200);
      motorRightRun(-200);
      delay(200);
      motorLeftRun(0);
      motorRightRun(0);
      
      while (getDistance() <= STOP_DISTANCE) {
        delay(50);
      }
      
      // Waiting 3 seconds before moving
      // delay(2000);
      lastDistance = 999; // Reset distance
    }
    
    readLine();
    bool allEqual = true;
    for (int i = 1; i < 8; i++) {
      if (sensorArray[i] != sensorArray[0]) {
        allEqual = false;
        break;
      }
    }
    
    if (currentSpeed < lfSpeed) currentSpeed++;
    
    //Extreme Left
    if (sensorArray[7] == 1 && sensorArray[6] == 1 && sensorArray[5] == 1 && sensorArray[4] == 1 && sensorArray[3] == 1 && sensorArray[0] == 0) {
      digitalWrite(13, LOW);
      if (lsp > extremeTurnBrakespeed && rsp > extremeTurnBrakespeed) {
        motorLeftRun(-extremeTurnBrakespeed);
        motorRightRun(-extremeTurnBrakespeed);
      }
      delay(1);
      motorLeftRun(0);
      motorRightRun(0);
      delay(10);

      motorLeftRun(-extremeTurnSpeed);
      motorRightRun(extremeTurnSpeed);
      while (true) {
        readLine();
        if (sensorArray[3] == 1 && sensorArray[0] == 0 && sensorArray[7] == 0) {
          motorLeftRun(0);
          motorRightRun(0);
          break;
        }
      }
    }
    // Extreme Right
    else if (sensorArray[0] == 1 && sensorArray[1] == 1 && sensorArray[2] == 1 && sensorArray[3] == 1 && sensorArray[4] == 1 && sensorArray[7] == 0) {
      digitalWrite(13, LOW);
      if (lsp > extremeTurnBrakespeed && rsp > extremeTurnBrakespeed) {
        motorLeftRun(-extremeTurnBrakespeed);
        motorRightRun(-extremeTurnBrakespeed);
      }
      delay(1);
      motorLeftRun(0);
      motorRightRun(0);
      delay(10);
      
      motorLeftRun(extremeTurnSpeed);
      motorRightRun(-extremeTurnSpeed);
      while (true) {
        readLine();
        if (sensorArray[3] == 1 && sensorArray[0] == 0 && sensorArray[7] == 0) {
          motorLeftRun(0);
          motorRightRun(0);
          break;
        }
      }
    }

    // usual PID line follow
    else if (!allEqual && onLine == 1) {  // PID LINE FOLLOW
      linefollow();
      digitalWrite(13, HIGH);
    }
  }
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 5000);
  
  if (duration == 0) {
    return 999;
  }
  
  float distance = duration * 0.034 / 2;
  return distance;
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  for (int i = 0; i < 8; i++) {
    error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
    activeSensors += sensorArray[i];
  }

  if (activeSensors > 0) {
    error = error / activeSensors;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed + PIDvalue; 
  rsp = currentSpeed - PIDvalue;

  if (lsp > 255) lsp = 255;
  if (lsp < -255) lsp = -255;

  if (rsp > 255) rsp = 255;
  if (rsp < -255) rsp = -255;

  motorLeftRun(lsp);
  motorRightRun(rsp);
}

void calibrate() {

  minValues[0] = analogRead(A6);
  maxValues[0] = analogRead(A6);
  minValues[1] = analogRead(A7);
  maxValues[1] = analogRead(A7);
  
  // Blinking LEDs to show callibration clearly

  for (int j = 0; j < 10000; j++) {
    if (j % 1000 == 0) {
      digitalWrite(13, !digitalRead(13));
    }
    
    int val6 = analogRead(A6);
    if (val6 < minValues[0]) minValues[0] = val6;
    if (val6 > maxValues[0]) maxValues[0] = val6;
    
    int val7 = analogRead(A7);
    if (val7 < minValues[1]) minValues[1] = val7;
    if (val7 > maxValues[1]) maxValues[1] = val7;
  }
  
  threshold[0] = (minValues[0] + maxValues[0]) / 2;
  threshold[1] = (minValues[1] + maxValues[1]) / 2;
  
  
  digitalWrite(13, LOW);
}


void readLine() {
  onLine = 0;
  
  for (int i = 0; i < 6; i++) {
    int digitalValue = digitalRead(A0 + i);
    
    sensorArray[i] = isBlackLine ? digitalValue : !digitalValue;

    sensorValue[i] = sensorArray[i] ? 1000 : 0;
    
    if (sensorArray[i]) onLine = 1;
  }
  
  // Read analog sensors (A6-A7)
  for (int i = 0; i < 2; i++) {
    int analogValue = analogRead(A6 + i);
    
    if (isBlackLine) {
      sensorValue[6 + i] = map(analogValue, minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[6 + i] = map(analogValue, minValues[i], maxValues[i], 1000, 0);
    }
    
    sensorValue[6 + i] = constrain(sensorValue[6 + i], 0, 1000);
    sensorArray[6 + i] = sensorValue[6 + i] > 500;
    
    if (sensorArray[6 + i]) onLine = 1;
  }
}

void motorLeftRun(int motorSpeed) {
  if (motorSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(motorSpeed));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void motorRightRun(int motorSpeed) {
  if (motorSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(motorSpeed));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}
