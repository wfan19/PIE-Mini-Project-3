#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

int leftSens = A0;
int rightSens = A1;

int leftMotorVal = 0;
int rightMotorVal = 0;

unsigned long tStart = 0;

uint16_t cmd_buffer_pos = 0;
const uint8_t CMD_BUFFER_LEN = 20;
char cmd_buffer[CMD_BUFFER_LEN];


// PD params
float k_p = 0.1;
float k_d = 0;
int sensDiffPrev = 0;
int tPrevious = 0;
int baseSpeed = 30;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
  tStart = millis();
  tPrevious = tStart;
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    
    Serial.println(cmd_buffer);
    
    if (ch == '\r') {
      Serial.println("New line detected");
      cmd_buffer[cmd_buffer_pos] = '\0';
      parseCommandBuffer();
      cmd_buffer_pos = 0;
    } else if (cmd_buffer_pos == CMD_BUFFER_LEN - 1) {
      cmd_buffer_pos = 0;
      cmd_buffer[cmd_buffer_pos] = ch;
      cmd_buffer_pos++;
    } else {
      cmd_buffer[cmd_buffer_pos] = ch;
      cmd_buffer_pos++;
    }
  }

  //bangBang();
  pdControl();
  
  motorWrite();
  delay(50);
}

void parseCommandBuffer() {
  Serial.print("Command read: ");
  Serial.println(cmd_buffer);
  if (strncmp(cmd_buffer, "LM", 2) == 0) {
    int val = atoi(cmd_buffer + 2);
    leftMotorVal = val;
    motorWrite();
    
    Serial.print("Setting left motor value: ");
    Serial.println(val);
  } else if (strncmp(cmd_buffer, "RM", 2) == 0) {
    int val = atoi(cmd_buffer + 2);
    rightMotorVal = val;
    motorWrite();
    
    Serial.print("Setting right motor value: ");
    Serial.println(val);
  }
}

void motorWrite() {
  if (leftMotorVal > 0) {
    leftMotor->run(FORWARD);
  } else if (leftMotorVal < 0) {
    leftMotor->run(BACKWARD);
  } else {
    leftMotor->run(RELEASE);
  }

  if (rightMotorVal >= 0) {
    rightMotor->run(FORWARD);
  } else {
    rightMotor->run(BACKWARD);
  }
  
  leftMotor->setSpeed(abs(leftMotorVal));
  rightMotor->setSpeed(abs(rightMotorVal));
}

void bangBang() {
  int leftSensVal = analogRead(leftSens);
  int rightSensVal = analogRead(rightSens);
  int difference = leftSensVal - rightSensVal;

  if (difference > 0) {
    leftMotorVal = 0;
    rightMotorVal = 128;
    }
  else {
    leftMotorVal = -128;
    rightMotorVal = 0;
    }
  
}



void pdControl() {
  int leftSensVal = analogRead(leftSens);
  int rightSensVal = analogRead(rightSens);
  int sensDiff = leftSensVal - rightSensVal;
  int tElapsed = millis() - tPrevious;
  int motorDiff = k_p * sensDiff + k_d * (sensDiff-sensDiffPrev)/tElapsed;

  leftMotorVal = -(baseSpeed + motorDiff);
  rightMotorVal = baseSpeed - motorDiff;
  sensDiffPrev = millis();
  }
