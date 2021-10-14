#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

int leftMotorVal = 0;
int rightMotorVal = 0;

unsigned long tStart = 0;

uint16_t cmd_buffer_pos = 0;
const uint8_t CMD_BUFFER_LEN = 20;
char cmd_buffer[CMD_BUFFER_LEN];

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
  tStart = millis();
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    
    if (ch == '\r') {
      Serial.println("New line detected");
      cmd_buffer[cmd_buffer_pos] = '\0';
      parseCommandBuffer();
      cmd_buffer_pos = 0;
      memset(cmd_buffer, 0, sizeof(cmd_buffer));
    } else if (cmd_buffer_pos == CMD_BUFFER_LEN - 1) {
      cmd_buffer_pos = 0;
      memset(cmd_buffer, 0, sizeof(cmd_buffer));
      cmd_buffer[cmd_buffer_pos] = ch;
      cmd_buffer_pos++;
    } else {
      cmd_buffer[cmd_buffer_pos] = ch;
      cmd_buffer_pos++;
    }
  }
  
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
