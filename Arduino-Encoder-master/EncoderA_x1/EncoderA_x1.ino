#include <AFMotor.h>

//#define ENC_COUNT_PER_ROTATION (1450)
#define ENC_COUNT_PER_ROTATION (1615)
#define ENC_A_PIN_NUM (18)
#define ENC_B_PIN_NUM (19)

#define ROBOT_CENTRE_TO_WHEEL_DISTANCE (11) //cm
#define WHEEL_RADIUS (3.5) //cm
#define RADIUS_RATIO (ROBOT_CENTRE_TO_WHEEL_DISTANCE / WHEEL_RADIUS)

int encCount = 0;
int turnFinished = 0;

AF_DCMotor motor1(1, MOTOR12_64KHZ); // Create motor #1, 64KHz pwm
AF_DCMotor motor2(2, MOTOR12_64KHZ); // Create motor #2, 64KHz pwm
AF_DCMotor motor3(3, MOTOR34_1KHZ); // Create motor #3, 1KHz pwm
AF_DCMotor motor4(4, MOTOR34_1KHZ); // Create motor #4, 1KHz pwm

void setup()
{
  Serial.begin(9600);

  pinMode(51, OUTPUT);
  digitalWrite(51, HIGH);

  pinMode(ENC_A_PIN_NUM, INPUT);
  pinMode(ENC_B_PIN_NUM, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN_NUM), ENC_PHASE_A_CHANGE_ISR, CHANGE);
}

// Turn robot a fraction of a 360 degree turn
void turnRobot(double turnFraction)
{
  int turnEncCount = -1 * (int)(turnFraction * ENC_COUNT_PER_ROTATION * RADIUS_RATIO);
  Serial.print("Encoder count to complete turn: ");
  Serial.println(turnEncCount);

  // Right turn, encoder count decreases
  if (turnFraction > 0)
  {
    // Left motors
    motor1.run(FORWARD);
    motor1.setSpeed(255);
    motor4.run(FORWARD);
    motor4.setSpeed(255);

    // Right motors
    motor2.run(BACKWARD);
    motor2.setSpeed(255);
    motor3.run(BACKWARD);
    motor3.setSpeed(255);

    while(encCount > turnEncCount) {
      Serial.print("Encoder count: ");
      Serial.println(encCount);
    }
  }

  // Left turn, encoder count increases
  else
  {
    // Left motors
    motor1.run(BACKWARD);
    motor1.setSpeed(255);
    motor4.run(BACKWARD);
    motor4.setSpeed(255);

    // Right motors
    motor2.run(FORWARD);
    motor2.setSpeed(255);
    motor3.run(FORWARD);
    motor3.setSpeed(255);

    while(encCount < turnEncCount) {
      Serial.print("Encoder count: ");
      Serial.println(encCount);
    }
  }

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

  encCount = 0;
  turnFinished = 1;
}

void loop()
{
  /*
  if(encCount >= total)
  {
    motor3.setSpeed(0);
  }
  else
  {
    motor3.run(FORWARD);
    motor3.setSpeed(255);
    Serial.println(encCount);
  }

  if(turnFinished == 0)
  {
    turnRobot(0.25);
    delay(1000);
  }
  */

  turnRobot(0.25);
  delay(1000);
}

// Service a change in encoder phase A logic level
void ENC_PHASE_A_CHANGE_ISR()
{
  if(digitalRead(ENC_B_PIN_NUM) == 0)
  {
    if (digitalRead(ENC_A_PIN_NUM) == 0)
    {
      // A fell, B is low
      encCount--; // Moving backwards
    }
    else
    {
      // A rose, B is low
      encCount++; // Moving forwards
    }
  }
}
