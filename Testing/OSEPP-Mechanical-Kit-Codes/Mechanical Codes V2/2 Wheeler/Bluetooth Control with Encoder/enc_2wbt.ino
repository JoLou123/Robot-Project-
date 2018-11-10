#include "Arduino.h"
#include "pins_arduino.h"
#include "TBMotor.h"
#include "oseppRemote.h"
#include <avr/wdt.h>

OseppTBMotor Motor1(12, 11);
OseppTBMotor Motor2(8, 3);
//OseppTBMotor Motor3(7, 6);
//OseppTBMotor Motor4(4, 5);
OseppRemote remote;
extern bool updateIMU();


volatile long pos_left = 0;
volatile long pos_right = 0;

const float offset_a = 0.0;

int leftSpeed;
int rightSpeed;

PID pidStand(20.0, 1.0, 60.0, 0.0);
PID pidLSpeed(0.2, 0.4, 0.0, 0.0);
PID pidRSpeed(0.2, 0.4, 0.0, 0.0);

extern float angle;

unsigned char state = 0;
unsigned long ObstacleTime = 0;
float dist = 0.;
boolean enableGravity = false;
boolean enableUltrasonic = false;


ISR (PCINT1_vect)
{
  static char oldPin = 0xff;
  char in = PINC;
  char change = in ^ oldPin;
  if (change & 1)
  {
    if (in & 1)
    {
      if (in & 2)pos_left++; else pos_left--;
    }
    else
    {
      if (in & 2)pos_left--; else pos_left++;
    }
  }
  else if (change & (1 << 1))
  {
    if (in & (1 << 1))
    {
      if (in & 1)pos_left--; else pos_left++;
    }
    else
    {
      if (in & 1)pos_left++; else pos_left--;
    }
  }

  if (change & (1 << 2))
  {
    if (in & (1 << 2))
    {
      if (in & (1 << 3))pos_right--; else pos_right++;
    }
    else
    {
      if (in & (1 << 3))pos_right++; else pos_right--;
    }

  }
  else if (change & (1 << 3))
  {
    if (in & (1 << 3))
    {
      if (in & (1 << 2))pos_right++; else pos_right--;
    }
    else
    {
      if (in & (1 << 2))pos_right--; else pos_right++;
    }

  }
  oldPin = in;
}
void button_change(uint8_t buttonDown, uint8_t buttonUp)
{
  if (BUTTON_A & buttonUp) {
    enableUltrasonic = !enableUltrasonic;
    Serial.print("Ultrasonic:");
    if (enableUltrasonic)Serial.print("Enable"); else Serial.print("Disable");
    Serial.println();
  }
  if (BUTTON_B & buttonUp) {
    enableGravity = !enableGravity;
    Serial.print("Gravity:");
    if (enableGravity)Serial.print("Enable"); else Serial.print("Disable");
    Serial.println();
  }
}

void setup() {
  //Setup a watchdog
  //When the battery voltage is insufficient / program unexpected
  //Watchdog reset chip
  wdt_enable(WDTO_1S);
  wdt_reset();
  Serial.begin(9600);

  updateIMU();
  //Change the timer frequency
  //To avoid the frequency of hearing.
  TCCR2B &= ~7;
  TCCR2B |= 1;

  pinMode(A0, INPUT_PULLUP); pinMode(A1, INPUT_PULLUP); pinMode(A2, INPUT_PULLUP); pinMode(A3, INPUT_PULLUP);
  PCIFR = 1 << 1;
  PCMSK1 = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
  PCICR = 1 << 1;
  sei();
  remote.attachButtonChange(button_change);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  pinMode(4, INPUT_PULLUP);

}


void loop() {
  while (Serial.available() > 0) remote.feed(Serial.read());
  static bool noConnection = true;
  if (remote.isTimeoutFor(400)) {
    enableGravity = false;
    enableUltrasonic = false;
    noConnection = true;
  } else {
    if (noConnection) {
      /*
         At 9600 baud rate,118byte=118ms
         program will block here(118-64byte buffer=54ms
         2W will lose balance
         don't print things through the serial port
         unless you use a higher baud rate
      */
      //      Serial.println("use [L][U][R][D] OR Left Joysick");
      //      Serial.println("to control robot Motion");
      //      Serial.println("[A] enable/disable Ultrasonic");
      //      Serial.println("[B] enable/disable Gravity");
    }
    noConnection = false;
  }


  wdt_reset();

  if (!updateIMU())return; else angle = -angle - offset_a;
  static long last_pos_left = 0;
  static long last_pos_right = 0;
  long curLSpeed = pos_left - last_pos_left;
  long curRSpeed = pos_right - last_pos_right;
  last_pos_right = pos_right;
  last_pos_left =  pos_left;
  if (!noConnection) {
    if (enableUltrasonic) {
      if (state == 0)
      {
        if (abs(angle) < 20.0)
        {
          curLSpeed -= 30;
          curRSpeed -= 30;
        }
      } else if (state == 1) {
        curRSpeed -= 15;
        curLSpeed += 15;
      } else //if (state == 2)
      {
        curRSpeed += 10;
        curLSpeed += 10;
      }
    } else {
      if (remote.getButtonState(BUTTON_U)) {
        curLSpeed -= 30;
        curRSpeed -= 30;
      } else if (remote.getButtonState(BUTTON_D)) {
        curLSpeed += 30;
        curRSpeed += 30;
      } else if (remote.getButtonState(BUTTON_R)) {
        curLSpeed -= 25;
        curRSpeed += 25;
      } else if (remote.getButtonState(BUTTON_L)) {
        curLSpeed += 25;
        curRSpeed -= 25;
      } else {
        int l, r, x, y, z;
        x = remote.getChannalData(CHANNEL_LEFTJOYSTICK_X);
        y = remote.getChannalData(CHANNEL_LEFTJOYSTICK_Y);
        if ((x == 0) && (y == 0) && (enableGravity)) {
          x = remote.getChannalData(CHANNEL_GRAVITY_X);
          y = remote.getChannalData(CHANNEL_GRAVITY_Y);
          z = remote.getChannalData(CHANNEL_GRAVITY_Z);
          y += 127;
          y = constrain(y, -127, 127);
          y = map(y, -127, 127, -255, 255);
        }
        l = r = y;
        l += x;
        r -= x;
        int m = max(abs(l), abs(r));
        if (m > 255) {
          l = map(l, -m, m, -30, 30);
          r = map(r, -m, m, -30, 30);
        } else {
          l = map(l, -255, 255, -30, 30);
          r = map(r, -255, 255, -30, 30);
        }
        curLSpeed -= l;
        curRSpeed -= r;
      }
    }
  }
  pidStand.sum_err *= 0.99;

  float pwmStand =  pidStand.pid(angle);
  float pwmLSpeed = pidLSpeed.pid(curLSpeed);
  float pwmRSpeed = pidRSpeed.pid(curRSpeed);
  leftSpeed =  pwmStand + pwmRSpeed;
  rightSpeed = pwmStand + pwmLSpeed;

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  if (abs(angle) < 60)
  {
    Motor1.SetSpeed(leftSpeed);
    Motor2.SetSpeed(-rightSpeed);
  } else {
    Motor1.SetSpeed(-0);
    Motor2.SetSpeed(-0);

    pidStand.reset();
    pidRSpeed.reset();
    pidLSpeed.reset();
  }


  digitalWrite(2, LOW);
  delayMicroseconds(2);
  digitalWrite(2, HIGH);
  delayMicroseconds(10);
  digitalWrite(2, LOW);
  int trigTime = pulseIn(4, HIGH, 2000);
  float ping = trigTime / 5.8;
  if (trigTime == 0)ping = 2000.0 / 5.8;
  dist = dist * 0.7 + ping * 0.3;
  if (dist < 200.)
  {
    static unsigned long lastEchoMillis = 0;
    if (millis() - lastEchoMillis > 300) {
      lastEchoMillis = millis();
      //Serial.print("Warning:");
      //Serial.print(dist);
      //Serial.println("mm");
    }
    state = 2;
    ObstacleTime = millis();
  } else {
    if (millis() - ObstacleTime > 1500) {
      state = 0;
    } else if (millis() - ObstacleTime > 700) {
      state = 1;
    }
  }
}




