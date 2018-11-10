#include "Arduino.h"
#include "pins_arduino.h"
#include "TBMotor.h"


OseppTBMotor Motor1(12, 11);
OseppTBMotor Motor2(8, 3);
//OseppTBMotor Motor3(7, 6);
//OseppTBMotor Motor4(4, 5);




volatile long pos_left_p = 0;
volatile long pos_left_n = 0;
volatile long pos_right_p = 0;
volatile long pos_right_n = 0;

volatile unsigned long counter[4];

ISR (PCINT1_vect)
{
  static char oldPin = 0xff;
  char in = PINC;
  char change = in ^ oldPin;
  if (change & 1)
  {
    counter[0]++;
    if (in & 1)
    {
      if (in & 2)pos_left_p++; else pos_left_n++;
    }
    else
    {
      if (in & 2)pos_left_n++; else pos_left_p++;
    }
  }
  else if (change & (1 << 1))
  {
    counter[1]++;
    if (in & (1 << 1))
    {
      if (in & 1)pos_left_n++; else pos_left_p++;
    }
    else
    {
      if (in & 1)pos_left_p++; else pos_left_n++;
    }
  }

  if (change & (1 << 2))
  {
    counter[2]++;
    if (in & (1 << 2))
    {
      if (in & (1 << 3))pos_right_p++; else pos_right_n++;
    }
    else
    {
      if (in & (1 << 3))pos_right_n++; else pos_right_p++;
    }

  }
  else if (change & (1 << 3))
  {
    counter[3]++;
    if (in & (1 << 3))
    {
      if (in & (1 << 2))pos_right_n++; else pos_right_p++;
    }
    else
    {
      if (in & (1 << 2))pos_right_p++; else pos_right_n++;
    }

  }
  oldPin = in;
}


void setup() {

  Serial.begin(115200);

  TCCR2B &= ~7;
  TCCR2B |= 1;

  pinMode(A0, INPUT_PULLUP); pinMode(A1, INPUT_PULLUP); pinMode(A2, INPUT_PULLUP); pinMode(A3, INPUT_PULLUP);
  PCIFR = 1 << 1;
  PCMSK1 = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
  PCICR = 1 << 1;
  sei();

}

int max_e_left = 0;
int max_e_right = 0;

void loop() {


  if (counter[0] || counter[1])
  {
    Serial.print("left error:");
    long diff = counter[0] - counter[1];
    diff *= 100;
    diff = abs(diff);
    long sum = counter[0] + counter[1];
    sum >>= 1;
    int e = diff / sum;
    max_e_left = max(max_e_left, e);
    Serial.print(max_e_left);
    Serial.print("%\t");
  } else {
    Serial.print("error:0%\t");
  }

  if (counter[2] || counter[3])
  {
    Serial.print("right error:");
    long diff = counter[2] - counter[3];
    diff *= 100;
    diff = abs(diff);
    long sum = counter[2] + counter[3];
    sum >>= 1;
    int e = diff / sum;
    max_e_right = max(max_e_right, e);
    Serial.print(max_e_right);
    Serial.println("%");
  } else {
    Serial.println("error:0%");
  }
  Motor1.forward(100 + random(150));
  Motor2.forward(100 + random(150));
  delay(500);

}



