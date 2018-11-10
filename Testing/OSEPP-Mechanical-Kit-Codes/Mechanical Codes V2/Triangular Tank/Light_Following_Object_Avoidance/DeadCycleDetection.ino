
//Sometimes the car will fall into the cycle of death
//We detect this
//then we can free the car in the main loop


int Left_Action_Detect = 0;
int Right_Action_Detect = 0;
int Action_Count = 0;

unsigned long fl_leftsite = 0;
unsigned long fl_righttsite = 0;
void deadCycleLog(int leftSpeed,int rightSpeed)
{
  if (leftSpeed > 0)
  {
    Left_Action_Detect++;
  } else if (leftSpeed < 0) {
    Left_Action_Detect--;
  }
  if (rightSpeed > 0)
  {
    Right_Action_Detect++;
  } else if (rightSpeed < 0) {
    Right_Action_Detect--;;
  }
  Action_Count++;
}
bool carInDeadCycle()
{
  bool result = false;
  if ((Action_Count > 500)) //2-5 s
  {
    //Wander forward and backward,
    //or one of the wheels did not move 
    //the car has been around in circles
    if ((abs(Right_Action_Detect) < 50) || (abs(Left_Action_Detect) < 50))result = true; 
    //Wander left and right
    if (abs(Left_Action_Detect + Right_Action_Detect) < 50)result = true;
    //Always go back
    if ((Left_Action_Detect < -50) && (Right_Action_Detect < -50))result = true;
    Action_Count = 0;
    Right_Action_Detect = 0;
    Left_Action_Detect = 0;
  }
  return result;
}


