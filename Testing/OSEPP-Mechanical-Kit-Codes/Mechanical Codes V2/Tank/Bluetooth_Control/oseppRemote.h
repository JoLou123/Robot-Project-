#ifndef __OSEPP_REMOTE_H__
#define __OSEPP_REMOTE_H__

#define BUTTON_L (1<<0)
#define BUTTON_R (1<<1)
#define BUTTON_U (1<<2)
#define BUTTON_D (1<<3)
#define BUTTON_A (1<<4)
#define BUTTON_B (1<<5)
#define BUTTON_X (1<<6)
#define BUTTON_Y (1<<7)

#define CHANNEL_LEFTJOYSTICK_X 0
#define CHANNEL_LEFTJOYSTICK_Y 1
#define CHANNEL_RIGHTJOYSTICK_X 2
#define CHANNEL_RIGHTJOYSTICK_Y 3

#define CHANNEL_GRAVITY_X 4
#define CHANNEL_GRAVITY_Y 5
#define CHANNEL_GRAVITY_Z 6

class OseppRemote
{
  private:
    char letter;
    int number;
    char minusSign;
    char tempButton = 0;
    char mButtonState;
    int mChannel[7];

    unsigned long mLast_Data_millis;

  public:
    OseppRemote();
    void feed(char speed);
    uint8_t getButtonState(uint8_t mask);
    int getChannalData(uint8_t channel);
    bool isTimeoutFor(unsigned long howlong);
};

#endif


