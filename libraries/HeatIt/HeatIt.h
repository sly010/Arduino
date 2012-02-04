#ifndef __HEAT_IT_H__
#define __HEAT_IT_H__

#include <Arduino.h>

class HeatPin {

private:

  unsigned int mAmps_goal;
  uint8_t counter;
  uint8_t duty;
  uint8_t pin;
  
  void adjustDuty();
  
public:

    HeatPin(int pin, int counter);
    void update();
    void setDuty(int duty);

};

// ====================================================================


class HeatItMain {

private:

  HeatPin * heatPins[8];

public:

  HeatItMain();
  void update();

  void set(int pin, int state);
  void led(int state);

};

extern HeatItMain HeatIt;


#endif//__HEAT_IT_H__
