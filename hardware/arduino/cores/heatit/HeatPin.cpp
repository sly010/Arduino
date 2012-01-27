#include <Arduino.h>

class HeatPin {

public:

  unsigned int mAmps_goal;
  uint8_t counter;
  uint8_t duty;
  uint8_t pin;

  void setGoal(unsigned int goal) {
    this->mAmps_goal = goal;
  }
  
  HeatPin(int pin, int counter) {
    this->pin = pin;
    this->counter = counter;
    pinMode(pin,OUTPUT);
    digitalWrite(pin,LOW);
  }

  inline int measure() {
    return 0;
  }
  
  void adjustDuty() {
    //duty++;
    /*
    unsigned int mAmpsGoal_measured = measure();
    
    if (mAmps_goal >= mAmpsGoal_measured && duty < 0xFF) {
      duty++;
    } else if (duty > 0x00) {
      duty--;
    }
    */
  }
  
  void update() {
    
    counter++;
    
    if (counter == 0) {
      adjustDuty();
      digitalWrite(pin, LOW);
    } else if (counter == (255-duty)) {
      digitalWrite(pin, HIGH);
    } 
    
  }

};

HeatPin * heatPins[8];

SIGNAL(TIMER1_COMPA_vect) {
  for (unsigned char i = 0; i < 8; i++) {
    heatPins[i]->update();
  }
}

void init_heatit_timer() {
  
  for (unsigned char i = 0; i < 8; i++) {
    heatPins[i] = new HeatPin(10+i, 0);
    heatPins[i]->duty = 1 + 32*i;
  }
  
  // Timer1
  // No Prescaler
  // ~ 64 * 256 Hz
  TCCR1A = B00000000;
  TCCR1B = B00001001;
  TCCR1C = B00000000;
  OCR1AH = 3;
  OCR1AL = 208;

  // Interrupt  
  TIMSK1 = B00000010;  
  sei();

}

