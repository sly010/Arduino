#include <Arduino.h>
#include <HeatIt.h>

// ====================================================================

HeatPin::HeatPin(int pin, int counter) {
  this->pin = pin;
  this->counter = counter;
  pinMode(pin,OUTPUT);
  digitalWrite(pin,LOW);
}

// ====================================================================

void HeatPin::update() {
  this->counter++;
  if (this->counter == 0) {
    this->adjustDuty();
    digitalWrite(this->pin, LOW);
  } 
  else if (counter == (255 - this->duty)) {
    digitalWrite(this->pin, HIGH);
  }
}

// ====================================================================

void HeatPin::adjustDuty() {
  // nothing for now
}

// ====================================================================

void HeatPin::setDuty(int duty) {
  if (duty >= 0 && duty <= 255) {
    this->duty = duty;
  }
}

// ====================================================================

HeatItMain::HeatItMain() {

  for (unsigned char i = 0; i < 8; i++) {
    heatPins[i] = new HeatPin(10+i, 0);
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

// ====================================================================

void HeatItMain::update() {
  for (unsigned char i = 0; i < 8; i++) {
    heatPins[i]->update();
  }
}

void HeatItMain::led(int state) {
  if (state == LOW) {
    PORTB &= ~(1<<0);
  } else {
    PORTB |= (1<<0);
  }
}

void HeatItMain::set(int pin, int duty) {
  if (pin >= 0 && pin <= 8) {
    heatPins[pin]->setDuty(duty);
  }
}

// ====================================================================

SIGNAL(TIMER1_COMPA_vect) {
  HeatIt.update();
}

HeatItMain HeatIt;

