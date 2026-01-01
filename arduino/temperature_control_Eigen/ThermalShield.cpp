
#include "ThermalShield.h"

#if defined(__AVR__)
#include "src\TimerOne\TimerOne.h"
#endif

#if defined(__arm__) || defined(__ARM__)
#include "src\STM32TimerInterrupt.h"
STM32Timer Timer1_(TIM2);
#endif


#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))


static void timer_callback();

void ThermalClass::begin(unsigned int Ts) {
  this->Ts = Ts;

  pinMode(HEATER_PIN, OUTPUT);
  pinMode(A2, INPUT);
  analogWrite(HEATER_PIN, 0);
  sensor.begin();
  sensor.setResolution(11);
  sensorRead();
  display.setBrightness(7);

#if defined(__AVR__)
  Timer1.initialize(1e6);
  Timer1.attachInterrupt(timer_callback);
#endif

#if defined(__arm__) || defined(__ARM__)
  Timer1_.attachInterruptInterval(1e6, timer_callback);
#endif
}

float ThermalClass::sensorRead() {
  sensor.requestTemperaturesByIndex(0);
  return sensor.getTempCByIndex(0);
}

float ThermalClass::sensorReadSampled() {
  return y_sampler.read();
}

float ThermalClass::referenceRead(void) {
  return mapFloat(analogRead(A2), 0.0, 1024.0, 0.0, 100.0);
}



void ThermalClass::actuatorWrite(float percentValue) {

  percentValue = MAX(percentValue, 0);
  percentValue = MIN(percentValue, 100);


  bool error = (temp == DEVICE_DISCONNECTED_C);
  bool overheat = (temp > OVERHEAT_TH);

  if (overheat || error) {
    analogWrite(HEATER_PIN, percToPwm(0));
    u = 0;
  } else {
    analogWrite(HEATER_PIN, percToPwm(percentValue));
    u = percentValue;
  }
}

float ThermalClass::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

byte ThermalClass::percToPwm(float perc) {
  return byte(mapFloat(perc, 0.0, 100, 0.0, 255.0));
}

void ThermalClass::displayPool() {
  if (second_tick % 3 == 0) {
    display.showNumberHexEx(0x0A, 0, false, 1, 0);
    display.showNumberDecEx((int)(temp), 0, false, 3, 1);
  }
  else if (second_tick % 3 == 1)
  {
    display.showNumberHexEx(0x0B, 0, false, 1, 0);
    display.showNumberDecEx((int)(ref), 0, false, 3, 1);  
  }
  else
  {
    display.showNumberHexEx(0x0C, 0, false, 1, 0);
    display.showNumberDecEx((int)(u), 0, false, 3, 1);
  }
}

class ThermalClass_ : public ThermalClass {

public:

  void timer_tick(void) {

    temp = sensorRead();
    bool error = (temp == DEVICE_DISCONNECTED_C);
    bool overheat = temp > OVERHEAT_TH;

    if (overheat || error) {
      actuatorWrite(0);
    }

    ref=referenceRead();

    if (second_tick % getTs() == 0) {
      y_sampler.sample(temp);
    if (sample_callback!=NULL);
        sample_callback();
      tick++;
    }

    if (second_callback!=NULL);
        second_callback();

    second_tick++;
  }
};


ThermalClass_ ThermalShield_;
ThermalClass& ThermalShield = ThermalShield_;

static void timer_callback() {
  ThermalShield_.timer_tick();
}
