

#if defined(__AVR__)

#include "TimerOne.h"
TimerOne Timer1;              // preinstantiate

ISR(TIMER1_OVF_vect)
{
  Timer1.isrCallback();
}
unsigned short TimerOne::pwmPeriod = 0;
unsigned char TimerOne::clockSelectBits = 0;
void (*TimerOne::isrCallback)() = TimerOne::isrDefaultUnused;
void TimerOne::isrDefaultUnused() { /* noop */; }

#endif

