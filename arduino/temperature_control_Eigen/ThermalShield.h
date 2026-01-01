
#ifndef ThermalSHIELD_H			      
#define ThermalSHIELD_H	
            
#include <Arduino.h>	
#include "src\DallasTemperature\DallasTemperature.h"
#include "src\TM1637Display\TM1637Display.h"



#define HEATER_PIN 5
#define SENSOR_PIN A0
#define OVERHEAT_TH 75

#define CLK_PIN 2
#define DIO_PIN 3


template<typename T>
class Sampler
{
public:
    void sample(const T& value_) {
        value = value_;
        missed = available;
        available = true;
    }

    bool isAvailable() const {
        return available;
    }

    bool isMissed() const {
        return missed;
    }

    T read() {
        available = false;
        missed = false;
        return value;
    }
    void read(T& value_) {
        available = false;
        missed = false;
        value_=value;
    }

    T getLastValue() const {
        return value;
    }

private:
    volatile T value{};               // Default-initialize value
    volatile bool available = false;
    volatile bool missed = false;
};

class ThermalClass{		    	                         
 
 public:
  ThermalClass():oneWire(SENSOR_PIN), sensor(&oneWire), display(CLK_PIN,DIO_PIN){};
  void begin(unsigned int);                                              
  void actuatorWrite(float); 
  void displayPool();                            
  float sensorRead(void); 
  float sensorReadSampled(void);  
  float referenceRead(void);
  bool isSampleAvailable(){return y_sampler.isAvailable();}               
  unsigned long const getTick(){return tick;}
  unsigned int const getTs(){return Ts;} 
  float getLastTemp(){return temp;}
  float getLastU(){return u;}
  float getLastRef(){return ref;}

  void (*sample_callback)()=NULL;
  void (*second_callback)()=NULL;
 
protected:

  Sampler<float> y_sampler;
  unsigned long tick=0;
  unsigned long second_tick=0;
  TM1637Display display;
  float temp=0;
  float u=0;
  float ref=0;
   
 private: 
                         
   float mapFloat(float , float , float , float , float );
   byte percToPwm(float);

  OneWire oneWire;
  DallasTemperature sensor;
  
  unsigned int Ts; 
  
  
};

extern ThermalClass& ThermalShield; 



#endif
