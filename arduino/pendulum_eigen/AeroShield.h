
#ifndef AEROSHIELD_H            
#define AEROSHIELD_H  
            
#include <Arduino.h>      
#include "src\AS5600\AS5600.h"

#define AERO_RPIN A3             
#define AERO_UPIN 5   


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

class AeroClass{                                   
 
 public:
  bool begin(float);                                              
  void actuatorWrite(float);             
                   
  float referenceRead(void);                       
  float sensorReadDegree(void); 
  float sensorReadDegreeSampled(void);    
  bool isSampleAvailable(){return y_sampler.isAvailable();}               
  void calibrate(void);
  unsigned long getTick(){return tick;}
  void (*sample_callback)()=NULL;
 
protected:

  Sampler<float> y_sampler;
  unsigned long tick=0;
   
 private: 
                         
   float mapFloat(float , float , float , float , float );
   byte percToPwm(float);

   uint16_t _zero;
   
  float Ts=0.05; 
   AS5600 as5600;
   
};

extern AeroClass& AeroShield; 



#endif
