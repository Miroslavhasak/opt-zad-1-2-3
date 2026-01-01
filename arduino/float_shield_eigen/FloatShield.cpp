#include "FloatShield.h"

#define FLOAT_RPIN A0        // Potentiometer pin (reference input)
#define FLOAT_AVPIN A1       // Actuator voltage feedback pin


#if defined(__AVR__)
#include "src\TimerOne\TimerOne.h"
#endif

#if defined(__arm__) || defined(__ARM__)
#include "src\STM32TimerInterrupt.h"
STM32Timer Timer1_(TIM1);
#endif


static void timer_callback();


// ===========================================================================
// Begin - Initialize distance sensor, timers, and set sampling period
// ===========================================================================
bool FloatClass::begin(float Ts) {
    this->Ts = Ts;
    bool res = true;

    // Initialize distance sensor
    res = distanceSensor.init();
    if (!res) return res;

    distanceSensor.setTimeout(1000);                 // Timeout = 1 s
    //distanceSensor.setMeasurementTimingBudget(20000); // High-speed mode
    distanceSensor.setMeasurementTimingBudget((uint32_t)(0.8*Ts*1000.0f));
    delay(200);
    distanceSensor.startContinuous((uint32_t)(Ts*1000.0f));                // Continuous mode

    delay(200);
    res = dac.begin();
    if (!res) return res;

    actuatorWrite(0.0f);

     delay(200);
    //calibrate(); 

    // Configure timer interrupt for sampling (if Ts > 0)
    if (Ts > 0.0) {
    #if defined(__AVR__)
        Timer1.initialize(Ts * 1e6);
        Timer1.attachInterrupt(timer_callback);
    #endif

    #if defined(__arm__) || defined(__ARM__)
        Timer1_.attachInterruptInterval(Ts * 1e6, timer_callback);
    #endif
    }
    
    return res;
}


// ===========================================================================
// Calibrate - Determine min/max ball positions using the fan
// ===========================================================================
void FloatClass::calibrate() {
    // Helper for averaging sensor readings
    auto averageReading = [&](int samples, int delayMs) {
        float sum = 0.0f;
        for (int i = 0; i < samples; i++) {
            sum += sensorReadDistance();
            delay(delayMs);
        }
        return sum / samples;
    };

    // --- Calibrate minimum distance (ball near top) ---
    actuatorWrite(100.0f); // Full fan power
    while (sensorReadDistance() > 100.0f) { delay(100); }
    delay(1000);
    _minDistance = averageReading(100, 25);

    // --- Calibrate maximum distance (ball near bottom) ---
    actuatorWrite(0.0f); // Fan off
    while (sensorReadDistance() < 240.0f) { delay(100); }
    delay(1000);
    _maxDistance = averageReading(100, 25);

    // --- Finalize calibration ---
    _range = _maxDistance - _minDistance;
    _wasCalibrated = true;
}


// ===========================================================================
// DAC Write - Send 12-bit value to MCP4725 DAC
// ===========================================================================
void FloatClass::dacWrite(uint16_t DAClevel) {
   dac.setValue(DAClevel);
}


// ===========================================================================
// Actuator voltage measurement (from feedback pin)
// ===========================================================================
float FloatClass::actuatorReadVoltage(void) {
    float rawADC = (float)analogRead(FLOAT_AVPIN);
    float voltage = 4.0 * mapFloat(rawADC, 0.0, 1023.0, 0.0, 3.3);
    return voltage;
}


// ===========================================================================
// Actuator write - Set fan speed (0–100%) via DAC
// ===========================================================================
void FloatClass::actuatorWrite(float aPercent) {
    float mappedValue = mapFloat(aPercent, 0.0, 100.0, 0.0, 4095.0);
    mappedValue = constrainFloat(mappedValue, 0.0, 4095.0);
    dacWrite((uint16_t)mappedValue);
}


// ===========================================================================
// Reference read (potentiometer) - as percentage
// ===========================================================================
float FloatClass::referenceRead(void) {
    float raw = (float)analogRead(FLOAT_RPIN);
    return mapFloat(raw, 0.0, 1023.0, 0.0, 100.0);
}


// ===========================================================================
// Reference read mapped to altitude range
// ===========================================================================
float FloatClass::referenceReadAltitude(void) {
    float raw = (float)analogRead(FLOAT_RPIN);
    return mapFloat(raw, 0.0, 1023.0, 0.0, _range);
}


// ===========================================================================
// Sensor read - Ball altitude in percentage of tube height
// ===========================================================================
float FloatClass::sensorRead(void) {
    float dist = sensorReadDistance();
    float percent = mapFloat(dist, _maxDistance, _minDistance, 0.0, 100.0);
    return constrainFloat(percent, 0.0, 100.0);
}


// ===========================================================================
// Sensor read - Ball altitude in millimetres
// ===========================================================================
float FloatClass::sensorReadAltitude(void) {
    float alt = _maxDistance - sensorReadDistance();
    return constrain(alt, 0, _maxDistance);
}


// ===========================================================================
// Sensor read - Return last sampled altitude (from timer ISR)
// ===========================================================================
float FloatClass::sensorReadAltitudeSampled() {
    return y_sampler.read();
}


// ===========================================================================
// Sensor read - Raw distance in millimetres
// ===========================================================================
float FloatClass::sensorReadDistance(void) {
    return (float)distanceSensor.readRangeContinuousMillimeters();
}


// ===========================================================================
// Internal derived class for timer-based sampling
// ===========================================================================
class FloatClass_ : public FloatClass {
public:
    void timer_tick() {
        y_sampler.sample(sensorReadAltitude());
        if (sample_callback!=NULL)
        sample_callback();
        tick++;
    }
};

FloatClass_ FloatShield_;
FloatClass& FloatShield = FloatShield_;

// Timer callback ISR
static void timer_callback() {
    FloatShield_.timer_tick();
}