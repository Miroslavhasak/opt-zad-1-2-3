#ifndef FLOATSHIELD_H
#define FLOATSHIELD_H

#include "src\VL53L0X\VL53L0X.h" 
#include "src\MCP4725\MCP4725.h" 

// ===========================================================================
// Generic Sampler<T> class
// Stores the latest sampled value of type T with flags for availability
// and missed updates. Useful for decoupling sensor sampling from usage.
// ===========================================================================
template<typename T>
class Sampler {
public:
    // Store a new value and mark it as available
    void sample(const T& value_) {
        value = value_;
        missed = available;
        available = true;
    }

    // Check if a new sample is available
    bool isAvailable() const { return available; }

    // Check if a previous sample was overwritten before being read
    bool isMissed() const { return missed; }

    // Read and consume the last sample
    T read() {
        available = false;
        missed = false;
        return value;
    }

    // Read into reference and consume the last sample
    void read(T& value_) {
        available = false;
        missed = false;
        value_ = value;
    }

    // Return the last sampled value without consuming it
    T getLastValue() const { return value; }

private:
    volatile T value{};              // Latest stored value
    volatile bool available = false; // True if a new sample is available
    volatile bool missed = false;    // True if a sample was overwritten
};


// ===========================================================================
// FloatClass - Controls the FloatShield hardware
// Provides actuator control, reference input reading, and distance/altitude
// measurement with calibration support. Designed for a levitation experiment
// with a ball in a vertical tube and a fan as actuator.
// ===========================================================================
class FloatClass {
public:

    FloatClass(): dac(0x60){}
    // Initialize hardware (sensor, timers, etc.)
    bool begin(float Ts);

    // Run calibration routine to determine min/max ball positions
    void calibrate(void);

    // Write fan power in percent (0.0–100.0)
    void actuatorWrite(float);

    // Read potentiometer as percent (0.0–100.0)
    float referenceRead(void);

    // Read potentiometer mapped to calibrated altitude range
    float referenceReadAltitude(void);

    // Read current ball position as percent of tube height
    float sensorRead(void);

    // Read current ball altitude in millimetres
    float sensorReadAltitude(void);

    // Read last sampled altitude (from timer-based sampling)
    float sensorReadAltitudeSampled(void);

    // Raw distance measurement (mm) from sensor
    float sensorReadDistance(void);

    // Measure actual voltage applied to actuator
    float actuatorReadVoltage(void);

    // --- Calibration status and parameters ---
    inline bool returnCalibrated(void) { return _wasCalibrated; }
    inline float returnMinDistance(void) { return _minDistance; }
    inline float returnMaxDistance(void) { return _maxDistance; }
    inline float returnRange(void) { return _range; }

    // --- Timing and sampling helpers ---
    inline unsigned long getTick() { return tick; }
    inline bool isSampleAvailable() { return y_sampler.isAvailable(); }

    void (*sample_callback)()=NULL;

protected:
    Sampler<float> y_sampler;   // Stores sampled altitude values
    unsigned long tick = 0;     // Incremented each sampling period

private:
    // Write raw 12-bit value to DAC (MCP4725)
    void dacWrite(uint16_t DAClevel);

    // Map float from one range to another
    inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    // Constrain float to a given range
    inline float constrainFloat(float x, float min_x, float max_x) {
        if (x >= max_x) x = max_x;
        if (x <= min_x) x = min_x;
        return x;
    }

    // Distance sensor (VL53L0X Time-of-Flight laser)
    VL53L0X distanceSensor;
    MCP4725 dac;

    // Calibration data
    float _minDistance = 17.0f;   // Default min (mm)
    float _maxDistance = 271.0f;  // Default max (mm)
    float _range = 271.0f - 17.0f;
    bool _wasCalibrated = false;

    float Ts; // Sampling time (s)
};

// Externally accessible instance
extern FloatClass& FloatShield;

#endif