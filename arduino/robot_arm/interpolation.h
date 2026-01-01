
#pragma once


#include "geometry.h"


/**
 * @brief Type alias for fixed-size Eigen matrix of floats.
 */
template<int Rows, int Cols>
using Matrix = Eigen::Matrix<float, Rows, Cols>;


class PoseInterpolator {
public:
    // Default constructor
    PoseInterpolator() 
        : steps_(0), currentStep_(0), start_(), end_(), q_start_(Eigen::Quaternionf::Identity()), q_end_(Eigen::Quaternionf::Identity()) {}

    // Explicit initialization method (can be called repeatedly)
    void init(const positionOrientation& start,
              const positionOrientation& end,
              float linearVelocity, // [m/s]
              float sampleTime)     // [s]
    {
        start_ = start;
        end_   = end;
        currentStep_ = 0;

        // Compute distance
        float distance = (end_.positionVector() - start_.positionVector()).norm();

        // Compute required steps (at least 2: start & end)
        steps_ = static_cast<int>(std::ceil(distance / (linearVelocity * sampleTime)));
        if (steps_ < 2) steps_ = 2;

        // Precompute quaternion start/end
        q_start_ = start_.toQuat();
        q_end_   = end_.toQuat();
    }

    void reset()
    {
        steps_=0;
        currentStep_=0;
        start_=positionOrientation();
        end_=positionOrientation();
        q_start_=Eigen::Quaternionf::Identity();
        q_end_=Eigen::Quaternionf::Identity();
    }

    bool hasNext() const {
        return currentStep_ < steps_;
    }

    positionOrientation next() {
        if (!hasNext()) {
            return end_;
        }

        float t = static_cast<float>(currentStep_) / (steps_ - 1);

        // Linear interpolation for position
        Matrix<3,1> pos_interp = (1.0f - t) * start_.positionVector() + t * end_.positionVector();

        // SLERP for orientation
        Eigen::Quaternionf q_interp = q_start_.slerp(t, q_end_);

        currentStep_++;
        return positionOrientation(pos_interp, q_interp.toRotationMatrix());
    }

private:
    positionOrientation start_;
    positionOrientation end_;
    int steps_;
    int currentStep_;
    Eigen::Quaternionf q_start_;
    Eigen::Quaternionf q_end_;
};


static inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

std::vector<positionOrientation> generateSphericaTrajectory(
    const Eigen::Vector3f& center,       // object in world
    float radius,                         // standoff distance
    float rMin, float rMax, int nR,       // roll interval & samples
    float pMin, float pMax, int nP,       // pitch interval & samples
    float yawFixed,   
    const Eigen::Vector3f& sensorForward )                   
{
    std::vector<positionOrientation> traj;
    traj.reserve(std::max(1, nR) * std::max(1, nP));

    const int dR = std::max(1, nR - 1);
    const int dP = std::max(1, nP - 1);

    for (int i = 0; i < std::max(1, nR); ++i) {
        float ti = (float)i / (float)dR;
        float roll = lerp(rMin, rMax, ti);

        bool forward = (i % 2 == 0); // serpentine for smooth transitions
        for (int j = 0; j < std::max(1, nP); ++j) {
            int jj = forward ? j : (nP - 1 - j);
            float tj = (float)jj / (float)dP;
            float pitch = lerp(pMin, pMax, tj);

            // Orientation (RPY, yaw fixed)
            Eigen::Matrix3f R = RPY(roll, pitch, yawFixed);

            // Sensor look vector in world
            Eigen::Vector3f look = R * sensorForward;  // unit by construction if sensorForward is unit

            // Position so that center lies exactly on the sensor forward axis at distance = radius
            Eigen::Vector3f pos = center - radius * look;

            traj.push_back(positionOrientation(pos.x(), pos.y(), pos.z(), roll, pitch, yawFixed));
        }
    }
    return traj;
}
