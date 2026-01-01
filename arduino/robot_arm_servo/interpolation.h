
#pragma once


#include "geometry.h"


/**
 * @brief Type alias for fixed-size Eigen matrix of floats.
 */
template<int Rows, int Cols>
using Matrix = Eigen::Matrix<float, Rows, Cols>;

class PoseInterpolator {
public:
    PoseInterpolator() 
        : steps_(0), currentStep_(0), start_(), end_(),
          q_start_(Eigen::Quaternionf::Identity()), q_end_(Eigen::Quaternionf::Identity()) {}

    void init(const positionOrientation& start,
              const positionOrientation& end,
              float velocity,  
              float sampleTime)      
    {
        start_ = start;
        end_   = end;
        currentStep_ = 0;

        q_start_ = start_.toQuat();
        q_end_   = end_.toQuat();

        // Compute distance between positions
        float distance = (end_.positionVector() - start_.positionVector()).norm();
        
        if (distance > 1e-3) {
            steps_ = static_cast<int>(std::ceil(distance / (velocity * sampleTime)));
        } else {
             float ang_distance = (end_.anglesVector() - start_.anglesVector()).norm();
            steps_ = static_cast<int>(std::ceil(ang_distance / (10.0*velocity * sampleTime)));
        }

        if (steps_ < 2) steps_ = 2;  // at least start & end
    }

    void reset()
    {
        steps_ = 0;
        currentStep_ = 0;
        start_ = positionOrientation();
        end_   = positionOrientation();
        q_start_ = Eigen::Quaternionf::Identity();
        q_end_   = Eigen::Quaternionf::Identity();
    }

    bool hasNext() const {
        return currentStep_ < steps_;
    }

    positionOrientation next() {
        if (!hasNext()) {
            return end_;
        }

        float t = static_cast<float>(currentStep_) / (steps_ - 1);

        Matrix<3,1> pos_interp =
            (1.0f - t) * start_.positionVector() + t * end_.positionVector();

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

