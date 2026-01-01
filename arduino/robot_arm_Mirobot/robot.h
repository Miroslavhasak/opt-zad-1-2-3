#include <cstddef>

#pragma once

#include "geometry.h"

template<int Rows, int Cols>
using Matrix = Eigen::Matrix<float, Rows, Cols>;

typedef struct
{
  float minAngle;
  float maxAngle;
  float gain;
  float offset;
} actuator_parameters;


namespace Robot {


template<std::size_t DOF>
class RobotKinematicsBase {
public:

  virtual bool SolveIK(
    const Matrix<3, 1>& pr,
    const Matrix<3, 3>& Rr,
    Matrix<DOF, 1>& theta_,
    float tol_pos,
    float tol_ori,
    float lambda,
    int max_iter)  = 0;


  virtual bool SolveIK(
    const Matrix<3, 1>& pr,
    Matrix<DOF, 1>& theta_,
    float tol,
    float lambda,
    int max_iter)  = 0;

  virtual std::pair<Matrix<3, 1>, Matrix<3, 3>> forwardKinematics(const Matrix<DOF, 1>& theta) const = 0;
};


template<std::size_t DOF>
class RobotKinematicsRotPtrs : public RobotKinematicsBase<DOF> {
public:

  RobotKinematicsRotPtrs(const RotationMatrix* rotations[DOF],
                         const Matrix<3, 1> (&translations)[DOF]) {
    for (std::size_t i = 0; i < DOF; i++) {
      rotations_[i] = rotations[i];
      translations_[i] = translations[i];
    }
  }

  virtual ~RobotKinematicsRotPtrs() = default;


  const RotationMatrix* getRotationMatrix(std::size_t jointIndex) const {
    return rotations_[jointIndex];
  }

  const Matrix<3, 1>& getTranslation(std::size_t linkIndex) const {
    return translations_[linkIndex];
  }


  bool SolveIK(
    const Matrix<3, 1>& pr,
    Matrix<DOF, 1>& theta_,
    float tol,
    float lambda,
    int max_iter) {
    Matrix<3, 1> zero_vec = Matrix<3, 1>::Zero();
    Matrix<DOF, 1> theta = theta_;


    for (int iter = 0; iter < max_iter; iter++) {
      // Position of end-effector in base frame

      auto [p0, R] = forwardKinematics(theta);

      // Position error
      Matrix<3, 1> r;
      // r  = TODO

      // Compute Jacobians for position and orientation
      Matrix<3, DOF> J = getPositionJacobian(theta, zero_vec);

      // Matrix<DOF, DOF> H = TODO

      // Solve for update step
      // Matrix<DOF, 1> delta_theta = TODO
      //theta = TODO;

      // Convergence check
      if ((r.transpose() * r)(0, 0) < tol) {
        theta_ = theta;
        return 1;
      }
    }

    return 0;
  }


  bool SolveIK(
    const Matrix<3, 1>& pr,
    const Matrix<3, 3>& Rr,
    Matrix<DOF, 1>& theta_,
    float tol_pos,
    float tol_ori,
    float lambda,
    int max_iter) {
    Matrix<3, 1> zero_vec = Matrix<3, 1>::Zero();
    Matrix<DOF, 1> theta = theta_;

    for (int iter = 0; iter < max_iter; iter++) {
      // Position of end-effector in base frame

      auto [p0, R] = forwardKinematics(theta);

      // Position error
      Matrix<3, 1> rp;
      //rp = TODO;

      // Orientation error using difference of rotation matrices
      // Matrix<3, 3> Re = TODO;
      Matrix<3, 1> ro;
      // ro << TODO;


      // Compute Jacobians for position and orientation
      Matrix<3, DOF> Jp = getPositionJacobian(theta, zero_vec);
      Matrix<3, DOF> Jo = getOrientationErrorJacobian(theta, Rr);


      // Combined Jacobian (6 x DOF)
      Matrix<6, DOF> J;
      // J.template block<3, DOF>(0, 0) = TODO;
      // J.template block<3, DOF>(3, 0) = TODO;

      // Error vector (6 x 1)
      Matrix<6, 1> r;
      // r.template block<3, 1>(0, 0) = TODO;
      // r.template block<3, 1>(3, 0) = TODO;


      // Matrix<DOF, DOF> H = TODO

      // Solve for update step
      //Matrix<DOF, 1> delta_theta = TODO
      // theta = TODO;

      // Convergence check
      if ((rp.transpose() * rp)(0, 0) < tol_pos && (ro.transpose() * ro)(0, 0) < tol_ori) {
        theta_ = theta;
        return 1;
      }
    }

    return 0;
  }

protected:

  std::pair<Matrix<3, 1>, Matrix<3, 3>> forwardKinematics(const Matrix<DOF, 1>& theta) const {
    Matrix<3, 1> p = Matrix<3, 1>::Zero();
    Matrix<3, 3> R = Matrix<3, 3>::Identity();
    for (int j = DOF - 1; j >= 0; j--) {
      //TODO
    }
    return { p, R };
  }


  Matrix<3, 3> getRtot(const Matrix<DOF, 1>& theta) const {
    Matrix<3, 3> R_tot = Matrix<3, 3>::Identity();
    for (std::size_t i = 0; i < DOF; i++) {
      //TODO
    }
    return R_tot;
  }

  Matrix<3, 3> getdRtot(const Matrix<DOF, 1>& theta, std::size_t j) const {
    Matrix<3, 3> dR_tot = Matrix<3, 3>::Identity();
    for (std::size_t i = 0; i < DOF; i++) {
      //TODO
    }
    return dR_tot;
  }


  Matrix<3, DOF> getPositionJacobian(const Matrix<DOF, 1>& theta, const Matrix<3, 1>& pn) const {

    Matrix<3, DOF> J;
    for (int i = 0; i < DOF; i++) {
      Matrix<3, 1> p;
      //TODO
      J.col(i) = p;
    }

    return J;
  }

  Matrix<3, DOF> getOrientationErrorJacobian(
    const Matrix<DOF, 1>& theta,
    const Matrix<3, 3>& Rr) const {
    Matrix<3, DOF> Jo;

    for (std::size_t i = 0; i < DOF; i++) {
      // TODO Jo.col(i) << 
    }

    return Jo;
  }

  const RotationMatrix* rotations_[DOF];
  Matrix<3, 1> translations_[DOF];
};


class JointActuators {
public:
  JointActuators() {}

  virtual bool writeAngles(const float*, std::size_t) = 0;
  virtual bool readAngles(float*, std::size_t) = 0;
};

template<std::size_t DOF>
class RobotArm {
public:

  RobotArm(RobotKinematicsBase<DOF>* robot_kinematics, JointActuators* actuators)
    : robot_kinematics_(robot_kinematics), jointAngles_(Matrix<DOF, 1>::Zero()), actuators_(actuators) {
  }

  ~RobotArm() = default;


  bool setJointAngles(const Matrix<DOF, 1>& angles) {

    if (!actuators_->writeAngles(angles.data(), DOF)) return false;
    if (!actuators_->readAngles(jointAngles_.data(), DOF)) return false;
    return true;
  }

  float getJointAngle(std::size_t index) const {
    return jointAngles_(index);
  }


  Matrix<DOF, 1> getJointAngles() const {
    return jointAngles_;
  }

  std::pair<Matrix<3, 1>, Matrix<3, 3>> getCurrentEffectorPositionOrientation() const {
    return robot_kinematics_->forwardKinematics(jointAngles_);
  }

  bool goTo(const positionOrientation& po) {
    Matrix<3, 1> pr;
    Matrix<3, 3> Rr;
    pr = po.positionVector();
    Rr = po.toRotationMatrix();
    return goTo(pr, Rr);
  }


  bool goTo(const Matrix<3, 1>& pr, const Matrix<3, 3>& Rr) {
    Matrix<DOF, 1> theta = jointAngles_;
    bool res = robot_kinematics_->SolveIK(pr, Rr, theta, 1e-5f, 1e-3f, 1e-2, 10);
    
    if (res) {
      res = setJointAngles(theta);
    }
    return res;
  }


  bool goTo(const Matrix<3, 1>& pr) {
    Matrix<DOF, 1> theta = jointAngles_;
    bool res = robot_kinematics_->SolveIK(pr, theta, 1e-5f, 1e-3f, 10);
    if (res) {
      res = setJointAngles(theta);
    }
    return res;
  }

protected:
  JointActuators* actuators_;
  Matrix<DOF, 1> jointAngles_;
  RobotKinematicsBase<DOF>* robot_kinematics_;
};

}
