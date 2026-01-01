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
class RobotKinematics {
public:

  RobotKinematics(const RotationMatrix* rotations[DOF],
                  const Matrix<3, 1> (&translations)[DOF]) {
    for (std::size_t i = 0; i < DOF; i++) {
      rotations_[i] = rotations[i];
      translations_[i] = translations[i];
    }
  }

  virtual ~RobotKinematics() = default;


  const RotationMatrix* getRotationMatrix(std::size_t jointIndex) const {
    return rotations_[jointIndex];
  }

  const Matrix<3, 1>& getTranslation(std::size_t linkIndex) const {
    return translations_[linkIndex];
  }


  Matrix<3, DOF> getJacobian(const Matrix<DOF, 1>& theta, const Matrix<3, 1>& pn) const {

    Matrix<3, DOF> J;
    for (int i = 0; i < DOF; i++) {
      //TODO J.col(i) 
    }

    return J;
  }

  bool SolveIK(
    const Matrix<3, 1>& pr,
    Matrix<DOF, 1>& theta_,
    float tol,
    float lambda = 0.001f,
    int max_iter=10) const {
    Matrix<3, 1> zero_vec = Matrix<3, 1>::Zero();
    Matrix<DOF, 1> theta=theta_;

    
    for (int iter=0;iter<max_iter;iter++)  {
      // Position of end-effector in base frame

      Matrix<3, 1> p0 = forwardKinematics(theta, zero_vec, DOF - 1);

      // Position error
      Matrix<3, 1> r;
      // TODO Matrix<3, 1> r

      // Compute Jacobians for position and orientation
      Matrix<3, DOF> J = getJacobian(theta, zero_vec);

      // TODO Matrix<DOF, DOF> H = 

      // Solve for update step
      // TODO Matrix<DOF, 1> delta_theta
      // TODO theta +=

      // Convergence check
      if ((r.transpose() * r)(0, 0) < tol) {
         theta_=theta;
        return 1;
      }
    }

    return 0;
  }

 bool SolveIK(
    const Matrix<3, 1>& pr,
    const Matrix<3, 3>& Rr,
    Matrix<DOF, 1>& theta_,
    float tol,
    float lambda = 0.001f,
    int max_iter=10) const {
    Matrix<3, 1> zero_vec = Matrix<3, 1>::Zero();
    Matrix<DOF, 1> theta=theta_;

    for (int iter=0;iter<max_iter;iter++) 
    {
      // Position of end-effector in base frame

       Matrix<3, 1> p0 = forwardKinematics(theta, zero_vec, DOF - 1);

      // Position error
      Matrix<3, 1> rp;
      //TODO rp=

      // Orientation error using difference of rotation matrices
      Matrix<3, 3> R_tot = getRtot(theta);
      //TODO Matrix<3, 3> Re=
      Matrix<3, 1> ro;
      //ro<<
    

      // Compute Jacobians for position and orientation
      Matrix<3, DOF> Jp = getJacobian(theta, zero_vec);
      Matrix<3, DOF> Jo;

      for (std::size_t i = 0; i < DOF; i++) {
        //TODO Matrix<3, 3> dR_totRr=
        //TODO Jo.col(i) << 
      }

      // Combined Jacobian (6 x DOF)
      Matrix<6, DOF> J;
      //TODO J.template block<3, DOF>(0, 0) =
      //TODO J.template block<3, DOF>(3, 0) = 
  
      // Error vector (6 x 1)
      Matrix<6, 1> r;
     //TODO r.template block<3, 1>(0, 0) = ;
     //TODO r.template block<3, 1>(3, 0) = ;
     

     // TODO Matrix<DOF, DOF> H =

     // Solve for update step
     // TODO Matrix<DOF, 1> delta_theta =
     // TODO theta += ;

      // Convergence check
      if ((rp.transpose() * rp + ro.transpose() * ro)(0, 0) < tol) {
        theta_=theta;
        return 1;
      }
    }

    return 0;
  }


protected:

  Matrix<3, 1> forwardKinematics(
    const Matrix<DOF, 1>& theta,
    const Matrix<3, 1>& pi,
    int i) const {
    Matrix<3, 1> p0 = pi;
    for (int j = i; j >= 0; j--) {
     // TODO p0 = 
    }
    return p0;
  }


  Matrix<3, 3> getRtot(const Matrix<DOF, 1>& theta) const {
    Matrix<3, 3> R_tot = Matrix<3, 3>::Identity();
    for (std::size_t i = 0; i < DOF; i++) {
     // TODO R_tot 
    }
    return R_tot;
  }

  Matrix<3, 3> getdRtot(const Matrix<DOF, 1>& theta, std::size_t j) const {
    Matrix<3, 3> dR_tot = Matrix<3, 3>::Identity();
    for (std::size_t i = 0; i < DOF; i++) {
      if (i == j) {
       //TODO dR_tot=
      } else {
       //TODO dR_tot=
      }
    }
    return dR_tot;
  }

  const RotationMatrix* rotations_[DOF];
  Matrix<3, 1> translations_[DOF];
};


class JointActuators {
public:
  JointActuators(){}

  virtual bool writeAngles(const float*, std::size_t) = 0;
  virtual bool readAngles(float*, std::size_t)=0;
};

template<std::size_t DOF>
class RobotArm : public RobotKinematics<DOF> {
public:

  RobotArm(const RotationMatrix* rotations[DOF], const Matrix<3, 1> (&translations)[DOF], JointActuators* actuators)
    : RobotKinematics<DOF>(rotations, translations), jointAngles_(Matrix<DOF, 1>::Zero()), actuators_(actuators) {
  }

  ~RobotArm() = default;


  bool setJointAngles(const Matrix<DOF, 1>& angles) {
    
    if (!actuators_->writeAngles(angles.data(),DOF)) return false;
    if (!actuators_->readAngles(jointAngles_.data(),DOF)) return false;
    jointAngles_=angles;
    return true;
  }

  float getJointAngle(std::size_t index) const {
    return jointAngles_(index);
  }

  Matrix<DOF, 1> getJointAngles() const {
    return jointAngles_;
  }

  Matrix<3, 3> getCurrentOrientation() const {
    return RobotKinematics<DOF>::getRtot(jointAngles_);
  }

  Matrix<3, 1> getCurrentEffectorPosition() const {
    return RobotKinematics<DOF>::forwardKinematics(jointAngles_, Matrix<3, 1>::Zero(), DOF - 1);
  }

  bool goTo(const positionOrientation& po) {
    Matrix<3, 1> pr;
    Matrix<3, 3> Rr;
    pr=po.positionVector();
    Rr = po.toRotationMatrix();
    return goTo(pr, Rr);
  }


  bool goTo(const Matrix<3, 1>& pr, const Matrix<3, 3>& Rr) {
    Matrix<DOF, 1> theta=jointAngles_;
    bool res=RobotKinematics<DOF>::SolveIK(pr, Rr, theta, 1e-4f);
    if(res)
    {
      res=setJointAngles(theta);
    }
    return res;
  }

  
  bool goTo(const Matrix<3, 1>& pr) {
    Matrix<DOF, 1> theta=jointAngles_;
    bool res=RobotKinematics<DOF>::SolveIK(pr, theta, 1e-4f);
    if(res)
    {
      res=setJointAngles(theta);
    }
    return res;
  }

protected:
  JointActuators* actuators_;
  Matrix<DOF, 1> jointAngles_;
};

}
