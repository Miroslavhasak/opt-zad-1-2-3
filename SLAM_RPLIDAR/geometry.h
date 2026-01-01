#pragma once

#include <Eigen/Dense>

/**
 * @brief Type alias for fixed-size Eigen matrix of doubles.
 */
template<int Rows, int Cols>
using Matrix = Eigen::Matrix<double, Rows, Cols>;

/**
 * @name Rotation matrix generation and derivatives
 * 
 * Each set of functions provides:
 * - Rotation matrix from sine and cosine values.
 * - Rotation matrix from an angle (radians).
 * - Derivative of rotation matrix w.r.t. angle.
 * 
 * Rotations follow the right-hand rule.
 */
///@{

/**
 * @brief Rotation matrix about the X-axis from sine and cosine of angle.
 */
Matrix<3,3> rotX(double s_theta, double c_theta)
{
    Matrix<3, 3> R;
    R << 1.0,     0,      0,
         0, c_theta, -s_theta,
         0, s_theta,  c_theta;
    return R;
}

/**
 * @brief Rotation matrix about the X-axis from angle in radians.
 */
Matrix<3,3> rotX(double theta)
{
    return rotX(sin(theta), cos(theta));
}

/**
 * @brief Derivative of rotation matrix about X-axis w.r.t. angle from sin/cos.
 *
 * This corresponds to d/dθ (rotX(θ)).
 */
Matrix<3,3> drotX(double s_theta, double c_theta)
{
    Matrix<3, 3> R;
    R << 0, 0, 0,
         0, -s_theta, -c_theta,
         0,  c_theta, -s_theta;
    return R;
}

/**
 * @brief Derivative of rotation matrix about X-axis from angle in radians.
 */
Matrix<3,3> drotX(double theta)
{
    return drotX(sin(theta), cos(theta));
}

/**
 * @brief Rotation matrix about the Y-axis from sine and cosine of angle.
 */
Matrix<3,3> rotY(double s_theta, double c_theta)
{
    Matrix<3, 3> R;
    R << c_theta, 0, s_theta,
         0,      1.0, 0,
        -s_theta, 0, c_theta;
    return R;
}

/**
 * @brief Rotation matrix about the Y-axis from angle in radians.
 */
Matrix<3,3> rotY(double theta)
{
    return rotY(sin(theta), cos(theta));
}

/**
 * @brief Derivative of rotation matrix about Y-axis w.r.t. angle from sin/cos.
 */
Matrix<3,3> drotY(double s_theta, double c_theta)
{
    Matrix<3, 3> R;
    R << -s_theta, 0, c_theta,
          0,      0, 0,
         -c_theta, 0, -s_theta;
    return R;
}

/**
 * @brief Derivative of rotation matrix about Y-axis from angle in radians.
 */
Matrix<3,3> drotY(double theta)
{
    return drotY(sin(theta), cos(theta));
}

/**
 * @brief Rotation matrix about the Z-axis from sine and cosine of angle.
 */
Matrix<3,3> rotZ(double s_theta, double c_theta)
{
    Matrix<3, 3> R; 
    R << c_theta, -s_theta, 0,
         s_theta,  c_theta, 0,
         0,        0,       1.0;
    return R;   
}

/**
 * @brief Rotation matrix about the Z-axis from angle in radians.
 */
Matrix<3,3> rotZ(double theta)
{
    return rotZ(sin(theta), cos(theta));
}

/**
 * @brief Derivative of rotation matrix about Z-axis w.r.t. angle from sin/cos.
 */
Matrix<3,3> drotZ(double s_theta, double c_theta)
{
    Matrix<3, 3> R;
    R << -s_theta, -c_theta, 0,
          c_theta, -s_theta, 0,
          0,        0,       0;
    return R;                     
}

/**
 * @brief Derivative of rotation matrix about Z-axis from angle in radians.
 */
Matrix<3,3> drotZ(double theta)
{
    return drotZ(sin(theta), cos(theta));
}
///@}

/**
 * @brief Compute rotation matrix from roll-pitch-yaw angles using ZYX convention.
 *
 * Result: R = Rz(yaw) * Ry(pitch) * Rx(roll)
 */
Matrix<3,3> RPY(double r, double p, double y)
{
    return rotZ(y) * rotY(p) * rotX(r);
}

/**
 * @brief Compute RPY rotation matrix and its partial derivatives w.r.t. each angle.
 *
 * @param r Roll angle in radians
 * @param p Pitch angle in radians
 * @param y Yaw angle in radians
 * @param R Output: rotation matrix
 * @param dR_dr Output: ∂R/∂roll
 * @param dR_dp Output: ∂R/∂pitch
 * @param dR_dy Output: ∂R/∂yaw
 */
void RPY(double r, double p, double y, Matrix<3,3>& R, Matrix<3,3>& dR_dr, Matrix<3,3>& dR_dp, Matrix<3,3>& dR_dy)
{
    double sr = sin(r);
    double cr = cos(r);
    double sp = sin(p);
    double cp = cos(p);
    double sy = sin(y);
    double cy = cos(y);

    Matrix<3,3> Rx = rotX(sr, cr);
    Matrix<3,3> Ry = rotY(sp, cp);
    Matrix<3,3> Rz = rotZ(sy, cy);

    //R      = TODO
    //dR_dr  = TODO
    //dR_dp  = TODO
    //dR_dy  = TODO
}




