#pragma once

#include "Eigen/Dense"

/**
 * @brief Type alias for fixed-size Eigen matrix of floats.
 */
template<int Rows, int Cols>
using Matrix = Eigen::Matrix<float, Rows, Cols>;

class EulerAngles {
public:
    float r; ///< Roll [rad]
    float p; ///< Pitch [rad]
    float y; ///< Yaw [rad]

    /// Default constructor (zero angles)
    EulerAngles() : r(0.0f), p(0.0f), y(0.0f) {}

    /// Constructor from explicit values
    EulerAngles(float roll, float pitch, float yaw)
        : r(roll), p(pitch), y(yaw) {}

    /// Constructor from rotation matrix (XYZ order)
    explicit EulerAngles(const Matrix<3,3>& R) {
        Eigen::Vector3f angles = R.eulerAngles(2, 1, 0);
        y = angles(0); // yaw   (rotation about Z)
        p = angles(1); // pitch (rotation about Y)
        r = angles(2); // roll  (rotation about X)
    }

     explicit EulerAngles(const Eigen::Quaternionf& q):EulerAngles(q.toRotationMatrix()) {}

    Eigen::Quaternionf toQuat()const {
        Eigen::AngleAxisf rollAngle(r, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(p, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(y, Eigen::Vector3f::UnitZ());
        return yawAngle * pitchAngle * rollAngle; // ZYX order
    }

    /// Convert back to rotation matrix (XYZ order)
    Matrix<3,3> toRotationMatrix() const {
        return toQuat().toRotationMatrix(); // ZYX composition
    }
};

class positionOrientation : public EulerAngles {
public:
    float X; ///< X position in same units as reference frame
    float Y; ///< Y position
    float Z; ///< Z position

    positionOrientation()
        : EulerAngles(), X(0), Y(0), Z(0) {}

     positionOrientation(float X, float Y, float Z,float r, float p, float y)
        : EulerAngles(r,p,y), X(X), Y(Y), Z(Z) {}    

    positionOrientation(const Matrix<3,1>& pos, const Matrix<3,3>& R)
        : EulerAngles(R), X(pos(0)), Y(pos(1)), Z(pos(2)) {}

    Matrix<3,1> positionVector() const {
        return (Matrix<3,1>() << X, Y, Z).finished();
    }

    Eigen::Vector3f anglesVector() const { return {r, p, y}; }
};

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
Matrix<3,3> rotX(float s_theta, float c_theta)
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
Matrix<3,3> rotX(float theta)
{
    return rotX(sin(theta), cos(theta));
}

/**
 * @brief Derivative of rotation matrix about X-axis w.r.t. angle from sin/cos.
 *
 * This corresponds to d/dθ (rotX(θ)).
 */
Matrix<3,3> drotX(float s_theta, float c_theta)
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
Matrix<3,3> drotX(float theta)
{
    return drotX(sin(theta), cos(theta));
}

/**
 * @brief Rotation matrix about the Y-axis from sine and cosine of angle.
 */
Matrix<3,3> rotY(float s_theta, float c_theta)
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
Matrix<3,3> rotY(float theta)
{
    return rotY(sin(theta), cos(theta));
}

/**
 * @brief Derivative of rotation matrix about Y-axis w.r.t. angle from sin/cos.
 */
Matrix<3,3> drotY(float s_theta, float c_theta)
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
Matrix<3,3> drotY(float theta)
{
    return drotY(sin(theta), cos(theta));
}

/**
 * @brief Rotation matrix about the Z-axis from sine and cosine of angle.
 */
Matrix<3,3> rotZ(float s_theta, float c_theta)
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
Matrix<3,3> rotZ(float theta)
{
    return rotZ(sin(theta), cos(theta));
}

/**
 * @brief Derivative of rotation matrix about Z-axis w.r.t. angle from sin/cos.
 */
Matrix<3,3> drotZ(float s_theta, float c_theta)
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
Matrix<3,3> drotZ(float theta)
{
    return drotZ(sin(theta), cos(theta));
}
///@}

/**
 * @brief Create a 4x4 homogeneous transformation matrix from rotation and translation.
 *
 * Homogeneous matrix form:
 * [ R t ]
 * [ 0 1 ]
 *
 * @param R 3x3 rotation matrix
 * @param t 3x1 translation vector
 * @return 4x4 homogeneous transformation matrix
 */
inline Matrix<4,4> toHomogeneous(const Matrix<3,3>& R, const Matrix<3,1>& t) {
    Matrix<4,4> T = Matrix<4,4>::Identity();
    T.template block<3,3>(0,0) = R;
    T.template block<3,1>(0,3) = t;
    return T;
}

/**
 * @brief Extract rotation matrix and translation vector from a homogeneous transformation matrix.
 *
 * @param T 4x4 homogeneous transformation matrix
 * @param R Output 3x3 rotation matrix
 * @param t Output 3x1 translation vector
 */
inline void fromHomogeneous(
    const Matrix<4,4>& T,
    Matrix<3,3>& R,
    Matrix<3,1>& t
) {
    R = T.template block<3,3>(0,0);
    t = T.template block<3,1>(0,3);
}

/**
 * @brief Compute rotation matrix from roll-pitch-yaw angles using ZYX convention.
 *
 * Result: R = Rz(yaw) * Ry(pitch) * Rx(roll)
 */
Matrix<3,3> RPY(float r, float p, float y)
{
    return rotZ(y) * rotY(p) * rotX(r);
}

Matrix<3,3> RPY(const EulerAngles& e)
{
    return RPY(e.r, e.p, e.y);
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
void RPY(float r, float p, float y, Matrix<3,3>& R, Matrix<3,3>& dR_dr, Matrix<3,3>& dR_dp, Matrix<3,3>& dR_dy)
{
    float sr = sin(r);
    float cr = cos(r);
    float sp = sin(p);
    float cp = cos(p);
    float sy = sin(y);
    float cy = cos(y);

    Matrix<3,3> Rx = rotX(sr, cr);
    Matrix<3,3> Ry = rotY(sp, cp);
    Matrix<3,3> Rz = rotZ(sy, cy);

    R      = Rz * Ry * Rx;
    dR_dr  = Rz * Ry * drotX(sr, cr);
    dR_dp  = Rz * drotY(sp, cp) * Rx;
    dR_dy  = drotZ(sy, cy) * Ry * Rx;
}

/**
 * @brief Abstract base class for rotation matrix generation.
 *
 * Provides an interface for:
 * - Computing a rotation matrix from an angle.
 * - Computing its derivative w.r.t. the angle.
 * - Computing both in a single call.
 */
class RotationMatrix {
public:
    virtual ~RotationMatrix() = default;

    /// Compute rotation matrix from angle
    virtual Matrix<3,3> operator()(float theta) const = 0;

    /// Compute derivative of rotation matrix w.r.t. angle
    virtual Matrix<3,3> d(float theta) const = 0;

    /// Compute both rotation matrix and derivative from angle
    virtual void operator()(float theta, Matrix<3,3> R, Matrix<3,3> &dR) const = 0;
};

/**
 * @brief Rotation about X-axis (right-hand rule).
 */
class RotationX : public RotationMatrix {
public:
    Matrix<3,3> operator()(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return rotX(s, c);
    }

    Matrix<3,3> d(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return drotX(s, c);
    }

    void operator()(float theta, Matrix<3,3> R, Matrix<3,3> &dR) const override {
        float s = sin(theta), c = cos(theta);
        R = rotX(s, c);
        dR = drotX(s, c);
    }
};

/**
 * @brief Rotation about Y-axis.
 */
class RotationY : public RotationMatrix {
public:
    Matrix<3,3> operator()(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return rotY(s, c);
    }

    Matrix<3,3> d(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return drotY(s, c);
    }

    void operator()(float theta, Matrix<3,3> R, Matrix<3,3> &dR) const override {
        float s = sin(theta), c = cos(theta);
        R = rotY(s, c);
        dR = drotY(s, c);
    }
};

/**
 * @brief Rotation about Z-axis.
 */
class RotationZ : public RotationMatrix {
public:
    Matrix<3,3> operator()(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return rotZ(s, c);
    }

    Matrix<3,3> d(float theta) const override {
        float s = sin(theta), c = cos(theta);
        return drotZ(s, c);
    }

    void operator()(float theta, Matrix<3,3> R, Matrix<3,3> &dR) const override {
        float s = sin(theta), c = cos(theta);
        R = rotZ(s, c);
        dR = drotZ(s, c);
    }
};




