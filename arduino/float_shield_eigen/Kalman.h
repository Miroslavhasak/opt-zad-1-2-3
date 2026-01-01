#pragma once
#include <Eigen/Dense>

// Define a template alias for Eigen matrices with float precision
template<int Rows, int Cols> using Matrix = Eigen::Matrix<float, Rows, Cols>;

/**
 * @brief Standard Kalman Filter class for linear systems.
 * 
 * @tparam n Number of states
 * @tparam r Number of inputs
 * @tparam m Number of outputs
 */
template<int n, int r, int m> class KalmanFilter {
public:

  /**
   * @brief Construct a new Kalman Filter object
   * 
   * @param A System matrix
   * @param B Input matrix
   * @param C Output matrix
   * @param R Measurement noise covariance
   * @param Q Process noise covariance
   * @param P0 Initial error covariance
   * @param x_hat0 Initial state estimate
   */
  KalmanFilter(const Matrix<n, n>& A, const Matrix<n, r>& B, const Matrix<m, n>& C, const Matrix<m, m>& R, const Matrix<n, n>& Q, const Matrix<n, n>& P0, const Matrix<n,1>& x_hat0)
    : A(A), B(B), C(C), Q(Q), R(R), x_hat(x_hat0), P(P0) {
    K = Matrix<n, m>::Zero();
  }

  // Getters for internal states
  Matrix<n, n> get_P() { return P; }
  Matrix<n, 1> get_x_hat() { return x_hat; }
  Matrix<n, m> get_K() { return K; }

  /**
   * @brief Update step of Kalman filter: predict and correct
   * 
   * @param y Measurement
   * @param u Input
   * @return Updated state estimate
   */
  Matrix<n, 1> update(const Matrix<m, 1>& y, const Matrix<r, 1>& u) {
    // TODO
    return x_hat;
  }

  /**
   * @brief Predict the next state and error covariance
   * 
   * @param u Input vector
   * @return Predicted state estimate
   */
  Matrix<n, 1> predict(const Matrix<r, 1>& u) {
    // TODO
    return x_hat;
  }

  /**
   * @brief Compute Kalman gain
   */
  void calculateK() {
   // TODO K =
  }

  /**
   * @brief Correct the prediction using measurement
   * 
   * @param y Measurement vector
   * @return Corrected state estimate
   */
  Matrix<n, 1> correct(const Matrix<m, 1>& y) {
    calculateK();
    // TODO
    return x_hat;
  }

private:
  const Matrix<n, n>& A;
  const Matrix<n, r>& B;
  const Matrix<m, n>& C;
  const Matrix<n, n>& Q;
  const Matrix<m, m>& R;

  Matrix<n, n> P;
  Matrix<n, 1> x_hat;
  Matrix<n, m> K;
};


