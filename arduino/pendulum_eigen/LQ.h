#pragma once

// Alias for a fixed-size Eigen matrix of floats
template<int Rows, int Cols>
using Matrix = Eigen::Matrix<float, Rows, Cols>;

namespace LQ {

/**
 * @brief Solves the Discrete Algebraic Riccati Equation (DARE) and computes optimal feedback gain K.
 *
 * This function iteratively solves the DARE for a discrete-time linear system:
 *     x[k+1] = A x[k] + B u[k]
 * with quadratic cost:
 *     J = Σ (xᵀQx + uᵀRu)
 *
 * It computes the solution P to the Riccati equation and the corresponding optimal gain:
 *     K = -(BᵀPB + R)⁻¹ BᵀPA
 *
 * @tparam n State dimension.
 * @tparam r Control input dimension.
 *
 * @param[out] P   Riccati solution matrix (positive semi-definite).
 * @param[out] K   Optimal feedback gain matrix.
 * @param[in]  A   State transition matrix.
 * @param[in]  B   Input matrix.
 * @param[in]  Q   State cost matrix.
 * @param[in]  R   Control cost matrix.
 * @param[in]  tol Tolerance for convergence (difference between iterations).
 */
template<int n, int r>
void DARE(Matrix<n, n> &P,
          Matrix<r, n> &K,
          const Matrix<n, n> &A,
          const Matrix<n, r> &B,
          const Matrix<n, n> &Q,
          const Matrix<r, r> &R,
          float tol=1e-1)
{
    P = Matrix<n, n>::Zero();   // Initialize Riccati matrix
    K = Matrix<r, n>::Zero();   // Initialize feedback gain

    while (1)
    {
        Matrix<n, r> W = A.transpose() * P * B;

       
        K=-(B.transpose() * P * B + R).ldlt().solve(W.transpose());
        //K = -(B.transpose() * P * B + R).inverse() * W.transpose();

        Matrix<n, n> P_ = A.transpose() * P * A + Q + W * K;

        if ((P - P_).norm() < tol) {
            break; // Converged
        }

        P = P_; // Update P for next iteration
    }
}

} // namespace LQ
