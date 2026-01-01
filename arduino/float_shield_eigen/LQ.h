#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

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


template<int n, int r>
void DARE_Schur(Matrix<n, n> &P,
                Matrix<r, n> &K,
                const Matrix<n, n> &A,
                const Matrix<n, r> &B,
                const Matrix<n, n> &Q,
                const Matrix<r, r> &R,
                double tol = 1e-9)
{
    using MatNN = Matrix<n,n>;
    using MatRR = Matrix<r,r>;
    using Mat2N = Matrix<2*n,2*n>;

    MatRR Rinv = R.ldlt().solve(MatRR::Identity());
    MatNN AinvT = A.transpose().lu().solve(MatNN::Identity());

    Mat2N H;
    H << A, -(B * Rinv * B.transpose()),
         -Q, AinvT;

    Eigen::RealSchur<Mat2N> schur(H);
    Mat2N U = schur.matrixU();
    Mat2N T = schur.matrixT();

    // TODO: reorder Schur decomposition so eigenvalues inside unit circle come first
    // Eigen does not provide built-in reordering for real Schur yet -> custom implementation

    MatNN U11 = U.topLeftCorner(n, n);
    MatNN U21 = U.bottomLeftCorner(n, n);

    // Solve U11 * X = U21 for X instead of inverse
    P = U11.fullPivLu().solve(U21);

    MatRR tmp = B.transpose() * P * B + R;
    K = - tmp.ldlt().solve(B.transpose() * P * A);
}

} // namespace LQ

/**
 * @brief Constructs an augmented system (A_tilde, B_tilde, C_tilde) for integral action or observer design.
 *
 * The augmented system has the form:
 *     [ A     0 ]
 * A~ = [ -C   I ]
 *
 *     [ B ]
 * B~ = [ 0 ]
 *
 *     [ C  I ]
 * C~ = 
 *
 * Typically used to add integral states to track reference or estimate output error.
 *
 * @tparam n State dimension.
 * @tparam r Input dimension.
 * @tparam m Output dimension (number of constraints or outputs).
 *
 * @param[out] A_tilde Augmented system matrix (n+m x n+m).
 * @param[out] B_tilde Augmented input matrix (n+m x r).
 * @param[out] C_tilde Augmented output matrix (m x n+m).
 * @param[in]  A       Original system matrix.
 * @param[in]  B       Original input matrix.
 * @param[in]  C       Original output matrix.
 */
template<int n, int r, int m>
void getAugmentedSystem(Matrix<n + m, n + m> &A_tilde,
                        Matrix<n + m, r> &B_tilde,
                        Matrix<m, n + m> &C_tilde,
                        const Matrix<n, n> &A,
                        const Matrix<n, r> &B,
                        const Matrix<m, n> &C)
{
    A_tilde = Matrix<n + m, n + m>::Zero();
    B_tilde = Matrix<n + m, r>::Zero();
    C_tilde = Matrix<m, n + m>::Zero();

    // Fill A_tilde:
    // Top-left block is original A
    // Bottom-left is -C to integrate output error
    // Bottom-right is identity for integral state
    A_tilde.template block<n, n>(0, 0) = A;
    A_tilde.template block<m, n>(n, 0) = -C;
    A_tilde.template block<m, m>(n, n) = Matrix<m, m>::Identity();

    // Fill B_tilde:
    // Only top rows affected by B
    B_tilde.template block<n, r>(0, 0) = B;

    // Fill C_tilde:
    // Used to recover outputs with integral states
    C_tilde.template block<m, n>(0, 0) = C;
    C_tilde.template block<m, m>(0, n) = Matrix<m, m>::Identity();
}