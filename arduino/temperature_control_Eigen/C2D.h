#pragma once

#include <Eigen/Dense>

// Type alias for a fixed-size matrix of floats using Eigen
template<int Rows, int Cols>
using Matrix = Eigen::Matrix<float, Rows, Cols>;

/**
 * @brief Computes the matrix exponential exp(A) using a truncated Taylor series.
 *
 * This function approximates the exponential of a square matrix A using
 * the Taylor series expansion:
 *     exp(A) = I + A + A²/2! + A³/3! + ...
 * The computation stops once the norm of the next term is below a given tolerance.
 *
 * @tparam n Size of the square matrix A (n x n).
 * @param[in] A   Input matrix to exponentiate.
 * @param[in] tol Tolerance to stop the expansion (default: 1e-1).
 * @return exp(A) approximated using series expansion.
 */
template<int n>
Matrix<n, n> expmat(const Matrix<n, n>& A, float tol = 1e-1)
{
    Matrix<n, n> eA = Matrix<n, n>::Zero();       // Result matrix
    Matrix<n, n> Ai = Matrix<n, n>::Identity();   // A^i initialized to I
    float term_norm;
    unsigned long fac = 1;                        // Factorial accumulator
    unsigned long i = 1;                          // Series index

    while (true)
    {
        eA += Ai / static_cast<float>(fac);       // Add current term to series

        term_norm = (Ai.norm()) / static_cast<float>(fac);  // Estimate contribution of next term
        if (term_norm < tol) break;               // Stop if contribution is negligible

        Ai *= A;       // Compute A^i for next iteration
        fac *= i;      // Update factorial
        ++i;
    }

    return eA;
}

/**
 * @brief Converts a continuous-time linear system (A, B) to discrete-time (Ad, Bd) using zero-order hold.
 *
 * The conversion is based on the matrix exponential:
 *     Ad = exp(Ac * Ts)
 *     Bd = A⁻¹ (Ad - I) B
 *
 * @tparam n State dimension.
 * @tparam r Input dimension.
 * @param[out] Ad Discrete-time system matrix.
 * @param[out] Bd Discrete-time input matrix.
 * @param[in]  Ac Continuous-time system matrix.
 * @param[in]  Bc Continuous-time input matrix.
 * @param[in]  Ts Sampling time (discretization step).
 */
template<int n, int r>
void C2D(
    Matrix<n, n>& Ad,
    Matrix<n, r>& Bd,
    const Matrix<n, n>& Ac,
    const Matrix<n, r>& Bc,
    const float Ts)
{
    Ad = expmat<n>(Ac * Ts);   // Discrete-time A matrix using matrix exponential

    // Bd computed using exact formula: Bd = A⁻¹ (Ad - I) B
    Bd = Ac.fullPivLu().solve((Ad - Matrix<n, n>::Identity()) * Bc);
    //Bd = Ac.inverse() * ((Ad - Matrix<n, n>::Identity()) * Bc);
}