#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

/**
 * @brief Alias for Eigen::Matrix with fixed dimensions.
 * 
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 */
template<int Rows, int Cols>
using Matrix = Eigen::Matrix<float, Rows, Cols>;

/**
 * @brief Solves a quadratic programming (QP) problem using a primal-dual interior-point method.
 * 
 * The problem is formulated as:
 * 
 *   minimize    0.5 * xᵀ A x + bᵀ x
 *   subject to  A_ineq * x ≤ b_ineq
 * 
 * where:
 * - @p A is a symmetric positive semi-definite matrix,
 * - @p b is a linear cost vector,
 * - @p A_ineq and @p b_ineq describe inequality constraints.
 * 
 * @tparam n Dimension of the optimization variable @p x.
 * @tparam n_con Number of inequality constraints.
 * 
 * @param[in,out] x Initial guess for the solution. On return, contains the computed optimizer.
 * @param[in] A Quadratic cost matrix (n × n).
 * @param[in] b Linear cost vector (n × 1).
 * @param[in] A_ineq Inequality constraint matrix (n_con × n).
 * @param[in] b_ineq Inequality constraint bound vector (n_con × 1).
 * @param[in] eps Convergence tolerance for residuals (default: 1e-6).
 * 
 * @details
 * The algorithm implements a primal-dual interior-point method:
 * - Introduces slack variables @p s and Lagrange multipliers @p lambda.
 * - Ensures strictly positive slack values at initialization.
 * - Iteratively solves the Karush-Kuhn-Tucker (KKT) system.
 * - Uses step-size selection to maintain positivity of slack and multipliers.
 * - Stops when the residual norm is below @p eps or after 50 iterations.
 * 
 * @note The method may fail to converge if @p A is not positive semidefinite,
 *       or if the problem is infeasible.
 * 
 * @warning The implementation uses fixed iteration limit (50). Increase if needed.
 */
template<int n, int n_con>
void quadprog(
    Matrix<n, 1>& x,
    const Matrix<n, n>& A,
    const Matrix<n, 1>& b,
    const Matrix<n_con, n>& A_ineq,
    const Matrix<n_con, 1>& b_ineq,
    float eps = 1e-6f)
{
    constexpr int total = n + 2 * n_con;

    Matrix<n_con, 1> lambda = Matrix<n_con, 1>::Constant(1.0f);
    Matrix<n_con, 1> s = b_ineq - A_ineq * x;

    // Ensure strictly positive slack
    for (int i = 0; i < n_con; ++i) {
        if (s(i) <= 0) s(i) = 1.0f;
    }

    for (int iter = 0; iter < 50; ++iter) {
        // Residuals
        Matrix<n, 1> r_dual = A * x + b + A_ineq.transpose() * lambda;
        Matrix<n_con, 1> r_prim = A_ineq * x - b_ineq + s;
        Matrix<n_con, 1> r_cent = s.array() * lambda.array();

        float mu = r_cent.sum() / n_con;
        float sigma = 0.1f;

        Matrix<n_con, 1> r_mu = r_cent.array() - sigma * mu;

        // Assemble KKT matrix
        Matrix<total, total> KKT = Matrix<total, total>::Zero();
        Matrix<total, 1> rhs;

        // Top-left A
        KKT.template block<n, n>(0, 0) = A;
        // Top-middle A_ineq'
        KKT.template block<n, n_con>(0, n) = A_ineq.transpose();
        // Middle-left A_ineq
        KKT.template block<n_con, n>(n, 0) = A_ineq;
        // Middle-right identity for slack
        KKT.template block<n_con, n_con>(n, n + n_con).setIdentity();
        // Bottom-middle diagonal(s)
        KKT.template block<n_con, n_con>(n + n_con, n).diagonal() = s;
        // Bottom-right diagonal(lambda)
        KKT.template block<n_con, n_con>(n + n_con, n + n_con).diagonal() = lambda;

        // Build RHS
        rhs << -r_dual,
               -r_prim,
               -r_mu;

        // Solve system
        Matrix<total, 1> delta = KKT.partialPivLu().solve(rhs);

        Matrix<n, 1> dx = delta.template segment<n>(0);
        Matrix<n_con, 1> dlambda = delta.template segment<n_con>(n);
        Matrix<n_con, 1> ds = delta.template segment<n_con>(n + n_con);

        // Compute step size to maintain positivity
        float alpha = 1.0f;
        for (int i = 0; i < n_con; ++i) {
            if (ds(i) < 0) {
                alpha = std::min(alpha, -0.99f * s(i) / ds(i));
            }
            if (dlambda(i) < 0) {
                alpha = std::min(alpha, -0.99f * lambda(i) / dlambda(i));
            }
        }

        // Convergence check
        float res_norm = std::max({ r_dual.norm(), r_prim.norm(), r_mu.norm() });
        if (res_norm < eps)
            return;

        // Update
        x += alpha * dx;
        lambda += alpha * dlambda;
        s += alpha * ds;
    }
}