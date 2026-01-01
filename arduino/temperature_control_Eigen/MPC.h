#pragma once

/**
 * @brief Defines a fixed-size Eigen matrix type alias with float precision.
 * 
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 */
template<int Rows, int Cols>
using Matrix = Eigen::Matrix<float, Rows, Cols>;

namespace MPC
{


template<int n, int r, int m, int p>
void getMN_output(Matrix<p * m, n> &M,
           Matrix<p * m, p * r> &N,
           const Matrix<n, n> &A,
           const Matrix<n, r> &B,
           const Matrix<m, n> &C) {
  M = Matrix<p * m, n>::Zero();
  N = Matrix<p * m, p * r>::Zero();
  Matrix<n, n> Ai = Matrix<n, n>::Identity();

  for (int i = 0; i < p; i++) {
   // TODO 
  }

  for (int i = 0; i < p - 1; i++) {
    for (int j = i + 1; j < p; j++) {
     // TODO
    }
  }
}

/**
 * @brief Constructs the Gamma matrix that maps ΔU to U in incremental MPC.
 * 
 * @tparam r Input dimension
 * @tparam p Prediction horizon
 * @return Gamma matrix (p*r x p*r)
 */
template<int r, int p>
Matrix<r * p, r * p> getGamma() {
  Matrix<r * p, r * p> Gamma = Matrix<r * p, r * p>::Zero();

  for (int i = 0; i < p; ++i) {
    for (int j = 0; j <= i; ++j)
      {}// TODO
  }

  return Gamma;
}


template<int n, int r, int m, int p>
void getHbIncrementalOutput(Matrix<p * r, 1> &b,
                      Matrix<p * r, p * r> &H,
                      const Matrix<m, m> &Q,
                      const Matrix<r, r> &R,
                      const Matrix<n, n> &A,
                      const Matrix<n, r> &B,
                      const Matrix<m, n> &C,
                      const Matrix<n, 1> &x,
                      const Matrix<m, 1> &y_ref,
                      const Matrix<r, 1> &u_prev) {
    Matrix<p * m, n> M;
    Matrix<p * m, p * r> N;
    getMN_output<n, r, m , p>(M, N, A, B, C);

    Matrix<p * r, p * r> R_tilde = Matrix<p * r, p * r>::Zero();
    Matrix<p * m, p * m> Q_tilde = Matrix<p * m, p * m>::Zero();

    for (int i = 0; i < p; ++i) {
        R_tilde.block(i * r, i * r, r, r) = R;
        Q_tilde.block(i * m, i * m, m, m) = Q;
    }

    Matrix<p * r, r * p> Gamma = MPC::getGamma<r, p>();

    Matrix<r * p, 1> U_prev_stack;
    for (int i = 0; i < p; ++i)
       // TODO { }

    Matrix<p * m, 1> Y_ref_tilde;
    for (int i = 0; i < p; ++i)
    {
        // TODO
    }
    // TODO H = 
    // TODO b =
}

/**
 * @brief Builds input constraints for incremental MPC using ΔU.
 */
template<int r, int p>
void getU_con_incremental(Matrix<r * p * 2, r * p> &Ac,
                          Matrix<r * p * 2, 1> &bc,
                          const Matrix<r, 1> &u_max,
                          const Matrix<r, 1> &u_min,
                          const Matrix<r, 1> &u_prev) {
  Ac = Matrix<r * p * 2, r * p>::Zero();
  bc = Matrix<r * p * 2, 1>::Zero();

  Matrix<r * p, r * p> Gamma = MPC::getGamma<r, p>();

  for (int i = 0; i < p; ++i) {
    // TODO
  }

  // TODO
}


} // namespace MPC