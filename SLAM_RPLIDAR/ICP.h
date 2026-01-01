#include <Eigen/Dense>
#include "MapKD.hpp"
#include "geometry.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Matrix3d;


// Residual + Jacobian evaluation
void get_rJ_RPY(
    const Eigen::Matrix<double,6,1>& x_hat,
    const MatrixXd& pA,   // Nx3
    const MatrixXd& pB,   // Nx3
    VectorXd& r,          // 3N x 1
    MatrixXd& J           // 3N x 6
) {
    const int N = pA.rows();

    // Extract rotation and translation
    const Vector3d theta_hat = x_hat.head<3>();
    const Vector3d t_hat     = x_hat.tail<3>();

    // Rotation and derivatives
    Matrix3d R_hat, dR1, dR2, dR3;
    RPY(theta_hat(0), theta_hat(1), theta_hat(2), R_hat, dR1, dR2, dR3);

    // Resize outputs
    r.setZero(3 * N);
    J.setZero(3 * N, 6);

    for (int i = 0; i < N; ++i) {
        const Vector3d pa = pA.row(i).transpose();
        const Vector3d pb = pB.row(i).transpose();

        // Residual
        // r.segment<3>(3 * i) = TODO

        // Jacobian wrt rotation
        //J.block<3,1>(3 * i, 0) = TODO
        
        // Jacobian wrt translation
        //J.block<3,3>(3 * i, 3) = TODO
    }
}


// Gauss-Newton optimizer (local g, H)
template<typename Func>
void optim_GaussNewton(
    Func&& f,
    Eigen::Matrix<double,6,1>& x_hat,
    double tol,
    int max_iters = 100
) {
    VectorXd r;
    MatrixXd J;

    for (int iter = 0; iter < max_iters; ++iter) {
        f(x_hat, r, J);

        //Eigen::Matrix<double,6,6> H = TODO
        //Eigen::Matrix<double,6,1> g = TODO

        //if (g.squaredNorm() < tol)
        //    break;

       //const Eigen::Matrix<double,6,1> delta = TODO
       //x_hat = TODO
    }
}

// Transformation estimation wrapper
void transform_estimate(
    const MatrixXd& pA,   // Nx3
    const MatrixXd& pB,   // Nx3
    Eigen::Matrix<double,6,1>& x_hat,
    double tol
) {
     auto f = [&](const Eigen::Matrix<double,6,1>& x, VectorXd& r, MatrixXd& J) {
        get_rJ_RPY(x, pA, pB, r, J);
      };
     optim_GaussNewton(f, x_hat, tol); 
}

// ICP with map correspondences
std::vector<Point> ICP_map_estimate(
    const MatrixXd& pA,  
    Map& map,
    Matrix3d& R_hat,
    Vector3d& t_hat,
    double max_distance,
    double GN_tol,
    double ICP_tol_pos,
    double ICP_tol_ori
) {     
    const int N = pA.rows();
    std::vector<Point> pA_transformed;
    pA_transformed.reserve(N);

    for (int iter = 0; iter < 1000; ++iter) {     
        pA_transformed.clear();
        //for (int i = 0; i < N; ++i)
        //TODO   pA_transformed.emplace_back(TODO);

        // Correspondences
        auto correspondences = map.findCorrespondences(pA_transformed, max_distance);

        const size_t M = correspondences.size();

        MatrixXd pA_(M, 3);
        MatrixXd pB_(M, 3);

        for (size_t i = 0; i < M; ++i) {
            const auto& [pa, pb] = correspondences[i];
            pA_.row(i) = pa;
            pB_.row(i) = pb;
        } 

        // Estimate incremental transform
        Eigen::Matrix<double,6,1> dx= Eigen::Matrix<double,6,1>::Zero();

        transform_estimate(pA_, pB_, dx, GN_tol);

        const Vector3d dtheta = dx.head<3>();
        const Vector3d dt     = dx.tail<3>();

        // Apply increment
        Matrix3d R_inc = RPY(dtheta(0), dtheta(1), dtheta(2));
        //R_hat = TODO
        //t_hat = TODO

        // Check convergence
        //if (TODO) { 
        //    break; 
        //}
    }
    return pA_transformed;   
}
