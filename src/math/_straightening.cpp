#include "_straightening.h"

namespace strg {

std::vector<Eigen::MatrixXd> straightenPanorama(std::vector<Eigen::MatrixXd>& rotations) {
        // 1. Extract camera X-vectors
        std::vector<Eigen::Vector3d> X_vectors;
        for (const auto& R : rotations) {
            X_vectors.push_back(R.col(0));
        }
        // 2. Compute covariance matrix C = Σ(X_i * X_i^T)
        Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
        for (const auto& X : X_vectors) {
            C += X * X.transpose();
        }

        // 3. Find up-vector
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(C);
        const Eigen::Vector3d world_up(0.0, 1.0, 0.0);
        Eigen::Vector3d up_vector = eigensolver.eigenvectors().col(0);

        if (up_vector.dot(world_up) < 0) {
            up_vector = -up_vector;
        }

        // 4. Get rotation such that up vector is horizontal
        Eigen::Vector3d w_cross = up_vector.cross(world_up);

        double s = w_cross.norm();
        double c = up_vector.dot(world_up);

        const Eigen::Matrix3d v = (Eigen::Matrix3d() <<
            0, -w_cross[2], w_cross[1],
            w_cross[2], 0, -w_cross[0],
            -w_cross[1], w_cross[0], 0).finished();

        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d g_rot = I + v + v*v * (1-c)/(s*s);

        std::vector<Eigen::MatrixXd> R_corrected;

        for(int i = 0;i < rotations.size();i++){

            R_corrected.push_back(g_rot*rotations[i]);

        }

        return R_corrected;

    }

}

