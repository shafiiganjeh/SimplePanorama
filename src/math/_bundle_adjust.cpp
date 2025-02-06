#include "_bundle_adjust.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

Eigen::MatrixXd cov_mat_XY(Eigen::MatrixXd X,Eigen::MatrixXd Y){

    Eigen::MatrixXd con_XY;

    if((X.rows() != Y.rows())){

        throw std::invalid_argument("Error: random vector X and Y have different sizes.");

    }

    Eigen::MatrixXd Xv;
    Eigen::MatrixXd Yv;

    Xv = (X.colwise() - X.rowwise().mean());
    Yv = (Y.colwise() - Y.rowwise().mean());

    con_XY = (Xv * Yv.transpose())/X.rows();

    return con_XY;

}






