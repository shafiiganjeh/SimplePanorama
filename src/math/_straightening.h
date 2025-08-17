#ifndef STRG_H
#define STRG_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

namespace strg {

//See: https://link.springer.com/article/10.1007/s11263-006-0002-3

std::vector<Eigen::MatrixXd> straightenPanorama(std::vector<Eigen::MatrixXd>& rotations);

}

#endif
