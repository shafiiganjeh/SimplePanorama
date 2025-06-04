
#ifndef STITCH_H
#define STITCH_H

#include <opencv2/opencv.hpp>
#include "_maths.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "_img_manipulation.h"
#include <tuple>
#include <iostream>
#include <fstream>
#include <unsupported/Eigen/MatrixFunctions>

#include <vector>
#include <string>
#include "_image.h"
#include "_blending.h"
#include "_bundle_adjust_main.h"
#include <climits>
#include "_gain_compensation.h"
#define PI 3.14159265

namespace stch {

struct OverlapInfo {
    int i;
    int j;
    double area;
    double I_i;
    double I_j;
};

struct stitch_data {
    std::vector<cv::Mat> imgs;
    std::vector<cv::Point> corners;
    cv::Mat adj;
};

    struct NodeConnection {
    int nodeAdded;
    int connectedTo;
};

    struct bundle_par{

        Eigen::MatrixXd k_val;
        Eigen::MatrixXd k_key;
        Eigen::MatrixXd rot_key;
        int prev_node;
        int current_node;
        cv::Matx33f translation;

    };

    struct adjust_par{

        cv::Mat adj;
        std::map<int, int> ind2numb;
        std::map<int, int> numb2ind;
        int ref;
        int attatch;
        class imgm::pan_img_transform T;
        adjust_par(const cv::Mat* adj_mat,const std::vector<cv::Mat> *imags) : T(adj_mat,imags) {}
        std::vector<maths::keypoints> kp;
        std::vector<std::vector<std::vector<cv::DMatch>>> match;

    };

    std::vector<std::vector< cv::Matx33f >> bundleadjust_stitching(class imgm::pan_img_transform &T,class imgm::pan_img_transform &Tnew,const std::vector<std::vector< cv::Matx33f >> &Hom_mat,const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat, int threads = 1);

    std::vector<double> computeRowSumDividedByZeroCount(const cv::Mat& mat);

}

#endif
