
#ifndef STITCH_H
#define STITCH_H

#include <opencv2/opencv.hpp>
#include "_util.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "_img_manipulation.h"
#include <tuple>
#include <iostream>
#include <fstream>
#include <unsupported/Eigen/MatrixFunctions>
#include "_homography.h"

#include <vector>
#include <string>
#include "_image.h"
#include "_blending.h"
#include "_bundle_adjust_main.h"
#include <climits>
#include "_gain_compensation.h"
#include "_straightening.h"
#define PI 3.14159265

#include "_graph_cut.h"

namespace stch {

struct stitch_data {
    std::vector<cv::Mat> imgs;
    std::vector<cv::Mat> msks;
    std::vector<cv::Mat> msks_cut;
    std::vector<cv::Point> corners;
    cv::Mat adj;
};


struct stitch_result {

    std::vector<Eigen::MatrixXd> rot;
    std::vector<Eigen::MatrixXd> K;
    std::vector<cv::Point> corners;
    std::vector<cv::Mat> masks;
    std::vector<cv::Mat> imgs;
    int maxLoc;
    std::vector<double> connectivity;
    std::vector<int> ind;
    std::vector<int> ord;
    cv::Mat adj;
    struct util::size_data prev_size;

};

    struct NodeConnection {
    int nodeAdded;
    int connectedTo;
};


    struct adjust_par{

        cv::Mat adj;
        std::map<int, int> ind2numb;
        std::map<int, int> numb2ind;
        int ref;
        int attatch;
        class imgm::pan_img_transform T;
        adjust_par(const cv::Mat* adj_mat,const std::vector<cv::Mat> *imags) : T(adj_mat,imags) {}
        std::vector<util::keypoints> kp;
        std::vector<std::vector<std::vector<cv::DMatch>>> match;

    };

    struct stitch_data get_proj_parameters(
        const std::vector<cv::Mat>& images,
        std::vector<Eigen::MatrixXd>& R,
        std::vector<Eigen::MatrixXd>& K,
        int maxLoc,
        std::vector<double> &con);

    struct stitch_result bundleadjust_stitching(class imgm::pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >> &Hom_mat,const std::vector<util::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,float lambda = .0001 ,int threads = 1,std::atomic<double>* f_adress = NULL,std::atomic<bool>* c_adress = NULL);

    std::vector<double> computeRowSumDividedByZeroCount(const cv::Mat& mat);

}

#endif
