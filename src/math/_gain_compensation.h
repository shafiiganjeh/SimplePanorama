
#ifndef GAIN_H
#define GAIN_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "_maths.h"
#include <Eigen/Dense>
#include "_blending.h"

namespace gain {

struct OverlapInfo {
    int i;
    int j;
    double area;
    double I_i;
    double I_j;
};


std::vector<OverlapInfo> get_overlapp_intensity(
    const std::vector<cv::Mat>& warped_images,
    const std::vector<cv::Point>& corners,
    const cv::Mat& adj_pass);

std::pair<Eigen::VectorXd,Eigen::MatrixXd> set_up_equations(const std::vector<cv::Mat> &imags,const cv::Mat& adj,const std::vector<cv::Point>& corners);

std::vector<double> gain_compensation(const std::vector<cv::Mat> &imags,const cv::Mat& adj_pass,const std::vector<cv::Point>& corners);

}

#endif
