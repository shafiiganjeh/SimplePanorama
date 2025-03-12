
#ifndef BLEND_H
#define BLEND_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "_maths.h"
#include <Eigen/Dense>
#include "_img_manipulation.h"


namespace blnd {

void simple_blend(const class imgm::pan_img_transform &Tr,const std::vector<cv::Mat> &imags);

void no_blend(const class imgm::pan_img_transform &Tr,const std::vector<cv::Mat> &imags);

void simple_test(const class imgm::pan_img_transform &Tr,const std::vector<cv::Mat> &imags);

void graph_blend(const class imgm::pan_img_transform &Tr,const std::vector<cv::Mat> &imags);

}

#endif

