
#ifndef GAIN_H
#define GAIN_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "_maths.h"
#include <Eigen/Dense>

namespace gain {

std::pair<int,float> get_overlapp_intensity(const cv::Mat &base, const cv::Mat &attach,const cv::Matx33f &H);

std::pair<Eigen::VectorXf,Eigen::MatrixXf> set_up_equations(std::vector<cv::Mat> &imags,const cv::Mat& adj,std::vector<std::vector< cv::Matx33f >>& Hom_mat);

std::vector<float> gain_compensation(std::vector<cv::Mat> &imags,const cv::Mat& adj,std::vector<std::vector< cv::Matx33f >>& Hom_mat);

}

#endif
