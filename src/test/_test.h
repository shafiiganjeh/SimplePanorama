

#ifndef TEST_H
#define TEST_H

#include <opencv2/opencv.hpp>
#include "_util.h"
#include <Eigen/Dense>
#include "_distance_cut.h"
#include "_img_manipulation.h"


namespace test {


std::vector<cv::Mat> equalizeIntensities(const std::vector<cv::Mat>& images,const std::vector<cv::Mat>& masks,const std::vector<cv::Point>& top_lefts,float ratio = .5);

void adjust_intensity(std::vector<cv::Mat>& images,const std::vector<cv::Mat>& intensities);

}

#endif



