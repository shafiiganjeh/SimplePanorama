
#ifndef BLEND_H
#define BLEND_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "_util.h"
#include <Eigen/Dense>
#include "_img_manipulation.h"

namespace blnd {

struct size_data{

    cv::Size dims;
    int min_x;
    int min_y;
    int max_x;
    int max_y;

};

void erode(const cv::Mat& tmp, cv::Mat& dst, const cv::Mat& kernel);
cv::Mat multi_blend(const std::vector<cv::Mat>& images,const std::vector<cv::Mat>& masks,const std::vector<cv::Mat>& masks_orig,const std::vector<cv::Point>& top_lefts,int bands,double sigma);

cv::Mat createSurroundingMask(const cv::Mat& inputImage, bool invert = false, uchar thresholdValue = 1);

cv::Mat simple_blend(const std::vector<cv::Mat>& images,const std::vector<cv::Mat>& masks,const std::vector<cv::Point>& top_lefts);

cv::Mat no_blend(const std::vector<cv::Mat>& images,const std::vector<cv::Mat>& masks,const std::vector<cv::Point>& top_lefts);


}

#endif

