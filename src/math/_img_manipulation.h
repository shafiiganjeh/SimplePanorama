
#ifndef IMGMAN_H
#define IMGMAN_H

#include <opencv2/opencv.hpp>
#include <tuple>
#include <iostream>
#include <fstream>

namespace imgm {


    cv::Mat resize_image( const cv::Mat& img, int target_width = 300);

    cv::Mat file_to_cv(std::string path);

    cv::Mat stitch(const cv::Mat &base, const cv::Mat &attach, const cv::Matx33f &H);


}
#endif

