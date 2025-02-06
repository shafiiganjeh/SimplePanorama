
#ifndef IMGMAN_H
#define IMGMAN_H

#include <opencv2/opencv.hpp>
#include <tuple>
#include <iostream>
#include <fstream>
#include "_maths.h"
#include <Eigen/Dense>

namespace imgm {


    cv::Mat resize_image( const cv::Mat& img, int target_width = 300);

    cv::Mat file_to_cv(std::string path);

    cv::Mat stitch(const cv::Mat &base, const cv::Mat &attach, const cv::Matx33f &H);

    void stitch_adj(const std::vector<cv::Mat> &imags,const std::vector<std::vector< cv::Matx33f >> &Hom,const cv::Mat& adj);

    cv::Mat project(const cv::Mat& imags,float xc,float yc);

}
#endif

