
#ifndef IMGMAN_H
#define IMGMAN_H

#include <opencv2/opencv.hpp>
#include <tuple>
#include <iostream>
#include <fstream>
#include "_maths.h"
#include <Eigen/Dense>

namespace imgm {


    class pan_img_transform {

        public:

            pan_img_transform(const cv::Mat& adj);

            std::vector<cv::Matx33f> img2pan;
            std::vector<cv::Matx33f> pan2img;
            std::vector<int> stitch_order;
            std::vector<struct maths::translation> translation;
            std::pair<int,int> pan_dim;


    };


    cv::Mat resize_image( const cv::Mat& img, int target_width = 300);

    cv::Mat file_to_cv(std::string path);

    cv::Mat stitch(const cv::Mat &base, const cv::Mat &attach, const cv::Matx33f &H,std::vector<cv::Vec3f> p);

    class pan_img_transform calc_stitch_from_adj(const std::vector<cv::Mat> &imags,const std::vector<std::vector< cv::Matx33f >> &Hom,const cv::Mat& adj);

    cv::Mat project(const cv::Mat& imags,float xc,float yc,float f);

}
#endif

