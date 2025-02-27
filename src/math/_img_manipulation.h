
#ifndef IMGMAN_H
#define IMGMAN_H

#include <opencv2/opencv.hpp>
#include <tuple>
#include <iostream>
#include <fstream>
#include "_maths.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>

namespace imgm {


    class pan_img_transform {

        public:

            pan_img_transform(const cv::Mat* adj_mat,const std::vector<cv::Mat> *imags);

            std::vector<cv::Matx33f> H_1j;
            std::vector<cv::Matx33f> img2pan;
            std::vector<cv::Matx33f> pan2img;
            std::vector<int> stitch_order;
            std::vector<struct maths::translation> translation;
            std::pair<int,int> pan_dim;

            const cv::Mat *adj;
            const std::vector<cv::Mat> *img_address;

            std::vector<std::vector<float>> img_dimensions;

            std::vector<Eigen::MatrixXd> rot;
            double focal = 1000;

    };


    cv::Mat resize_image( const cv::Mat& img, int target_width = 300);

    cv::Mat file_to_cv(std::string path);

    cv::Mat stitch(const cv::Mat &base, const cv::Mat &attach, const cv::Matx33f &H);

    void calc_stitch_from_adj(pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >> &Hom);

    cv::Mat project(const cv::Mat& imags,float xc,float yc,float f);

}
#endif

