
#ifndef IMGMAN_H
#define IMGMAN_H

#include <opencv2/opencv.hpp>
#include <tuple>
#include <iostream>
#include <fstream>
#include "_util.h"
#include "_homography.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unsupported/Eigen/MatrixFunctions>


namespace imgm {

    enum Operation { MULTIPLY, DIVIDE };
    void elementwiseOperation(const cv::Mat& A, const cv::Mat& B, cv::Mat& result, Operation op = MULTIPLY);

    class pan_img_transform {

        public:

            pan_img_transform(const cv::Mat* adj_mat,const std::vector<cv::Mat> *imags);
            pan_img_transform(const struct util::adj_str* adj_mat,const std::vector<cv::Mat> *imags) : pan_img_transform(&(adj_mat->adj),imags){ connectivity = adj_mat->connectivity;};

            std::vector<cv::Matx33f> H_1j;
            std::vector<cv::Matx33f> img2pan;
            std::vector<cv::Matx33f> pan2img;
            std::vector<int> stitch_order;
            std::vector<struct util::translation> translation;
            cv::Size flat_pan_dim;

            std::vector<std::vector<int>> pair_stitch;

            const cv::Mat *adj;
            const std::vector<cv::Mat>* img_address;
            std::vector<std::vector<int>> img_dimensions;

            std::vector<Eigen::MatrixXd> rot;
            std::vector<Eigen::MatrixXd> K;

            std::vector<double> connectivity;

            double focal = 1000;

    };

    cv::Mat multiplyChannels(const cv::Mat& A, const cv::Mat& B);

    cv::Mat resize_image( const cv::Mat& img, int target_width = 300);

    cv::Mat resizeKeepAspectRatio(const cv::Mat& input, int desiredWidth);


    template <typename T> cv::Mat applyGeometricTransform(const cv::Mat& img, T& transformer,const cv::Size size) {

        cv::Mat map_x(size, CV_32FC1);
        cv::Mat map_y(size, CV_32FC1);

        for (int y = 0; y < size.height; ++y) {
            for (int x = 0; x < size.width; ++x) {
                auto [src_x, src_y] = transformer.inv(x, y);

                // Handle invalid coordinates
                if (src_x < 0 || src_y < 0) {
                    map_x.at<float>(y, x) = -1;
                    map_y.at<float>(y, x) = -1;
                } else {
                    map_x.at<float>(y, x) = src_x;
                    map_y.at<float>(y, x) = src_y;
                }
            }
        }

        cv::Mat result;
        cv::remap(img, result, map_x, map_y, cv::INTER_LINEAR,
                cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        return result;
    }

    cv::Mat file_to_cv(std::string &path);

    cv::Mat stitch(const cv::Mat &base, const cv::Mat &attach, const cv::Matx33f &H);

    void calc_stitch_from_adj(pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >> &Hom,std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,std::vector<util::keypoints> &keypnts);

    void bundleadjust_stitching(pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >> &Hom_mat,const std::vector<util::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat);

    cv::Mat project(const cv::Mat& imags,float xc,float yc,float f,cv::Matx33f hom = cv::Matx33f::eye());

}
#endif

