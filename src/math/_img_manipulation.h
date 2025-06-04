
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
            std::vector<cv::Vec2f> im_center;

            std::vector<std::vector<int>> pair_stitch;

            const cv::Mat *adj;
            const std::vector<cv::Mat>* img_address;

            std::vector<std::vector<float>> img_dimensions;

            std::vector<Eigen::MatrixXd> rot;
            std::vector<Eigen::MatrixXd> K;

            double focal = 1000;
            std::vector<double> focal_vec;

    };


    class cylhom {

    const cv::Matx33f H;

    public:
        cylhom(const cv::Matx33f hom) :H(hom) {


        }

        std::pair<float, float> inv(float x, float y);

    };


class cylproj {

    std::vector<float> v;
    std::vector<float> v_f;
    std::vector<float> kb;
    std::vector<float> ka;
    std::vector<float> kb_inv;
    std::vector<float> ka_inv;
    float f_b;
    float f_a;
    float cx_b;
    float cy_b;
    float cx_a;
    float cy_a;
    float tx;
    float ty;
    cv::Size s;
    double thet;
    float total_theta = 2 * 3.14159;


    public:
        cylproj(Eigen::MatrixXd rot,Eigen::MatrixXd base,Eigen::MatrixXd attach,cv::Size size,float x = 0,float y = 0){

            f_b = (float)base(0,0);
            cx_b = (float)base(0,2);
            cy_b = (float)base(1,2);
            f_a = (float)attach(0,0);
            cx_a = (float)attach(0,2);
            cy_a = (float)attach(1,2);
            tx = x;
            ty = y;
            Eigen::MatrixXd base_inv = base.inverse().eval();
            Eigen::MatrixXd attach_inv = attach.inverse().eval();

            //eigen2cv(rot,R);

            s = size;

            for(int i =0;i < 3;i++){
                for(int j =0;j < 3;j++){

                    v.push_back((float)rot(j,i));
                    v_f.push_back((float)rot(i,j));
                    kb.push_back((float)base(i,j));
                    ka.push_back((float)attach(i,j));
                    kb_inv.push_back((float)base_inv(i,j));
                    ka_inv.push_back((float)attach_inv(i,j));

                }
            }


        }

        std::pair<float, float> inv(float x, float y);

        std::pair<float, float> forward(float x, float y);

    };


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



    cv::Mat file_to_cv(std::string path);

    cv::Mat stitch(const cv::Mat &base, const cv::Mat &attach, const cv::Matx33f &H);

    void calc_stitch_from_adj(pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >> &Hom,std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,std::vector<maths::keypoints> &keypnts);

    void bundleadjust_stitching(pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >> &Hom_mat,const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat);

    cv::Mat project(const cv::Mat& imags,float xc,float yc,float f,cv::Matx33f hom = cv::Matx33f::eye());

}
#endif

