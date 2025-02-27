
#ifndef BUNDLE_H
#define BUNDLE_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "_maths.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <opencv2/core/eigen.hpp>
#include "_img_manipulation.h"


namespace bund {

    struct approx{

    std::vector<Eigen::Matrix3d> K;
    std::vector<Eigen::Matrix3d> K_inv;
    std::vector<Eigen::Matrix3d> R;
    std::vector<Eigen::MatrixXd> R_v;
    std::vector<std::vector<Eigen::MatrixXd>> normalizerTi;
    std::vector<std::vector<Eigen::MatrixXd>> normalizerTj;

};


    class parameters {

        public:

            parameters(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj,float foc,const std::vector<std::vector< cv::Matx33f >> &hom_mat,const imgm::pan_img_transform &Tr);
            Eigen::MatrixXd ret_hom(int i, int j);
            std::vector<Eigen::MatrixXd> ret_B_i();
            std::vector<Eigen::MatrixXd> ret_B_i_num();
            std::vector<Eigen::MatrixXd> ret_A_i();
            std::vector<Eigen::MatrixXd> ret_A_i_num();
            std::vector<Eigen::VectorXd> ret_measurements();
            std::vector<std::vector< cv::Matx33d >> ret_hmat();
            std::vector<std::vector< cv::Matx33d >> ret_kmat();
            struct approx ret_all();
            void add_delta(std::vector<Eigen::VectorXd> delta_b,Eigen::VectorXd delta_a);
            void reset();

        private:

            Eigen::MatrixXf get_rot(int i);
            Eigen::MatrixXf get_foc(bool inv);

            std::vector<std::vector<std::vector<Eigen::VectorXd>>> measurements;
            std::vector<std::vector<std::vector<Eigen::VectorXd>>> measurements_res;

            std::vector<std::vector<int>> idx_set;

            struct approx R_approx;
            struct approx R_approx_res;

            Eigen::Matrix3d R_id = Eigen::Matrix3d::Identity();

            cv::Mat adj;

    };


    class E_func {

        public:

            E_func(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj);
            std::vector<Eigen::VectorXd> error(const std::vector<Eigen::VectorXd> &t_to_q,const struct approx &normalize);
            std::vector<Eigen::VectorXd> get_measurements();
            std::vector<std::vector<int>> ret_idx_set();

        private:

            std::vector<Eigen::VectorXd> measurements;
            std::vector<std::vector<int>> idx_set;

    };



}

#endif

