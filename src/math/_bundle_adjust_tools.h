
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

    struct A_vec{

        int idx_i;
        Eigen::MatrixXd Ai;
        int idx_j;
        Eigen::MatrixXd Aj;
        int size;

    };


    class parameters {

        public:

            parameters(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const class imgm::pan_img_transform &T,int threads = 8);
            std::vector<Eigen::MatrixXd> ret_B_i();
            std::vector<Eigen::MatrixXd> ret_B_i_num();
            std::vector<A_vec> ret_A_i();
            std::vector<A_vec> ret_A_i_num();
            std::vector<Eigen::VectorXd> ret_measurements();
            std::vector<std::vector< cv::Matx33f >> ret_hmat();
            std::vector<double> ret_focal();
            Eigen::MatrixXd ret_hom(int i, int j);
            std::vector<Eigen::MatrixXd> ret_rot();
            std::vector<Eigen::MatrixXd> ret_K();
            void add_delta(std::vector<Eigen::VectorXd> delta_b,Eigen::VectorXd delta_a,bool add_rot);
            void reset();
            std::vector<bool> adjust_yn;

            cv::Mat adj;

        private:

            std::vector<std::vector<std::vector<Eigen::VectorXd>>> measurements;
            std::vector<std::vector<std::vector<Eigen::VectorXd>>> measurements_res;

            std::vector<std::vector<int>> idx_set;

            std::vector<double> focal;
            std::vector<double> focal_res;

            std::vector<Eigen::MatrixXd> rot;
            std::vector<Eigen::MatrixXd> rot_res;
            std::vector<Eigen::MatrixXd> rot_vec;
            std::vector<Eigen::MatrixXd> rot_vec_res;
            std::vector<Eigen::Vector2d> principal_vec;
            std::vector<Eigen::Vector2d> principal_vec_res;

            std::vector<Eigen::MatrixXd> K;
            std::vector<Eigen::MatrixXd> K_inv;
            std::vector<Eigen::MatrixXd> K_inv_res;
            std::vector<Eigen::MatrixXd> K_res;

            std::vector<std::vector<int>> thread_parts;
            std::vector<std::vector<std::vector<int>>> threads_vector;
            std::vector<int> thread_sizes;

            void calc_B_i(int thread,const std::vector<std::vector<Eigen::MatrixXd>> hom_mat,std::vector<Eigen::MatrixXd> &B_i);
            void calc_A_i(int thread,int size,const std::vector<std::vector<Eigen::MatrixXd>> hom_mat,std::vector<Eigen::Matrix3d> D,std::vector<A_vec> &A_i);
            void calc_A_i_num(int thread,int size,std::vector<A_vec> &A_i);

    };


    class E_func {

        public:

            E_func(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adjm);
            std::vector<Eigen::VectorXd> error(const std::vector<Eigen::VectorXd> &t_to_q);
            std::vector<Eigen::VectorXd> get_measurements();
            std::vector<std::vector<int>> ret_idx_set();

        private:

            std::vector<Eigen::VectorXd> measurements;
            std::vector<std::vector<int>> idx_set;

    };



}

#endif
