
#ifndef BUNDLE_H
#define BUNDLE_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "_maths.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <opencv2/core/eigen.hpp>


namespace bund {


    class parameters {

        public:

            parameters(std::vector<cv::Matx33f> &H,const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj,float foc);
            Eigen::MatrixXf ret_hom(int i, int j);
            std::vector<Eigen::MatrixXf> ret_B_i();
            std::vector<Eigen::MatrixXf> ret_A_i();
            std::vector<Eigen::VectorXf> ret_measurements();
            std::vector<std::vector< cv::Matx33f >> ret_hmat();
            void add_delta(std::vector<Eigen::VectorXf> delta_b,Eigen::VectorXf delta_a);
            void reset();

        private:

            Eigen::MatrixXf get_rot(int i);
            Eigen::MatrixXf get_foc(bool inv);

            std::vector<std::vector<std::vector<Eigen::VectorXf>>> measurements;
            std::vector<std::vector<std::vector<Eigen::VectorXf>>> measurements_res;

            std::vector<std::vector<int>> idx_set;
            float focal;
            float focal_res;

            std::vector<Eigen::Vector3f> rot;
            std::vector<Eigen::Vector3f> rot_res;

            cv::Mat adj;

    };


    class E_func {

        public:

            E_func(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj);
            std::vector<Eigen::VectorXf> error(const std::vector<Eigen::VectorXf> &t_to_q);
            std::vector<Eigen::VectorXf> get_measurements();
            std::vector<std::vector<int>> ret_idx_set();

        private:

            std::vector<Eigen::VectorXf> measurements;
            std::vector<std::vector<int>> idx_set;

    };



}

#endif

