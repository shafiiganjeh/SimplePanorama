
#ifndef BUNDLE_H
#define BUNDLE_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "_maths.h"
#include <unsupported/Eigen/MatrixFunctions>

namespace bund {

    Eigen::MatrixXf cov_mat_XY(Eigen::MatrixXf X,Eigen::MatrixXf Y);


    class E_func {

        public:

            E_func(const std::vector<maths::keypoints> &kp,std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj);
            std::vector<Eigen::VectorXf> error(std::vector<Eigen::VectorXf> &t_to_q);
            std::vector<Eigen::VectorXf> get_measurements();

        protected:

            std::vector<Eigen::VectorXf> measurements;
            std::vector<std::vector<int>> index_set;

    };


    class cov_vec {

        public:

            cov_vec(const std::vector<Eigen::VectorXf> &measurements);
            std::vector<Eigen::MatrixXf> cov;

    };



    class parameters {

        public:

            Eigen::MatrixXf ret_hom(int i, int j);

        private:

            Eigen::MatrixXf get_rot(int i);
            Eigen::MatrixXf get_foc(bool inv);

            float focal;
            std::vector<Eigen::Vector3f> rot;

    };

}

#endif

