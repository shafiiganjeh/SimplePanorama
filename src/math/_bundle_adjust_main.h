
#ifndef BUNDLEM_H
#define BUNDLEM_H

#include "_bundle_adjust_tools.h"


namespace bundm {

    struct inter_par{

        float lambda;

        std::vector<Eigen::VectorXf> e_vec;

        Eigen::MatrixXf u_vecf;
        std::vector<Eigen::MatrixXf> v_vec;
        std::vector<Eigen::MatrixXf> w_vec;
        Eigen::VectorXf eA_vec;
        std::vector<Eigen::VectorXf> eB_vec;

        Eigen::MatrixXf u_vecf_augmented;
        std::vector<Eigen::MatrixXf> v_vec_augmented;
        std::vector<Eigen::MatrixXf> Y_vec;

        std::vector<Eigen::VectorXf> delta_b;
        Eigen::VectorXf delta_a;

        std::vector<std::vector< cv::Matx33f >> hom;

    };

    class adjuster {

        public:

            adjuster(std::vector<cv::Matx33f> &H,const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj,float foc,float lmbd);

            struct inter_par iterate();

        private:

            std::shared_ptr<class bund::parameters> par_img;
            std::shared_ptr<class bund::E_func> par_er;
            struct inter_par iter;

            Eigen::MatrixXf ret_uf(const std::vector<Eigen::MatrixXf> &avec);

            std::vector<Eigen::MatrixXf> sum_transpose(const std::vector<Eigen::MatrixXf> &bvec,const std::vector<Eigen::MatrixXf> &avec);

            Eigen::VectorXf ret_Ea(const std::vector<Eigen::MatrixXf> &avec,const std::vector<Eigen::VectorXf> &e_vec);

            std::vector<Eigen::VectorXf> ret_Eb(const std::vector<Eigen::MatrixXf> &bvec,const std::vector<Eigen::VectorXf> &e_vec);

            void get_iter_par();
            void augment();
            void get_error();

    };



}

#endif
