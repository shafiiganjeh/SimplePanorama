
#ifndef BUNDLEM_H
#define BUNDLEM_H

#include "_bundle_adjust_tools.h"


namespace bundm {

    struct inter_par{

        float lambda;

        std::vector<Eigen::VectorXd> e_vec;

        Eigen::MatrixXd u_vecf;
        std::vector<Eigen::MatrixXd> v_vec;
        std::vector<Eigen::MatrixXd> w_vec;
        Eigen::VectorXd eA_vec;
        std::vector<Eigen::VectorXd> eB_vec;

        Eigen::MatrixXd u_vecf_augmented;
        std::vector<Eigen::MatrixXd> v_vec_augmented;
        std::vector<Eigen::MatrixXd> Y_vec;

        std::vector<Eigen::VectorXd> delta_b;
        Eigen::VectorXd delta_a;

        std::vector<std::vector< cv::Matx33d >> hom;

    };

    class adjuster {

        public:

            adjuster(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj,float foc,float lmbd,const std::vector<std::vector< cv::Matx33f >> &hom_mat,const imgm::pan_img_transform &Tr);

            struct inter_par iterate();
            std::vector<std::vector< cv::Matx33d >> ret_k();

        private:

            std::shared_ptr<class bund::parameters> par_img;
            std::shared_ptr<class bund::E_func> par_er;
            struct inter_par iter;

            Eigen::MatrixXd ret_uf(const std::vector<Eigen::MatrixXd> &avec);

            std::vector<Eigen::MatrixXd> sum_transpose(const std::vector<Eigen::MatrixXd> &bvec,const std::vector<Eigen::MatrixXd> &avec);

            Eigen::VectorXd ret_Ea(const std::vector<Eigen::MatrixXd> &avec,const std::vector<Eigen::VectorXd> &e_vec);

            std::vector<Eigen::VectorXd> ret_Eb(const std::vector<Eigen::MatrixXd> &bvec,const std::vector<Eigen::VectorXd> &e_vec);

            void get_iter_par();
            void augment();
            void get_error();

    };

}

#endif
