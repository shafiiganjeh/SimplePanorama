
#ifndef BUNDLEM_H
#define BUNDLEM_H

#include "_bundle_adjust_tools.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <unordered_map>
#include "_homography.h"


namespace bundm {


    struct alignas(64) inter_par{

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

        std::vector<std::vector< cv::Matx33f >> hom;
        std::vector<double> focal;
        float error;

    };


    class adjuster_basic {
        public:

            virtual ~adjuster_basic() = default;
            virtual struct inter_par iterate() = 0;
            virtual std::vector<Eigen::MatrixXd> ret_rot() const = 0;
            virtual std::vector<Eigen::MatrixXd> ret_K() const = 0;

    };


    class adjuster  : public adjuster_basic {

        public:

            adjuster(const std::vector<util::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,float lmbd,const imgm::pan_img_transform &Tr,int threads = 8);

            struct inter_par iterate() override;
            std::vector<Eigen::MatrixXd> ret_rot() const override;
            std::vector<Eigen::MatrixXd> ret_K() const override;
            void set_ignore(std::vector<int> idx,bool mode);
            double error_value;
            class util::Timer timer;

        private:
            //class for calculating derivatives.
            std::shared_ptr<class bund::parameters> par_img;
            //class for calculating residual errors.
            std::shared_ptr<class bund::E_func> par_er;
            struct inter_par iter;

            Eigen::MatrixXd ret_uf(const std::vector<bund::A_vec>& avec, int total_cols);
            void calc_uf(int thread,const std::vector<bund::A_vec>& avec,std::vector<Eigen::MatrixXd>& thread_accumulators);

            std::vector<Eigen::MatrixXd> sum_transpose(const std::vector<Eigen::MatrixXd> &bvec,const std::vector<Eigen::MatrixXd> &avec);
            void sum_transposeAB_calc(int thread,const std::vector<bund::A_vec>& avec,const std::vector<Eigen::MatrixXd>& bvec,std::vector<Eigen::MatrixXd> &W);

            std::vector<Eigen::MatrixXd> sum_transposeAB(const std::vector<Eigen::MatrixXd>& bvec,const std::vector<bund::A_vec>& avec);

            Eigen::VectorXd ret_Ea(const std::vector<bund::A_vec>& avec,const std::vector<Eigen::VectorXd>& e_vec);
            void ret_Ea_calc(int thread,const std::vector<bund::A_vec>& avec,const std::vector<Eigen::VectorXd>& e_vec,std::vector<Eigen::VectorXd> &thread_Ev);

            std::vector<Eigen::VectorXd> ret_Eb(const std::vector<Eigen::MatrixXd> &bvec,const std::vector<Eigen::VectorXd> &e_vec);

            std::vector<bund::A_vec> get_iter_par();
            void augment(const std::vector<bund::A_vec> &avec);
            void augment_calc(int thread,double aug_lamda,const std::vector<bund::A_vec> &avec,struct inter_par& iter);

            void get_error(const std::vector<bund::A_vec> &avec);
            void error_calc(int thread,std::vector<Eigen::MatrixXd>& thread_accumulators_wy,std::vector<Eigen::VectorXd>& thread_accumulators_YEb,const std::vector<bund::A_vec> &avec);

            int thr;
            std::vector<int> thread_parts;
            std::vector<std::vector<int>> threads_vector;
            std::vector<int> thread_sizes;

    };

}

#endif
