
#ifndef BUNDLEF_H
#define BUNDLEF_H

#include "_bundle_adjust_tools.h"
#include "_bundle_adjust_main.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <unordered_map>
#include "_homography.h"


namespace bundf {



    class adjuster  : public bundm::adjuster_basic  {

        public:

            adjuster(const std::vector<util::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,float lmbd,const imgm::pan_img_transform &Tr,int threads = 8);

            double error_value;

            struct bundm::inter_par iterate() override;
            std::vector<Eigen::MatrixXd> ret_rot() const override;
            std::vector<Eigen::MatrixXd> ret_K() const override;


        private:

            struct bundm::inter_par iter;
            //class for calculating derivatives.
            std::shared_ptr<class bund::parameters> par_img;
            //class for calculating residual errors.
            std::shared_ptr<class bund::E_func> par_er;

            std::vector<bund::A_vec> get_iter_par();

            void calc_uf(int thread,const std::vector<bund::A_vec>& avec,std::vector<Eigen::MatrixXd>& thread_accumulators);
            Eigen::MatrixXd ret_uf(const std::vector<bund::A_vec>& avec, int total_cols);
            void ret_Ea_calc(int thread,const std::vector<bund::A_vec>& avec,const std::vector<Eigen::VectorXd>& e_vec,std::vector<Eigen::VectorXd> &thread_Ev);
            Eigen::VectorXd ret_Ea(const std::vector<bund::A_vec>& avec,const std::vector<Eigen::VectorXd>& e_vec);
            void augment(const std::vector<bund::A_vec> &avec);
            void get_error(const std::vector<bund::A_vec> &avec);

            int thr;
            std::vector<int> thread_parts;
            std::vector<std::vector<int>> threads_vector;
            std::vector<int> thread_sizes;


    };

}

#endif
