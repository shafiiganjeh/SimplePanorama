
#include "_bundle_adjust_main.h"


namespace bundm {


Eigen::MatrixXf adjuster::ret_uf(const std::vector<Eigen::MatrixXf> &avec){

    Eigen::MatrixXf U;
    int c = avec[0].cols();
    U = Eigen::MatrixXf::Zero(c,c);

     for(int i = 0;i < avec.size();i++){

         U = U + avec[i].transpose()*avec[i];

     }

     return U;
}


std::vector<Eigen::MatrixXf> adjuster::sum_transpose(const std::vector<Eigen::MatrixXf> &bvec,const std::vector<Eigen::MatrixXf> &avec){

    std::vector<Eigen::MatrixXf> W;

    for(int i = 0;i < avec.size();i++){

         W.push_back(avec[i].transpose() * bvec[i]);

    }

    return W;
}


Eigen::VectorXf adjuster::ret_Ea(const std::vector<Eigen::MatrixXf> &avec,const std::vector<Eigen::VectorXf> &e_vec){

    Eigen::VectorXf Ev = Eigen::VectorXf::Zero(avec[0].cols());

    for(int i = 0;i < avec.size();i++){

        Ev = Ev + avec[i].transpose() * e_vec[i];

    }

    return Ev;
}


std::vector<Eigen::VectorXf> adjuster::ret_Eb(const std::vector<Eigen::MatrixXf> &bvec,const std::vector<Eigen::VectorXf> &e_vec){

    std::vector<Eigen::VectorXf> Ev;

    for(int i = 0;i < bvec.size();i++){

         Ev.push_back(bvec[i].transpose() * e_vec[i]);

    }

    return Ev;
}


adjuster::adjuster(std::vector<cv::Matx33f> &H,const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj,float foc,float lmbd){

    par_er = std::make_shared<class bund::E_func>(kp,match,adj);
    par_img = std::make_shared<class bund::parameters>(H,kp,match,adj,foc);
    iter.lambda = lmbd;

}


void adjuster::get_iter_par(){

    std::vector<Eigen::MatrixXf> avec = par_img -> ret_A_i();

    iter.u_vecf = ret_uf(avec);

    std::vector<Eigen::MatrixXf> bvec = par_img -> ret_B_i();
    iter.v_vec = sum_transpose(bvec,bvec);

    iter.w_vec = sum_transpose(bvec,avec);

    std::vector<Eigen::VectorXf> ms = par_img -> ret_measurements();
    iter.e_vec = par_er -> error(ms);
    iter.eA_vec = ret_Ea(avec,iter.e_vec);

    iter.eB_vec = ret_Eb(bvec,iter.e_vec);
}


void adjuster::augment(){

    iter.u_vecf_augmented = iter.u_vecf;
    iter.u_vecf_augmented.diagonal() = iter.u_vecf_augmented.diagonal()*(1 + iter.lambda);

    iter.v_vec_augmented = iter.v_vec;
    for (int p = 0;p < iter.v_vec_augmented.size();p++){

        iter.v_vec_augmented[p].diagonal() = iter.v_vec_augmented[p].diagonal()*(1 + iter.lambda);
        iter.v_vec_augmented[p] = iter.v_vec_augmented[p].inverse().eval();
        iter.Y_vec.push_back(iter.w_vec[p]*iter.v_vec_augmented[p]);

    }

}


void adjuster::get_error(){

    Eigen::MatrixXf sum_wy = Eigen::MatrixXf::Zero(iter.Y_vec[0].rows(),iter.w_vec[0].rows());
    Eigen::VectorXf sum_YEb = Eigen::VectorXf::Zero(iter.Y_vec[0].rows());
    Eigen::MatrixXf uvecf = iter.u_vecf_augmented;

    for(int i = 0;i < iter.w_vec.size();i++){

        sum_wy = sum_wy + iter.Y_vec[i]*iter.w_vec[i].transpose();
        sum_YEb = sum_YEb + iter.Y_vec[i] * iter.eB_vec[i];

    }

    uvecf = uvecf - sum_wy;

    iter.delta_a = uvecf.colPivHouseholderQr().solve(iter.eA_vec - sum_YEb);

    for (int p = 0;p < iter.v_vec_augmented.size();p++){

        iter.delta_b.push_back( iter.v_vec_augmented[p] * (iter.eB_vec[p] - iter.w_vec[p].transpose() * iter.delta_a) );

    }

}


struct inter_par adjuster::iterate(){

    float error_start;
    float error_new;
    float lambda = iter.lambda;

iter.hom = par_img -> ret_hmat();
return iter;


    int break_counter = 0;
    for (int it = 0;it<100;it++){

        get_iter_par();
        augment();
        get_error();

        error_start = 0;
        for (Eigen::VectorXf e : iter.e_vec){

            error_start = error_start + e.norm();

        }

        par_img -> add_delta(iter.delta_b,iter.delta_a);

        std::vector<Eigen::VectorXf> ms = par_img -> ret_measurements();
        std::vector<Eigen::VectorXf> new_error = par_er -> error(ms);

        error_new = 0;
        for (Eigen::VectorXf e : new_error){

            error_new = error_new + e.norm();

        }

        if( error_start > error_new ){

            iter.lambda = iter.lambda / 10;
            std::cout <<"lambda: "<< iter.lambda<< " new error: " << error_new<<"\n";
            error_start = error_new;
            break_counter = 0;

        }else{

            iter.lambda = iter.lambda * 10;
            std::cout <<"lambda: "<< iter.lambda<< " old error: " << error_start<<"\n";
            par_img -> reset();
            break_counter++;
        }

        if (break_counter > 5){ break;};

        lambda = iter.lambda;
        iter = inter_par();
        iter.lambda = lambda;

    }

    iter.hom = par_img -> ret_hmat();

    return iter;

}


}


