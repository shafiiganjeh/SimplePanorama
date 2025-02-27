
#include "_bundle_adjust_main.h"


namespace bundm {


Eigen::MatrixXd adjuster::ret_uf(const std::vector<Eigen::MatrixXd> &avec){

    Eigen::MatrixXd U;
    int c = avec[0].cols();
    U = Eigen::MatrixXd::Zero(c,c);

     for(int i = 0;i < avec.size();i++){

         U = U + avec[i].transpose()*avec[i];

     }

     return U;
}


std::vector<Eigen::MatrixXd> adjuster::sum_transpose(const std::vector<Eigen::MatrixXd> &bvec,const std::vector<Eigen::MatrixXd> &avec){

    std::vector<Eigen::MatrixXd> W;

    for(int i = 0;i < avec.size();i++){

         W.push_back(avec[i].transpose() * bvec[i]);

    }

    return W;
}


Eigen::VectorXd adjuster::ret_Ea(const std::vector<Eigen::MatrixXd> &avec,const std::vector<Eigen::VectorXd> &e_vec){

    Eigen::VectorXd Ev = Eigen::VectorXd::Zero(avec[0].cols());

    for(int i = 0;i < avec.size();i++){

        Ev = Ev + avec[i].transpose() * e_vec[i];

    }

    return Ev;
}


std::vector<Eigen::VectorXd> adjuster::ret_Eb(const std::vector<Eigen::MatrixXd> &bvec,const std::vector<Eigen::VectorXd> &e_vec){

    std::vector<Eigen::VectorXd> Ev;

    for(int i = 0;i < bvec.size();i++){

         Ev.push_back(bvec[i].transpose() * e_vec[i]);

    }

    return Ev;
}


adjuster::adjuster(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj,float foc,float lmbd,const std::vector<std::vector< cv::Matx33f >> &hom_mat,const imgm::pan_img_transform &Tr){

    par_er = std::make_shared<class bund::E_func>(kp,match,adj);
    par_img = std::make_shared<class bund::parameters>(kp,match,Tr);
    iter.lambda = lmbd;
    iter.hom = par_img -> ret_hmat();

}


void adjuster::get_iter_par(){

    std::vector<Eigen::MatrixXd> avec = par_img -> ret_A_i_num();

    iter.u_vecf = ret_uf(avec);

    std::vector<Eigen::MatrixXd> bvec = par_img -> ret_B_i_num();
    iter.v_vec = sum_transpose(bvec,bvec);

    iter.w_vec = sum_transpose(bvec,avec);

    std::vector<Eigen::VectorXd> ms = par_img -> ret_measurements();
    //struct bund::approx normalizer = par_img -> ret_all();

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

    Eigen::MatrixXd sum_wy = Eigen::MatrixXd::Zero(iter.Y_vec[0].rows(),iter.w_vec[0].rows());
    Eigen::VectorXd sum_YEb = Eigen::VectorXd::Zero(iter.Y_vec[0].rows());
    Eigen::MatrixXd uvecf = iter.u_vecf_augmented;

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

    std::vector<Eigen::VectorXd> ms = par_img -> ret_measurements();
    //struct bund::approx normalizer = par_img -> ret_all();

    std::vector<Eigen::VectorXd> new_error = par_er -> error(ms);





    int break_counter = 0;
    for (int it = 0;it<20;it++){

        get_iter_par();
        augment();
        get_error();

        error_start = 0;
        for (Eigen::VectorXd e : iter.e_vec){

            error_start = error_start + e.norm();

        }
        std::cout <<"error "<<error_start<<"\n";

        par_img -> add_delta(iter.delta_b,iter.delta_a);

        std::vector<Eigen::VectorXd> ms = par_img -> ret_measurements();
        //struct bund::approx normalizer = par_img -> ret_all();
        std::vector<Eigen::VectorXd> new_error = par_er -> error(ms);

        error_new = 0;
        for (Eigen::VectorXd e : new_error){

            error_new = error_new + e.norm();

        }

        std::cout<<"\n" <<"test error: " << error_new<<"\n";
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

/*
std::vector<std::vector< cv::Matx33d >> adjuster::ret_k(){

    std::vector<std::vector< cv::Matx33d >> k;
    k = par_img -> ret_kmat();

    return k;
}
*/

}


