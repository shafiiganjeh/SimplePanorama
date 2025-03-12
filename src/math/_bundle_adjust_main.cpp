
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


adjuster::adjuster(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj,float lmbd,const imgm::pan_img_transform &Tr){

    par_er = std::make_shared<class bund::E_func>(kp,match,adj);
    par_img = std::make_shared<class bund::parameters>(kp,match,Tr);
    iter.lambda = lmbd;
    iter.hom = par_img -> ret_hmat();

}

void printvec(const std::vector<Eigen::VectorXd> &vec){

    std::cout<<"\n" << "err vec: ";

    for(int i = 0;i<vec.size();i++){

        std::cout << " :"<<vec[i]<< ": "<<"\n";

    }
    std::cout<<"\n" << "err end "<<"\n";
}

void adjuster::get_iter_par(){

    std::vector<Eigen::MatrixXd> avec = par_img -> ret_A_i();

    iter.u_vecf = ret_uf(avec);

    std::vector<Eigen::MatrixXd> bvec = par_img -> ret_B_i();
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
    //iter.u_vecf_augmented.diagonal() = iter.u_vecf_augmented.diagonal()*(1 + iter.lambda);

    std::vector<double> foc_vec = par_img->ret_focal();
    double ang_ = (3.141/16);

    for(int i = 0; i < (par_img->adj).rows;i++){

        Eigen::VectorXd augmvec(6);
        augmvec <<foc_vec[i]/10,1,1,ang_,ang_,ang_;

        for(int j = 0 ; j < 6;j++){

            iter.u_vecf_augmented.diagonal()[6*i + j] = iter.u_vecf_augmented.diagonal()[6*i + j]*(1 + iter.lambda * augmvec[j]);

        }

    }

    iter.v_vec_augmented = iter.v_vec;
    for (int p = 0;p < iter.v_vec_augmented.size();p++){

        iter.v_vec_augmented[p].diagonal() = iter.v_vec_augmented[p].diagonal()*(1 + iter.lambda );
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

    bool addrot = true;



    int break_counter = 0;
    for (int it = 0;it<50;it++){

        get_iter_par();
        augment();
        get_error();


        error_start = 0;
        for (Eigen::VectorXd e : iter.e_vec){

            error_start = error_start + e.norm();

        }
        //std::cout <<"error "<<error_start<<"\n";


        par_img -> add_delta(iter.delta_b,iter.delta_a,addrot);

        std::vector<Eigen::VectorXd> ms = par_img -> ret_measurements();
        std::vector<Eigen::VectorXd> new_error = par_er -> error(ms);

        error_new = 0;
        for (Eigen::VectorXd e : new_error){

            error_new = error_new + e.norm();

        }

        //std::cout<<"\n" <<"test error: " << error_new<<"\n";
        if( error_start > error_new ){

            iter.lambda = iter.lambda / 10;
            std::cout <<"lambda: "<< iter.lambda<< " new error: " << error_new<<"\n";
            error_start = error_new;
            break_counter = 0;

        }else{

            iter.lambda = iter.lambda * 10;
            //std::cout <<"lambda: "<< iter.lambda<< " old error: " << error_start<<"\n";
            par_img -> reset();
            break_counter++;
        }
        //std::cout <<"\n"<<"ddrot: "<< addrot<<"\n";
        if (break_counter > 5){
            if(addrot == false){
                addrot = true;
                break_counter = 0;
                std::cout <<"\n"<<"set addrot: "<< iter.lambda<< "error: " << error_new<<"\n";
                iter.lambda = 0.0001;
            }else{
                break;

            }

        };

        lambda = iter.lambda;
        iter = inter_par();
        iter.lambda = lambda;

    }

    iter.focal = par_img -> ret_focal();
    iter.hom = par_img -> ret_hmat();
    return iter;

}


std::vector<Eigen::MatrixXd> adjuster::ret_K(){

    return par_img -> ret_K();

}

std::vector<Eigen::MatrixXd> adjuster::ret_rot(){

    return par_img -> ret_rot();

}



}


