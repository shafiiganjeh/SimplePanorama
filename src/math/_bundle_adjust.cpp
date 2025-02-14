#include "_bundle_adjust.h"

namespace bund {

using Eigen::MatrixXf;
using Eigen::VectorXf;

void write_to_eigen(Eigen::VectorXf &v,const cv::Vec2f &cv_v,int n,int st){

    for(int i = 0;i < n;i++){

        v[st+i] = cv_v[i];

    }

}


Eigen::MatrixXf cov_mat_XY(Eigen::MatrixXf X,Eigen::MatrixXf Y){

    Eigen::MatrixXf con_XY;

    if((X.rows() != Y.rows())){

        throw std::invalid_argument("Error: random vector X and Y have different sizes.");

    }

    Eigen::MatrixXf Xv;
    Eigen::MatrixXf Yv;

    Xv = (X.colwise() - X.rowwise().mean());
    Yv = (Y.colwise() - Y.rowwise().mean());

    con_XY = (Xv * Yv.transpose())/X.rows();

    return con_XY;

}

E_func::E_func(const std::vector<maths::keypoints> &kp,std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj){

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                //std::cout<<"size: " << match[i][j].size()<<"\n";
                for (int p = 0;p < match[i][j].size();p++){

                    Eigen::VectorXf measure(4);
                    std::vector<int> push_idx = {p,i,j};
                    index_set.push_back(push_idx);
                    write_to_eigen(measure, kp[i].keypoint[match[i][j][p].queryIdx].pt,2,2);
                    write_to_eigen(measure, kp[j].keypoint[match[i][j][p].trainIdx].pt,2,0);
                    measurements.push_back(measure);

                }
            }
        }
    }
}


std::vector<Eigen::VectorXf> E_func::get_measurements(){

    return measurements;

}


std::vector<Eigen::VectorXf> E_func::error(std::vector<Eigen::VectorXf> &t_to_q){

    std::vector<Eigen::VectorXf> error_vec;

    for (int i = 0;i < measurements.size();i++){

        error_vec.push_back(measurements[i] - t_to_q[i]);

    }

    return error_vec;

}


cov_vec::cov_vec(const std::vector<Eigen::VectorXf> &measurements){

    for (int i = 0;i < measurements.size();i++){

        Eigen::MatrixXf matC(2, 2);
        matC.col(0) = measurements[i]({0,1});
        matC.col(1) = measurements[i]({2,3});

        cov.push_back(cov_mat_XY(matC,matC));

    }

}


Eigen::MatrixXf parameters::get_rot(int i){

    Eigen::MatrixXf v(3,3);
    v << 0,-rot[i][3],rot[i][2],rot[i][3],0,-rot[i][1],-rot[i][2],rot[i][1],0;
    Eigen::MatrixXf ex = v.exp();
    return ex;

}


Eigen::MatrixXf parameters::get_foc(bool inv){

    Eigen::MatrixXf v(3,3);
    if(inv){

        v << 1/focal,0,0,0,1/focal,0,0,0,1;
        return v;

    }else{

        v << focal,0,0,0,focal,0,0,0,1;
        return v;

    }

}


Eigen::MatrixXf parameters::ret_hom(int i, int j){

    Eigen::MatrixXf foc_ = get_foc(false);
    Eigen::MatrixXf foc_inv = get_foc(true);
    Eigen::MatrixXf rot_i = get_rot(i);
    Eigen::MatrixXf rot_j = get_rot(j);

    Eigen::MatrixXf Hom = foc_inv * rot_i * rot_j.transpose() * foc_;
    return Hom;
}


}




