#include "_bundle_adjust.h"

namespace bund {

using Eigen::MatrixXf;
using Eigen::VectorXf;


Eigen::VectorXf findRealColumns(const Eigen::Matrix3f & H) {
    float cnst = 1e-4;
    std::vector<int> real_cols;
    Eigen::EigenSolver<Eigen::Matrix3f> M(H);

    for (int i = 0; i < M.eigenvectors().cols(); ++i) {

        if ( cnst > M.eigenvectors().col(i).imag().squaredNorm() ) {

            real_cols.push_back(i);
        }
    }

    Eigen::Index   minIndex;
    Eigen::VectorXcf ONE = Eigen::VectorXcf::Ones(real_cols.size());
    ONE = ONE - M.eigenvalues()(real_cols);

    for (int i = 0;i < ONE.size();i++){

        ONE[i] = ONE[i] * ONE[i];

    }

    ONE.real().minCoeff(&minIndex);

    return M.eigenvectors().col(real_cols[minIndex]).real();

}


float get_approx(const Eigen::VectorXf &rot,const Eigen::Matrix3f & H,int searches,int steps){


    float step_size = 4 * M_PI / steps;
    float min = 0;

    for (int i = 0;i < searches;i++){

        std::vector<float> norms;
        std::vector<float> V(steps);
        std::iota( V.begin(), V.end(), step_size );

        for(float& d : V)
            d += (min - .5*step_size*steps);

        for(float& d : V){

            Eigen::MatrixXf v(3,3);
            v<< 0,-rot[2]*d,rot[1]*d,rot[2]*d,0,-rot[0]*d,-rot[1]*d,rot[0]*d,0;
            float asd = (H -  v.exp() ).norm();
            norms.push_back(asd);
            //std::cout<<"norm: " <<asd<<"\n";
        }

        std::vector<float>::iterator result = std::min_element(norms.begin(), norms.end());
        //std::cout << "index of smallest element: " << std::distance(std::begin(norms), result);

        step_size = step_size/steps;
        min = V[std::distance(std::begin(norms), result)];
        //std::cout<<"final norm: " <<norms[std::distance(std::begin(norms), result)]<<"\n";

    }

    return min;

}


float grad(const Eigen::VectorXf &rot,const Eigen::Matrix3f & H,float angle){

    Eigen::MatrixXf v(3,3);
    v<< 0,-rot[2],rot[1],rot[2],0,-rot[0],-rot[1],rot[0],0;

    Eigen::MatrixXf v_angle(3,3);
    v_angle<< 0,angle*rot[2],-angle*rot[1],-angle*rot[2],0,angle*rot[0],angle*rot[1],-angle*rot[0],0;

    float grd = 2* ( v * H *  v_angle.exp()).trace();

    return grd;
}


float norm_fkt(const Eigen::VectorXf &rot,const Eigen::Matrix3f & H,float d){

    Eigen::MatrixXf v(3,3);
    v<< 0,-rot[2]*d,rot[1]*d,rot[2]*d,0,-rot[0]*d,-rot[1]*d,rot[0]*d,0;
    float nrm = (H -  v.exp() ).norm();
    return nrm;
}

float gdsc(const Eigen::VectorXf &rot,const Eigen::Matrix3f & H,float step_size,float start,int steps){

    float opt = start;
    float norm;

    for (int i = 0;i<steps;i++){
        norm = norm_fkt(rot,H,opt);

        float step = grad(rot,H,opt);
        step = step * step_size;
        opt = opt - step;

    }

    return opt;

}


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
                    std::vector<int> idx(2);
                    idx[0] = i;
                    idx[1] = j;

                    Eigen::VectorXf measure = Eigen::VectorXf::Zero(2*adj.cols);
                    write_to_eigen(measure, kp[i].keypoint[match[i][j][p].queryIdx].pt,2,j*2);
                    write_to_eigen(measure, kp[j].keypoint[match[i][j][p].trainIdx].pt,2,i*2);
                    measurements.push_back(measure);
                    idx_set.push_back(idx);

                }
            }
        }
    }
}


std::vector<Eigen::VectorXf> E_func::get_measurements(){

    return measurements;

}

std::vector<std::vector<int>> E_func::ret_idx_set(){

    return idx_set;
}


std::vector<Eigen::VectorXf> E_func::error(const std::vector<Eigen::VectorXf> &t_to_q){

    std::cout <<"size: "<< measurements[0].size();

    std::vector<Eigen::VectorXf> error_vec;

    for (int n = 0;n < measurements.size();n++){
        Eigen::VectorXf er = Eigen::VectorXf::Zero(measurements[0].size());
        int i = idx_set[n][0];
        int j = idx_set[n][1];

        er({2*j,1+2*j}) = t_to_q[n]({3*j,1+3*j})/t_to_q[n][2+3*j];
        er({2*i,1+2*i}) = t_to_q[n]({3*i,1+3*i})/t_to_q[n][2+3*i];

        error_vec.push_back(measurements[n] - er);

    }

    return error_vec;

}



Eigen::MatrixXf parameters::get_rot(int i){

    Eigen::MatrixXf v(3,3);
    v << 0,-rot[i][2],rot[i][1],rot[i][2],0,-rot[i][0],-rot[i][1],rot[i][0],0;
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

    Eigen::MatrixXf Hom = foc_ * rot_i * rot_j.transpose() * foc_inv;
    return Hom;
}


parameters::parameters(std::vector<cv::Matx33f> &H,const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &ad,float foc,const std::vector<std::vector< cv::Matx33f >> &hom_mat){

    focal = foc;
    adj = ad;

    Eigen::MatrixXf foc_mat = get_foc(false);
    Eigen::MatrixXf foc_mat_inv = get_foc(true);

    for(int i = 0;i < H.size();i++){

        Eigen::Matrix3f matr;
        cv::cv2eigen(H[i], matr);
        matr = foc_mat_inv * matr * foc_mat;

        Eigen::VectorXf rotation = findRealColumns(matr);
        float min_val = get_approx(rotation,matr,3,40);
        min_val = gdsc(rotation,matr,.01,min_val,150);
        rotation = min_val * rotation;
        rot.push_back(rotation);

    }


    std::vector<std::vector<std::vector<Eigen::VectorXf>>> measure_mat(adj.rows, std::vector<std::vector<Eigen::VectorXf>>(adj.cols));

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            Eigen::MatrixXf hom;// = ret_hom(i, j);
            cv::cv2eigen(hom_mat[i][j], hom);


            if (.5 <= adj.at<double>(i,j)){

                for (int p = 0;p < match[i][j].size();p++){
                    Eigen::VectorXf temp(3);
                    Eigen::VectorXf measure = Eigen::VectorXf::Zero(3*adj.cols);

                    write_to_eigen(measure, kp[j].keypoint[match[i][j][p].trainIdx].pt,2,3*i);
                    measure[2+3*i] = 1;

                    write_to_eigen(temp, kp[j].keypoint[match[i][j][p].trainIdx].pt,2,0);
                    temp[2] = 1;
                    temp = hom * temp;
                    measure({3*j,1+3*j,2+3*j}) = temp;

                    //write_to_eigen(measure, kp[i].keypoint[match[i][j][p].queryIdx].pt,2,3);
                    //measure[5] = 1;

                    measure_mat[i][j].push_back(measure);

                }
            }
        }
    }

    measurements = measure_mat;

}


std::vector<Eigen::VectorXf> parameters::ret_measurements(){

    std::vector<Eigen::VectorXf> M;

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                for (int p = 0;p < measurements[i][j].size();p++){
                    M.push_back(measurements[i][j][p]);
                }
            }
        }
    }

    return M;
}


std::vector<Eigen::MatrixXf> parameters::ret_B_i(){

    int size = rot.size();
    std::vector<Eigen::MatrixXf> B_i;

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                Eigen::MatrixXf hom = ret_hom(i,j);

                for (int p = 0;p < measurements[i][j].size();p++){

                    Eigen::MatrixXf B_insert = Eigen::MatrixXf::Zero(2*size,3*size);

                    Eigen::VectorXf t = measurements[i][j][p]({3*i,1+3*i,2+3*i});
                    Eigen::VectorXf t_tr = hom*t;

                    Eigen::MatrixXf B1(2,3);
                    Eigen::MatrixXf B2(2,3);

                    B1 << 1/t[2],0,-t[0]/(t[2]*t[2]),0,1/t[2],-t[1]/(t[2]*t[2]);

                    B2 << (hom(0,0)*t_tr[2] - t_tr[0]*hom(2,0))/(t_tr[2] * t_tr[2]),(hom(0,1)*t_tr[2] - t_tr[0]*hom(2,1))/(t_tr[2] * t_tr[2]),(hom(0,2)*t_tr[2] - t_tr[0]*hom(2,2))/(t_tr[2] * t_tr[2]),(hom(1,0)*t_tr[2] - t_tr[1]*hom(2,0))/(t_tr[2] * t_tr[2]),(hom(1,1)*t_tr[2] - t_tr[1]*hom(2,1))/(t_tr[2] * t_tr[2]),(hom(1,2)*t_tr[2] - t_tr[1]*hom(2,2))/(t_tr[2] * t_tr[2]);

                    B_insert({0 + 2*i,1 + 2*i},{0 + 3*i,1 + 3*i,2 + 3*i}) = B1;
                    B_insert({0 + 2*j,1 + 2*j},{0 + 3*j,1 + 3*j,2 + 3*j}) = B2;
                    B_i.push_back(B_insert);

                }
            }
        }
    }

    return B_i;

}


Eigen::Matrix3f get_dR(const Eigen::Vector3f &rot_vec,const Eigen::MatrixXf &rot_mat,int axis,bool transposed){

    Eigen::Vector3f e_i = Eigen::Vector3f::Zero();
    Eigen::Matrix3f Id = Eigen::Matrix3f::Identity();
    e_i[axis] = 1;

    Eigen::Vector3f v_cross = rot_vec.cross( ( Id - rot_mat ) * e_i );
    Eigen::Matrix3f v_cx(3,3);
    v_cx << 0,-v_cross[2],v_cross[1],v_cross[2],0,-v_cross[0],-v_cross[1],v_cross[0],0;

    Eigen::MatrixXf v_x(3,3);
    v_x << 0,-rot_vec[2],rot_vec[1],rot_vec[2],0,-rot_vec[0],-rot_vec[1],rot_vec[0],0;
    v_x = v_x * rot_vec[axis];
    float sq = rot_vec.squaredNorm();

    Eigen::Matrix3f d_rot = ( ( v_x + v_cx )/sq ) * rot_mat;

    if (transposed){d_rot.transposeInPlace();}
    return d_rot;
}


std::vector<Eigen::MatrixXf> parameters::ret_A_i(){

    int size = rot.size();
    std::vector<Eigen::MatrixXf> A_i;

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            Eigen::MatrixXf hom = ret_hom(i,j);
            Eigen::MatrixXf rot_i = get_rot(i);
            Eigen::MatrixXf rot_j = get_rot(j);
            Eigen::MatrixXf foc_inv = get_foc(true);
            Eigen::MatrixXf foc_ = get_foc(false);
            Eigen::Matrix3f df;
            df << 1,0,0,0,1,0,0,0,0;

            Eigen::MatrixXf dfocal = ( df * rot_j * rot_i.transpose() * foc_inv - foc_ * rot_j * rot_i.transpose() * foc_inv * df * foc_inv );

            if (.5 <= adj.at<double>(i,j)){

                for (int p = 0;p < measurements[i][j].size();p++){

                    Eigen::MatrixXf A_insert = Eigen::MatrixXf::Zero(2*size,3*size+1);

                    Eigen::VectorXf t = measurements[i][j][p]({3*i,1+3*i,2+3*i});
                    Eigen::VectorXf f_i = dfocal * t;
                    Eigen::VectorXf t_tr = hom * t;

                    A_insert.col(0)({2*i,1+2*i})<< 0 , 0;
                    A_insert.col(0)({2*j,1+2*j})<< (f_i[0] * t_tr[2] - t_tr[0] * f_i[2])/(t_tr[2]*t_tr[2]),(f_i[1] * t_tr[2] - t_tr[0] * f_i[2])/(t_tr[2]*t_tr[2]);


                    for (int k = 0;k < 3;k++){

                        Eigen::Matrix3f d_R_i = get_dR(rot[i],rot_i,k,false);
                        Eigen::Matrix3f Hi = foc_ * d_R_i * rot_j.transpose() * foc_inv;
                        Eigen::VectorXf vec_dRi = Hi * t;

                        Eigen::Matrix3f d_R_j = get_dR(rot[j],rot_j,k,true);
                        Eigen::Matrix3f Hj = foc_ * rot_i * d_R_j * foc_inv;
                        Eigen::VectorXf vec_dRj = Hj * t;

                        A_insert.col(3*i+1+k)({2*i,1+2*i})<< 0,0;
                        A_insert.col(3*i+1+k)({2*j,1+2*j})<< (vec_dRi[0] * t_tr[2] - t_tr[0] * vec_dRi[2])/(t_tr[2]*t_tr[2]),(vec_dRi[1] * t_tr[2] - t_tr[0] * vec_dRi[2])/(t_tr[2]*t_tr[2]);

                        A_insert.col(3*j+1+k)({2*j,1+2*j})<< 0,0;
                        A_insert.col(3*j+1+k)({2*i,1+2*i})<<(vec_dRj[0] * t_tr[2] - t_tr[0] * vec_dRj[2])/(t_tr[2]*t_tr[2]),(vec_dRj[1] * t_tr[2] - t_tr[0] * vec_dRj[2])/(t_tr[2]*t_tr[2]);

                    }

                    A_i.push_back(A_insert);
                }

            }
        }
    }

    return A_i;

}


}


#ifndef BUNDLE_H
#define BUNDLE_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "_maths.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <opencv2/core/eigen.hpp>


namespace bund {

    Eigen::MatrixXf cov_mat_XY(Eigen::MatrixXf X,Eigen::MatrixXf Y);


    class parameters {

        public:

            parameters(std::vector<cv::Matx33f> &H,const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj,float foc,const std::vector<std::vector< cv::Matx33f >> &hom_mat);
            Eigen::MatrixXf ret_hom(int i, int j);
            std::vector<Eigen::MatrixXf> ret_B_i();
            std::vector<Eigen::MatrixXf> ret_A_i();
            std::vector<Eigen::VectorXf> ret_measurements();

        private:

            Eigen::MatrixXf get_rot(int i);
            Eigen::MatrixXf get_foc(bool inv);

            std::vector<std::vector<std::vector<Eigen::VectorXf>>> measurements;
            float focal;
            std::vector<Eigen::Vector3f> rot;
            cv::Mat adj;

    };



    class E_func {

        public:

            E_func(const std::vector<maths::keypoints> &kp,std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj);
            std::vector<Eigen::VectorXf> error(const std::vector<Eigen::VectorXf> &t_to_q);
            std::vector<Eigen::VectorXf> get_measurements();
            std::vector<std::vector<int>> ret_idx_set();

        private:

            std::vector<Eigen::VectorXf> measurements;
            std::vector<std::vector<int>> idx_set;

    };



}

#endif



parameters::parameters(std::vector<cv::Matx33f> &H,const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &ad,float foc,const std::vector<std::vector< cv::Matx33f >> &hom_mat,const imgm::pan_img_transform &Tr){

    adj = ad;
    float eps = 5e-5;

    std::vector<std::vector<std::vector<Eigen::VectorXf>>> measure_mat(adj.rows, std::vector<std::vector<Eigen::VectorXf>>(adj.cols));

    std::vector<std::vector<Eigen::MatrixXf>> normalizeri(hom_mat.size(), std::vector<Eigen::MatrixXf>(hom_mat[0].size()));

    std::vector<std::vector<Eigen::MatrixXf>> normalizerj(hom_mat.size(), std::vector<Eigen::MatrixXf>(hom_mat[0].size()));

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            std::vector<cv::Vec3f> normi;
            std::vector<cv::Vec3f> normj;

            if (.5 <= adj.at<double>(i,j)){

                for (int p = 0;p < match[i][j].size();p++){
                    cv::Vec3f vec;
                    cv::Vec2f tempf = kp[j].keypoint[match[i][j][p].trainIdx].pt;

                    vec[0] = tempf[0];
                    vec[1] = tempf[1];
                    vec[2] = 1;
                    normi.push_back(vec);

                    tempf = kp[i].keypoint[match[i][j][p].queryIdx].pt;
                    vec[0] = tempf[0];
                    vec[1] = tempf[1];
                    vec[2] = 1;
                    normj.push_back(vec);

                }

                cv::Matx33f N = maths::Normalize2D(normi);
                cv::cv2eigen(N,normalizeri[i][j]);

                N = maths::Normalize2D(normj);
                cv::cv2eigen(N,normalizerj[i][j]);

            }
        }
    }

    R_approx.normalizerTi = normalizeri;
    R_approx.normalizerTj = normalizerj;

    R_approx = approximate_R(hom_mat,Tr,foc);


    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            //std::cout<<"kapprox" << R_approx.K[i][j]<<"\n";
            //std::cout<<"kinv" << R_approx.K[i][j].inverse()<<"\n";
            //std::cout<<"R" << R_approx.R[i][j]<<"\n";

            Eigen::MatrixXf hom = ret_hom(i,j);

                for (int p = 0;p < match[i][j].size();p++){
                    Eigen::VectorXf temp(3);
                    Eigen::VectorXf measure = Eigen::VectorXf::Zero(6);

                    write_to_eigen(temp, kp[j].keypoint[match[i][j][p].trainIdx].pt,2,0);
                    temp[2] = 1;
                    temp = normalizerj[i][j] * temp;
                    measure({0,1,2}) = temp;

                    temp = hom * temp;
                    measure({3,4,5}) = temp;

                    measure_mat[i][j].push_back(measure);

                }
            }

        }
    }

    measurements = measure_mat;

}



