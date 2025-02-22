#include "_bundle_adjust_tools.h"

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


E_func::E_func(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adj){


    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                //std::cout<<"size: " << match[i][j].size()<<"\n";
                for (int p = 0;p < match[i][j].size();p++){
                    std::vector<int> idx(2);
                    idx[0] = i;
                    idx[1] = j;

                    Eigen::VectorXf measure = Eigen::VectorXf::Zero(4);
                    write_to_eigen(measure, kp[i].keypoint[match[i][j][p].queryIdx].pt,2,2);
                    write_to_eigen(measure, kp[j].keypoint[match[i][j][p].trainIdx].pt,2,0);
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


    std::vector<Eigen::VectorXf> error_vec;

    for (int n = 0;n < measurements.size();n++){
        Eigen::VectorXf er = Eigen::VectorXf::Zero(4);

        er({2,3}) = t_to_q[n]({3,4})/t_to_q[n][5];
        er({0,1}) = t_to_q[n]({0,1})/t_to_q[n][2];

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


parameters::parameters(std::vector<cv::Matx33f> &H,const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &ad,float foc){

    focal = foc;
    adj = ad;
    float eps = 5e-5;

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
    std::vector<int> idx(2);

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            Eigen::MatrixXf hom = ret_hom(i, j);

            if (.5 <= adj.at<double>(i,j)){

                for (int p = 0;p < match[i][j].size();p++){
                    Eigen::VectorXf temp(3);
                    Eigen::VectorXf measure = Eigen::VectorXf::Zero(6);

                    write_to_eigen(measure, kp[j].keypoint[match[i][j][p].trainIdx].pt,2,0);
                    measure[2] = 1;

                    write_to_eigen(temp, kp[j].keypoint[match[i][j][p].trainIdx].pt,2,0);
                    temp[2] = 1;
                    temp = hom * temp;
                    measure({3,4,5}) = temp;

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

                    Eigen::MatrixXf B_insert = Eigen::MatrixXf::Zero(4,2);

                    B_insert({0,1},{0,1}) = Eigen::MatrixXf::Identity(2,2);


                    Eigen::VectorXf t = measurements[i][j][p]({0,1,2});
                    Eigen::VectorXf t_tr = hom*t;
                    //Eigen::VectorXf t_tr = measurements[i][j][p]({3,4,5});


                    Eigen::MatrixXf B1(2,3);
                    B1 << 1/t_tr[0],0,-t_tr[0]/(t_tr[2]*t_tr[2]),0,1/t_tr[0],-t_tr[1]/(t_tr[2]*t_tr[2]);
                    B1 = B1 *  hom({0,1,2},{0,1});

                    B_insert({2,3},{0,1}) = B1;

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

                    Eigen::MatrixXf A_insert = Eigen::MatrixXf::Zero(4,3*size+1);

                    Eigen::VectorXf t = measurements[i][j][p]({0,1,2});
                    Eigen::VectorXf f_i = dfocal * t;
                    Eigen::VectorXf t_tr = hom * t;
                    //Eigen::VectorXf t_tr = measurements[i][j][p]({3,4,5});

                    A_insert.col(0)({0,1})<< 0 , 0;
                    A_insert.col(0)({2,3})<< (f_i[0] * t_tr[2] - t_tr[0] * f_i[2])/(t_tr[2]*t_tr[2]),(f_i[1] * t_tr[2] - t_tr[0] * f_i[2])/(t_tr[2]*t_tr[2]);


                    for (int k = 0;k < 3;k++){

                        Eigen::Matrix3f d_R_i = get_dR(rot[i],rot_i,k,false);
                        Eigen::Matrix3f Hi = foc_ * d_R_i * rot_j.transpose() * foc_inv;
                        Eigen::VectorXf vec_dRi = Hi * t;

                        Eigen::Matrix3f d_R_j = get_dR(rot[j],rot_j,k,true);
                        Eigen::Matrix3f Hj = foc_ * rot_i * d_R_j * foc_inv;
                        Eigen::VectorXf vec_dRj = Hj * t;

                        A_insert.col(3*i+1+k)({0,1})<< 0,0;
                        A_insert.col(3*i+1+k)({2,3})<< (vec_dRi[0] * t_tr[2] - t_tr[0] * vec_dRi[2])/(t_tr[2]*t_tr[2]),(vec_dRi[1] * t_tr[2] - t_tr[0] * vec_dRi[2])/(t_tr[2]*t_tr[2]);

                        A_insert.col(3*j+1+k)({0,1})<< 0,0;
                        A_insert.col(3*j+1+k)({2,3})<<(vec_dRj[0] * t_tr[2] - t_tr[0] * vec_dRj[2])/(t_tr[2]*t_tr[2]),(vec_dRj[1] * t_tr[2] - t_tr[0] * vec_dRj[2])/(t_tr[2]*t_tr[2]);

                    }

                    A_i.push_back(A_insert);
                }

            }
        }
    }

    return A_i;

}

void parameters::add_delta(std::vector<Eigen::VectorXf> delta_b,Eigen::VectorXf delta_a){

    int c = 0;
    focal_res = focal;

    std::vector<Eigen::Vector3f>().swap(rot_res);
    std::vector<std::vector<std::vector<Eigen::VectorXf>>>().swap(measurements_res);

    copy(rot.begin(), rot.end(), back_inserter(rot_res));
    copy(measurements.begin(), measurements.end(), back_inserter(measurements_res));

    focal = focal + delta_a[0];

    for (int i = 0;i < rot.size();i++){

        rot[i] = delta_a({1+i*3,2+i*3,3+i*3});

    }

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            Eigen::MatrixXf hom = ret_hom(i, j);

            if (.5 <= adj.at<double>(i,j)){

                for (int p = 0;p < measurements[i][j].size();p++){

                    Eigen::VectorXf temp(3);
                    measurements[i][j][p]({0,1}) = measurements[i][j][p]({0,1}) + delta_b[c];

                    temp({0,1}) = measurements[i][j][p]({0,1});
                    temp[2] = 1;
                    temp = hom * temp;
                    measurements[i][j][p]({3,4,5}) = temp;

                    c++;
                }
            }
        }
    }

}

void parameters::reset(){

    focal = focal_res;

    std::vector<Eigen::Vector3f>().swap(rot);
    std::vector<std::vector<std::vector<Eigen::VectorXf>>>().swap(measurements);

    copy(rot_res.begin(), rot_res.end(), back_inserter(rot));
    copy(measurements_res.begin(), measurements_res.end(), back_inserter(measurements));

}


std::vector<std::vector< cv::Matx33f >> parameters::ret_hmat(){

    //std::vector<std::vector< cv::Matx33f >> hom_mat(adj.rows, std::vector<cv::Matx33f>(adj.cols, cv::Matx33f));
    std::vector<std::vector<cv::Matx33f>> hom_mat(adj.rows, std::vector<cv::Matx33f>(adj.cols, cv::Matx33f::eye()));

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            Eigen::MatrixXf hom = ret_hom(i, j);

            if (.5 <= adj.at<double>(i,j)){

                cv::eigen2cv(hom,hom_mat[i][j]);

            }
        }
    }

    return hom_mat;

}


}




