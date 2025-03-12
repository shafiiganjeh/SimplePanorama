#include "_bundle_adjust_tools.h"

namespace bund {

using Eigen::MatrixXf;
using Eigen::VectorXf;


void write_to_eigen(Eigen::VectorXd &v,const cv::Vec2d &cv_v,int n,int st){

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

                    Eigen::VectorXd measure = Eigen::VectorXd::Zero(4);
                    write_to_eigen(measure, static_cast<cv::Point2d>(kp[i].keypoint[match[i][j][p].queryIdx].pt),2,2);
                    write_to_eigen(measure, static_cast<cv::Point2d>(kp[j].keypoint[match[i][j][p].trainIdx].pt),2,0);
                    measurements.push_back(measure);
                    idx_set.push_back(idx);

                }
            }
        }
    }
}


std::vector<Eigen::VectorXd> E_func::get_measurements(){

    return measurements;

}

std::vector<std::vector<int>> E_func::ret_idx_set(){

    return idx_set;
}


std::vector<Eigen::VectorXd> E_func::error(const std::vector<Eigen::VectorXd> &t_to_q){


    std::vector<Eigen::VectorXd> error_vec;

    for (int n = 0;n < measurements.size();n++){
        Eigen::VectorXd er = Eigen::VectorXd::Zero(4);

        er({2,3}) = t_to_q[n]({3,4})/t_to_q[n][5];
        er({0,1}) = t_to_q[n]({0,1})/t_to_q[n][2];

        error_vec.push_back(measurements[n] - er);

    }

    return error_vec;

}

Eigen::MatrixXd get_rot(Eigen::MatrixXd rotv){


    Eigen::MatrixXd v(3,3);
    v << 0,-rotv(2,0),rotv(1,0),rotv(2,0),0,-rotv(0,0),-rotv(1,0),rotv(0,0),0;
    Eigen::MatrixXd ex = v.exp();
    return ex;

}


Eigen::MatrixXd parameters::ret_hom(int i, int j){

    Eigen::MatrixXd Hom = K[j] * rot[j] * rot[i].transpose() * K_inv[i];
    return Hom;
}

Eigen::MatrixXd to_K(double focal,const Eigen::Vector2d &pvec){

    Eigen::MatrixXd v(3,3);
    v << focal,0,pvec[0],0,focal,pvec[1],0,0,1;

    return v;
}

Eigen::MatrixXd to_K_inv(double focal,const Eigen::Vector2d &pvec){

    Eigen::MatrixXd v(3,3);
    v << focal,0,pvec[0],0,focal,pvec[1],0,0,1;

    return v.inverse().eval();
}

parameters::parameters(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const class imgm::pan_img_transform &T){

    //double focal = T.focal;
    adj = *T.adj;
    adj = adj  + adj.t();
    float eps = 5e-5;

    std::cout<<"adj :"<< adj<<" \n";

    for(int i = 0 ; i < T.rot.size();i++){
        std::cout<<"rot :"<< T.rot[i] <<" \n";
        focal.push_back(T.focal);

        cv::Mat r_transform;
        rot.push_back(T.rot[i]);
        cv::eigen2cv(T.rot[i],r_transform);
        cv::Mat r_transform_vec;
        cv::Rodrigues(r_transform, r_transform_vec);
        Eigen::MatrixXd rot_v;
        cv::cv2eigen(r_transform_vec,rot_v);
        rot_vec.push_back(rot_v);

        //principal_vec
        Eigen::Vector2d pvec;

        pvec <<0,0;
        principal_vec.push_back(pvec);
        K.push_back(to_K(focal[i],pvec));
        K_inv.push_back(to_K_inv(focal[i],pvec));

    }


    std::vector<std::vector<std::vector<Eigen::VectorXd>>> measure_mat(adj.rows, std::vector<std::vector<Eigen::VectorXd>>(adj.cols));
    std::vector<int> idx(2);

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){
                Eigen::MatrixXd hom = ret_hom(i, j);

                for (int p = 0;p < match[i][j].size();p++){
                    Eigen::VectorXd temp(3);
                    Eigen::VectorXd measure = Eigen::VectorXd::Zero(6);

                    write_to_eigen(measure, static_cast<cv::Point2d>(kp[j].keypoint[match[i][j][p].trainIdx].pt),2,0);
                    measure[2] = 1;

                    write_to_eigen(temp, static_cast<cv::Point2d>(kp[j].keypoint[match[i][j][p].trainIdx].pt),2,0);
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


std::vector<Eigen::VectorXd> parameters::ret_measurements(){

    std::vector<Eigen::VectorXd> M;

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



std::vector<Eigen::MatrixXd> parameters::ret_B_i(){

    std::vector<Eigen::MatrixXd> B_i;

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                Eigen::MatrixXd hom = ret_hom(i,j);

                for (int p = 0;p < measurements[i][j].size();p++){

                    Eigen::MatrixXd B_insert = Eigen::MatrixXd::Zero(4,2);

                    B_insert({0,1},{0,1}) = Eigen::MatrixXd::Identity(2,2);


                    Eigen::VectorXd t = measurements[i][j][p]({0,1,2});
                    Eigen::VectorXd t_tr = hom*t;
                    //Eigen::VectorXf t_tr = measurements[i][j][p]({3,4,5});


                    Eigen::MatrixXd B1(2,3);
                    B1 << 1/t_tr[2],0,-t_tr[0]/(t_tr[2]*t_tr[2]),0,1/t_tr[2],-t_tr[1]/(t_tr[2]*t_tr[2]);
                    B1 = B1 *  hom({0,1,2},{0,1});

                    B_insert({2,3},{0,1}) = B1;

                    B_i.push_back(B_insert);

                }
            }
        }
    }

    return B_i;

}



Eigen::VectorXd d_funcb(Eigen::VectorXd &inp_v,const Eigen::VectorXd &par){

    Eigen::MatrixXd hom = Eigen::Map<Eigen::MatrixXd>(inp_v.data(), 3, 3);

    Eigen::VectorXd v(3);
    v({0,1}) = par;
    v[2] = 1;
    v = hom * v;
    v = v/v[2];

    Eigen::VectorXd ret(4);
    ret({0,1}) = par;
    ret({2,3}) = v({0,1});

    return ret;
}


Eigen::MatrixXd nummeric_divb(const Eigen::VectorXd &inp_vf,const Eigen::VectorXd &parf){

    Eigen::VectorXd inp_v = inp_vf.cast<double>();
    Eigen::VectorXd par = parf.cast<double>();

    float eps = 1e-5;
    Eigen::VectorXd test = d_funcb(inp_v,par);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(test.size(), par.size());

    for(int i = 0;i < par.size(); i++){

        float h = eps;//par[i] * eps;
        Eigen::VectorXd xph = par;
        xph[i] = xph[i] + h;
        Eigen::VectorXd xmh = par;
        xmh[i] = xmh[i] - h;

        Eigen::VectorXd p_x = d_funcb(inp_v,xph);
        Eigen::VectorXd m_x = d_funcb(inp_v,xmh);

        Eigen::VectorXd df = (p_x - m_x)/(2*h);

        for (int j = 0;j < df.size();j++){

            D(j,i) = df[j];

        }

    }

    return D;

}


std::vector<Eigen::MatrixXd> parameters::ret_B_i_num(){

    std::vector<Eigen::MatrixXd> B_i;

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                Eigen::MatrixXd hom = ret_hom(i,j);
                Eigen::VectorXd hom_v = Eigen::Map<Eigen::VectorXd>(hom.data(), hom.cols()*hom.rows());

                for (int p = 0;p < measurements[i][j].size();p++){

                    Eigen::VectorXd t = measurements[i][j][p]({0,1,2});
                    t = t/t[2];

                    Eigen::MatrixXd bi = nummeric_divb(hom_v,t({0,1}));


                    B_i.push_back(bi);

                }
            }
        }
    }

    return B_i;

}


Eigen::Matrix3d get_dR(const Eigen::Vector3d &rot_vec,const Eigen::MatrixXd &rot_mat,int axis,bool transposed){

    double eps = 1e-8;

    Eigen::Vector3d e_i = Eigen::Vector3d::Zero();
    Eigen::Matrix3d Id = Eigen::Matrix3d::Identity();
    e_i[axis] = 1;

    Eigen::Vector3d v_cross = rot_vec.cross( ( Id - rot_mat ) * e_i );
    Eigen::Matrix3d v_cx(3,3);
    v_cx << 0,-v_cross[2],v_cross[1],v_cross[2],0,-v_cross[0],-v_cross[1],v_cross[0],0;

    Eigen::MatrixXd v_x(3,3);
    v_x << 0,-rot_vec[2],rot_vec[1],rot_vec[2],0,-rot_vec[0],-rot_vec[1],rot_vec[0],0;
    v_x = v_x * rot_vec[axis];
    float sq = rot_vec.squaredNorm();

    Eigen::Matrix3d d_rot = ( ( v_x + v_cx ) / (sq + eps) ) * rot_mat;

    if (transposed){d_rot.transposeInPlace();}
    return d_rot;
}



std::vector<Eigen::MatrixXd> parameters::ret_A_i(){

    int size = 3*rot_vec.size()+focal.size() + 2*principal_vec.size();
    std::vector<Eigen::MatrixXd> A_i;

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            //K[j] * rot[j] * rot[i].transpose() * K_inv[i];
            if (.5 <= adj.at<double>(i,j)){

                Eigen::MatrixXd hom = ret_hom(i,j);

                Eigen::Matrix3d df;
                df << 1,0,0,0,1,0,0,0,0;

                Eigen::Matrix3d dx;
                dx << 0,0,1,0,0,0,0,0,0;

                Eigen::Matrix3d dy;
                dy << 0,0,0,0,0,1,0,0,0;

                std::vector<Eigen::Matrix3d> D = {df,dx,dy};

                for (int p = 0;p < measurements[i][j].size();p++){

                    Eigen::VectorXd t = measurements[i][j][p]({0,1,2});
                    Eigen::VectorXd t_tr = hom * t;
                    Eigen::MatrixXd A_insert = Eigen::MatrixXd::Zero(4,size);

                    for(int d = 0;d<D.size();d++){

                        Eigen::MatrixXd dfocal_I = -(K[j] * rot[j] * rot[i].transpose() * K_inv[i] * D[d] * K_inv[i]);
                        Eigen::MatrixXd dfocal_J = ( D[d] * rot[j] * rot[i].transpose() * K_inv[i]);

                        Eigen::VectorXd f_i = dfocal_I * t;

                        //Eigen::VectorXf t_tr = measurements[i][j][p]({3,4,5});

                        A_insert.col(i * 6 + d)({0,1})<< 0 , 0;
                        A_insert.col(i * 6 + d)({2,3})<< (f_i[0] * t_tr[2] - t_tr[0] * f_i[2])/(t_tr[2]*t_tr[2]),(f_i[1] * t_tr[2] - t_tr[1] * f_i[2])/(t_tr[2]*t_tr[2]);


                        f_i = dfocal_J * t;
                        t_tr = hom * t;

                        A_insert.col(j * 6 + d)({0,1})<< 0 , 0;
                        A_insert.col(j * 6 + d)({2,3})<< (f_i[0] * t_tr[2] - t_tr[0] * f_i[2])/(t_tr[2]*t_tr[2]),(f_i[1] * t_tr[2] - t_tr[1] * f_i[2])/(t_tr[2]*t_tr[2]);
                    }

                    for (int k = 0;k < 3;k++){

                        Eigen::Matrix3d d_R_i = get_dR(rot_vec[i],rot[i],k,true);
                        Eigen::Matrix3d Hi = K[j] * rot[j] * d_R_i * K_inv[i];
                        Eigen::VectorXd vec_dRj = Hi * t;

                        A_insert.col(i * 6 + k +3)({0,1})<< 0,0;
                        A_insert.col(i * 6 + k +3)({2,3})<<(vec_dRj[0] * t_tr[2] - t_tr[0] * vec_dRj[2])/(t_tr[2]*t_tr[2]),(vec_dRj[1] * t_tr[2] - t_tr[0] * vec_dRj[2])/(t_tr[2]*t_tr[2]);

                        d_R_i = get_dR(rot_vec[j],rot[j],k,false);
                        Hi = K[j] * d_R_i * rot[i].transpose() * K_inv[i];
                        vec_dRj = Hi * t;

                        A_insert.col(j * 6 + k +3)({0,1})<< 0,0;
                        A_insert.col(j * 6 + k +3)({2,3})<<(vec_dRj[0] * t_tr[2] - t_tr[0] * vec_dRj[2])/(t_tr[2]*t_tr[2]),(vec_dRj[1] * t_tr[2] - t_tr[0] * vec_dRj[2])/(t_tr[2]*t_tr[2]);

                    }

                    A_i.push_back(A_insert);
                }

            }
       }
    }

    return A_i;

}


Eigen::VectorXd d_func(const Eigen::VectorXd &inp_v,const Eigen::VectorXd &par){

    Eigen::Vector3d x;
    x({0,1}) = inp_v;
    x[2] = 1;

    Eigen::Matrix3d Ki_inv  = to_K_inv(par[0],par({1,2}));
    Eigen::Matrix3d Kj = to_K(par[6],par({7,8}));
    Eigen::Matrix3d Ri;
    Eigen::Matrix3d Rj;

    double mi[1][3] = {{par[3], par[4], par[5]}};
    cv::Mat ri(1, 3, CV_64F, mi);
    double mj[1][3] = {{par[9], par[10], par[11]}};
    cv::Mat rj(1, 3, CV_64F, mj);

    Eigen::MatrixXd tri(3,1);
    tri <<par[3], par[4], par[5];

    Eigen::MatrixXd trj(3,1);
    trj <<par[9], par[10], par[11];

    Ri = get_rot(tri);
    Rj = get_rot(trj);

    Eigen::MatrixXd Hom = Kj * Rj * Ri.transpose() * Ki_inv;

    x = Hom * x;
    x = x/x[2];

    Eigen::VectorXd ret(4);
    ret({0,1}) = inp_v;
    ret({2,3}) = x({0,1});

    return ret;
}



Eigen::MatrixXd nummeric_div(const Eigen::VectorXd &inp_vf,const Eigen::VectorXd &parf){

    Eigen::VectorXd inp_v = inp_vf.cast<double>();

    double eps = 1e-5;
    Eigen::VectorXd test = d_func(inp_v,parf);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(test.size(), parf.size());

    for(int i = 0;i < parf.size(); i++){

        double h = eps;
        Eigen::VectorXd xph = parf;
        xph[i] = xph[i] + h;
        Eigen::VectorXd xmh = parf;
        xmh[i] = xmh[i] - h;

        Eigen::VectorXd p_x = d_func(inp_v,xph);
        Eigen::VectorXd m_x = d_func(inp_v,xmh);

        Eigen::VectorXd df = (p_x - m_x)/(2*h);

        for (int j = 0;j < df.size();j++){

            D(j,i) = df[j];

        }

    }

    return D;

}


std::vector<Eigen::MatrixXd> parameters::ret_A_i_num(){

    int size = 3*rot_vec.size() + focal.size() + 2*principal_vec.size();
    std::vector<Eigen::MatrixXd> A_i;

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                Eigen::VectorXd par(12);
                par[0] = focal[i];
                par[1] = principal_vec[i][0];
                par[2] = principal_vec[i][1];
                par({3,4,5}) = rot_vec[i]({0,1,2},{0});

                par[6] = focal[j];
                par[7] = principal_vec[j][0];
                par[8] = principal_vec[j][1];
                par({9,10,11}) = rot_vec[j]({0,1,2},{0});


                for (int p = 0;p < measurements[i][j].size();p++){

                    Eigen::MatrixXd A_insert = Eigen::MatrixXd::Zero(4,size);

                    Eigen::VectorXd t = measurements[i][j][p]({0,1});
                    Eigen::MatrixXd Ainum = nummeric_div(t,par);

                    A_insert({0,1,2,3},{0+i*6,1+i*6,2+i*6,3+i*6,4+i*6,5+i*6,0+j*6,1+j*6,2+j*6,3+j*6,4+j*6,5+j*6}) << Ainum;
                    A_i.push_back(A_insert);

                }

            }

        }
    }

    return A_i;

}



void parameters::add_delta(std::vector<Eigen::VectorXd> delta_b,Eigen::VectorXd delta_a,bool add_rot){

    std::vector<Eigen::MatrixXd>().swap(rot_res);
    copy(rot.begin(), rot.end(), back_inserter(rot_res));

    std::vector<Eigen::MatrixXd>().swap(rot_vec_res);
    copy(rot_vec.begin(), rot_vec.end(), back_inserter(rot_vec_res));

    std::vector<std::vector<std::vector<Eigen::VectorXd>>>().swap(measurements_res);
    copy(measurements.begin(), measurements.end(), back_inserter(measurements_res));

    std::vector<Eigen::MatrixXd>().swap(K_res);
    copy(K.begin(), K.end(), back_inserter(K_res));

    std::vector<Eigen::MatrixXd>().swap(K_inv_res);
    copy(K_inv.begin(), K_inv.end(), back_inserter(K_inv_res));

    std::vector<double>().swap(focal_res);
    copy(focal.begin(), focal.end(), back_inserter(focal_res));

    std::vector<Eigen::Vector2d>().swap(principal_vec_res);
    copy(principal_vec.begin(), principal_vec.end(), back_inserter(principal_vec_res));

    double kl = 1;
    double kl2 = 1;
    double kl3 = 1;
    for(int i = 0;i < rot.size();i++){

        if(add_rot){
            //rot_vec[i] = rot_vec[i] + delta_a({3+i*6,4+i*6,5+i*6});
            rot_vec[i](0,0) = rot_vec[i](0,0) + delta_a[3+i*6]*kl;
            rot_vec[i](1,0) = rot_vec[i](1,0) + delta_a[4+i*6]*kl;
            rot_vec[i](2,0) = rot_vec[i](2,0) + delta_a[5+i*6]*kl;
            //std::cout <<"rot_vec[i] " <<rot_vec[i]<<"\n";

        }

        rot[i] = get_rot(rot_vec[i]);
        focal[i] = focal[i] + delta_a[i*6]*kl3;
        principal_vec[i] = principal_vec[i] + delta_a({1+i*6,2+i*6})*kl2;

        K[i] = to_K(focal[i],principal_vec[i]);
        K_inv[i] = to_K_inv(focal[i],principal_vec[i]);


    }

    int c = 0;
    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                Eigen::MatrixXd hom = ret_hom(i, j);
                for (int p = 0;p < measurements[i][j].size();p++){

                    Eigen::VectorXd temp(3);
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

    std::vector<Eigen::MatrixXd>().swap(rot);
    copy(rot_res.begin(), rot_res.end(), back_inserter(rot));

    std::vector<Eigen::MatrixXd>().swap(rot_vec);
    copy(rot_vec_res.begin(), rot_vec_res.end(), back_inserter(rot_vec));

    std::vector<std::vector<std::vector<Eigen::VectorXd>>>().swap(measurements);
    copy(measurements_res.begin(), measurements_res.end(), back_inserter(measurements));

    std::vector<Eigen::MatrixXd>().swap(K);
    copy(K_res.begin(), K_res.end(), back_inserter(K));

    std::vector<Eigen::MatrixXd>().swap(K_inv);
    copy(K_inv_res.begin(), K_inv_res.end(), back_inserter(K_inv));

    std::vector<double>().swap(focal);
    copy(focal_res.begin(), focal_res.end(), back_inserter(focal));

    std::vector<Eigen::Vector2d>().swap(principal_vec);
    copy(principal_vec_res.begin(), principal_vec_res.end(), back_inserter(principal_vec));

}


std::vector<std::vector< cv::Matx33f >> parameters::ret_hmat(){

    //std::vector<std::vector< cv::Matx33f >> hom_mat(adj.rows, std::vector<cv::Matx33f>(adj.cols, cv::Matx33f));
    std::vector<std::vector<cv::Matx33f>> hom_mat(adj.rows, std::vector<cv::Matx33f>(adj.cols, cv::Matx33f::eye()));

    for (int i = 0;i < adj.rows;i++){

        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                Eigen::MatrixXd hom = ret_hom(i, j);
                Eigen::MatrixXd hom_inv = ret_hom(j, i);
                Eigen::MatrixXf homf = hom.cast <float> ();
                Eigen::MatrixXf homfinv = hom_inv.cast <float> ();

                cv::eigen2cv(homf,hom_mat[i][j]);
                cv::eigen2cv(homfinv,hom_mat[j][i]);

            }
        }
    }

    return hom_mat;

}

std::vector<double> parameters::ret_focal(){

    return focal;

}


std::vector<Eigen::MatrixXd> parameters::ret_rot(){

    return rot;

}


std::vector<Eigen::MatrixXd> parameters::ret_K(){

    return K;

}


}
