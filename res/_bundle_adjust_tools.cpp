#include "_bundle_adjust_tools.h"

namespace bund {


double cv_frobeniusNorm(const cv::Matx33d &mat){

    cv::Matx33d output = mat.mul(mat);

    cv::Scalar sum = cv::sum(output);
    cv::sqrt(sum,sum);

    return sum[0];
}

//nimm translation matrix aus der approximation
struct approx approximate_R(const std::vector<std::vector< cv::Matx33f >> &hom_mat,const imgm::pan_img_transform &Tr,float focal,struct approx &app){

    struct approx H_approx;
    std::vector<std::vector<Eigen::Matrix3d>> rot_approx(hom_mat.size(), std::vector<Eigen::Matrix3d>(hom_mat[0].size(), Eigen::Matrix3d::Identity()));

    std::vector<std::vector<Eigen::Matrix3d>> foc_approx(hom_mat.size(), std::vector<Eigen::Matrix3d>(hom_mat[0].size(), Eigen::Matrix3d::Identity()));

    std::vector<std::vector<Eigen::Matrix3d>> foc_approx_inv(hom_mat.size(), std::vector<Eigen::Matrix3d>(hom_mat[0].size(), Eigen::Matrix3d::Identity()));

    std::vector<std::vector<Eigen::MatrixXd>> rotV_approx(hom_mat.size(), std::vector<Eigen::MatrixXd>(hom_mat[0].size()));

    H_approx.normalizerTi = app.normalizerTi;
    H_approx.normalizerTj = app.normalizerTj;

    for (int i = 0;i < (*Tr.adj).rows;i++){
        for(int j = i;j < (*Tr.adj).cols;j++){

            if (.5 <= (*Tr.adj).at<double>(i,j)){

                cv::Matx33d homij(hom_mat[i][j]);

                cv::Matx33d normi;
                cv::Matx33d normj;
                cv::eigen2cv(H_approx.normalizerTi[i][j],normi);
                cv::eigen2cv(H_approx.normalizerTj[i][j],normj);

                struct maths::translation trans = maths::get_translation((*Tr.img_address)[i], (*Tr.img_address)[j],hom_mat[i][j]);

                //homij = normidouble * homij * normjdouble.inv();

                cv::Matx33d foca(focal, 0, 0,
                                0, focal, 0,
                                0, 0, 1);
                cv::Matx33d trs(trans.T);
                foca(0,2) = trs(0,2);
                foca(1,2) = trs(1,2);


                homij = normj * homij * normi.inv();
                foca = normj * foca * normi.inv();

                std::vector<cv::Mat> Rs;
                std::vector<cv::Mat> Ts;

                cv::decomposeHomographyMat(homij, foca, Rs, Ts, cv::noArray());

                cv::cv2eigen(foca,foc_approx[i][j]);
                foc_approx_inv[i][j] = foc_approx[i][j].inverse().eval();


                cv::Mat Rot;
                float ediff = -1;

                for(int i = 0;i < Rs.size();i++){

                    cv::Mat homap_0 = foca * Rs[i] * foca.inv();
                    double s = homij.dot(homap_0) / homij.dot(homap_0);
                    cv::Mat hom_est_scaled = homap_0 * s;
                    double h_dif = cv::norm(homij-hom_est_scaled, cv::NORM_L2);
                    if ((h_dif < ediff) or (ediff == -1) ){

                        Rot = Rs[i];
                        ediff = h_dif;
                        std::cout <<"rs  "<<i<<"\n";
                    }

                }


                cv::Matx33d rotf = Rot;
                cv::cv2eigen(rotf,rot_approx[i][j]);

                cv::Mat rot_v;
                cv::Rodrigues(rotf, rot_v);
                cv::cv2eigen(rot_v,rotV_approx[i][j]);

            }
        }
    }
    //std::cout<<"foc_app: " << foc_approx[0][1]<<"\n";
    H_approx.K = foc_approx;
    H_approx.K_inv = foc_approx_inv;
    H_approx.R = rot_approx;
    H_approx.R_v = rotV_approx;

    return H_approx;

}




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


std::vector<Eigen::VectorXd> E_func::error(const std::vector<Eigen::VectorXd> &t_to_q,const struct approx &normalize){


    std::vector<Eigen::VectorXd> error_vec;

    for (int n = 0;n < measurements.size();n++){
        Eigen::VectorXd er = Eigen::VectorXd::Zero(4);
        Eigen::VectorXd prd = Eigen::VectorXd::Zero(4);
        Eigen::Vector3d temp;
        Eigen::Vector3d temp_m;
        temp_m({0,1}) = measurements[n]({0,1});
        temp_m[2] = 1;
        temp_m = normalize.normalizerTi[idx_set[n][0]][idx_set[n][1]] * temp_m;
        temp_m = temp_m/temp_m[2];
        prd({0,1}) = temp_m({0,1});

        temp_m({0,1}) = measurements[n]({2,3});
        temp_m[2] = 1;
        temp_m = normalize.normalizerTj[idx_set[n][0]][idx_set[n][1]] * temp_m;
        temp_m = temp_m/temp_m[2];
        prd({2,3}) = temp_m({0,1});

        temp = t_to_q[n]({3,4,5});
        //temp = normalize.normalizerTj[idx_set[n][0]][idx_set[n][1]] * temp;
        er({2,3}) = temp({0,1})/temp[2];

        temp = t_to_q[n]({0,1,2});
        //temp = normalize.normalizerTi[idx_set[n][0]][idx_set[n][1]] * temp;
        er({0,1}) = temp({0,1})/temp[2];

        //error_vec.push_back(measurements[n] - er);
        error_vec.push_back(prd - er);

    }

    return error_vec;

}


Eigen::MatrixXd parameters::ret_hom(int i, int j){

    Eigen::MatrixXd hom = R_approx.K[i][j] * R_approx.R[i][j] * R_approx.K[i][j].inverse();
    //std::cout<<"r_app: " << R_approx.K[i][j]<<"\n";

    return hom;
}


struct approx parameters::ret_all(){

    return R_approx;

}


parameters::parameters(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &ad,float foc,const std::vector<std::vector< cv::Matx33f >> &hom_mat,const imgm::pan_img_transform &Tr){

    adj = ad;
    float eps = 5e-5;

    std::vector<std::vector<std::vector<Eigen::VectorXd>>> measure_mat(adj.rows, std::vector<std::vector<Eigen::VectorXd>>(adj.cols));

    std::vector<std::vector<Eigen::MatrixXd>> normalizeri(hom_mat.size(), std::vector<Eigen::MatrixXd>(hom_mat[0].size()));

    std::vector<std::vector<Eigen::MatrixXd>> normalizerj(hom_mat.size(), std::vector<Eigen::MatrixXd>(hom_mat[0].size()));

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
                    normj.push_back(vec);

                    tempf = kp[i].keypoint[match[i][j][p].queryIdx].pt;
                    vec[0] = tempf[0];
                    vec[1] = tempf[1];
                    vec[2] = 1;
                    normi.push_back(vec);

                }

                cv::Matx33f N = maths::Normalize2D(normi);
                cv::cv2eigen(static_cast<cv::Matx33d>(N),normalizeri[i][j]);

                N = maths::Normalize2D(normj);
                cv::cv2eigen(static_cast<cv::Matx33d>(N),normalizerj[i][j]);

            }
        }
    }

    R_approx.normalizerTi = normalizeri;
    R_approx.normalizerTj = normalizerj;

    R_approx = approximate_R(hom_mat,Tr,foc,R_approx);
    //std::cout<<"foc_app: " << R_approx.K[0][1]<<"\n";


    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                Eigen::MatrixXd hom = ret_hom(i,j);

                for (int p = 0;p < match[i][j].size();p++){
                    Eigen::VectorXd temp(3);
                    Eigen::VectorXd measure = Eigen::VectorXd::Zero(6);

                    write_to_eigen(temp, static_cast<cv::Point2d>(kp[j].keypoint[match[i][j][p].trainIdx].pt),2,0);
                    temp[2] = 1;
                    temp = R_approx.normalizerTi[i][j] * temp;
                    measure({0,1,2}) = temp;

                    temp =  hom * temp;
                    temp = temp/temp[2];
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


Eigen::VectorXd d_func(const Eigen::VectorXd &inp_v,const Eigen::VectorXd &par){

    Eigen::Vector3d x;
    x({0,1}) = inp_v;
    x[2] = 1;


    Eigen::Matrix3d K;
    K <<par[0],0,par[4],0,par[0],par[5],0,0,1;


    Eigen::MatrixXd v(3,3);
    v << 0,-par[3],par[2],par[3],0,-par[1],-par[2],par[1],0;

    //cv::Mat h(1, 3, CV_64F, {par[1], par[2], par[3]});
    //cv::Mat v_rotMat;
    //cv::Rodrigues(h,v_rotMat);
   // cv::cv2eigen(v_rotMat,v);

    v = v.exp();

    x = ( K * v * K.inverse() ) * x;

    x = x/x[2];
    Eigen::VectorXd ret(4);
    ret({0,1}) = inp_v;
    ret({2,3}) = x({0,1});

    return ret;
}



Eigen::MatrixXd nummeric_div(const Eigen::VectorXd &inp_vf,const Eigen::VectorXd &parf){

    Eigen::VectorXd inp_v = inp_vf.cast<double>();
    Eigen::VectorXd par = parf.cast<double>();

    float eps = 1e-4;
    Eigen::VectorXd test = d_func(inp_v,par);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(test.size(), par.size());

    for(int i = 0;i < par.size(); i++){

        float h = par[i] * eps;
        Eigen::VectorXd xph = par;
        xph[i] = xph[i] + h;
        Eigen::VectorXd xmh = par;
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


Eigen::Matrix3d get_dR(const Eigen::Vector3d &rot_vec,const Eigen::MatrixXd &rot_mat,int axis,bool transposed){

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

    Eigen::Matrix3d d_rot = ( ( v_x + v_cx )/sq ) * rot_mat;

    if (transposed){d_rot.transposeInPlace();}
    return d_rot;
}


std::vector<Eigen::MatrixXd> parameters::ret_A_i(){

    int size = adj.cols;
    std::vector<Eigen::MatrixXd> A_i;

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                Eigen::MatrixXd hom = ret_hom(i,j);
                Eigen::MatrixXd rot = R_approx.R[i][j];
                Eigen::MatrixXd rot_v = R_approx.R_v[i][j];
                Eigen::MatrixXd foc_inv = R_approx.K_inv[i][j];
                Eigen::MatrixXd foc_ = R_approx.K[i][j];

                Eigen::Matrix3d df;
                df << 1,0,0,0,1,0,0,0,0;

                Eigen::MatrixXd dfocal = ( df * rot * foc_inv - foc_ * rot * foc_inv * df * foc_inv );


                for (int p = 0;p < measurements[i][j].size();p++){

                    Eigen::MatrixXd A_insert = Eigen::MatrixXd::Zero(4,5*size+1);

                    Eigen::VectorXd t = measurements[i][j][p]({0,1,2});
                    Eigen::VectorXd f_i = dfocal * t;
                    Eigen::VectorXd t_tr = hom * t;
                    //Eigen::VectorXf t_tr = measurements[i][j][p]({3,4,5});

                    A_insert.col(0)({0,1})<< 0 , 0;
                    A_insert.col(0)({2,3})<< (f_i[0] * t_tr[2] - t_tr[0] * f_i[2])/(t_tr[2]*t_tr[2]),(f_i[1] * t_tr[2] - t_tr[1] * f_i[2])/(t_tr[2]*t_tr[2]);

                    for (int k = 0;k < 3;k++){

                        Eigen::Matrix3d d_R_j = get_dR(rot_v,rot,k,false);
                        Eigen::Matrix3d Hj = foc_ * d_R_j * foc_inv;
                        Eigen::VectorXd vec_dRj = Hj * t;

                        A_insert.col(5*j+1+k)({0,1})<< 0,0;
                        A_insert.col(5*j+1+k)({2,3})<<(vec_dRj[0] * t_tr[2] - t_tr[0] * vec_dRj[2])/(t_tr[2]*t_tr[2]),(vec_dRj[1] * t_tr[2] - t_tr[0] * vec_dRj[2])/(t_tr[2]*t_tr[2]);

                    }

                    for (int k = 0;k < 2;k++){

                        Eigen::Matrix3d df_t = Eigen::Matrix3d::Zero();
                        df_t(k,2) = 1;

                        Eigen::Matrix3d df_xy_i = df_t * rot * foc_inv - foc_ * rot * foc_inv * df_t * foc_inv;
                        Eigen::VectorXd vec_df_xy_i = df_xy_i * t;

                        A_insert.col(5*j+4+k)({0,1})<< 0,0;
                        A_insert.col(5*j+4+k)({2,3})<<(vec_df_xy_i[0] * t_tr[2] - t_tr[0] * vec_df_xy_i[2])/(t_tr[2]*t_tr[2]),(vec_df_xy_i[1] * t_tr[2] - t_tr[0] * vec_df_xy_i[2])/(t_tr[2]*t_tr[2]);

                    }

                    A_i.push_back(A_insert);

                }

            }

        }
    }

    return A_i;

}


std::vector<Eigen::MatrixXd> parameters::ret_A_i_num(){

    int size = adj.cols;
    std::vector<Eigen::MatrixXd> A_i;

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                Eigen::VectorXd par(6);
                par[0] = R_approx.K[i][j](0,0);
                par[4] = R_approx.K[i][j](0,2);
                par[5] = R_approx.K[i][j](1,2);

                par({1,2,3}) = R_approx.R_v[i][j];

                for (int p = 0;p < measurements[i][j].size();p++){

                    Eigen::MatrixXd A_insert = Eigen::MatrixXd::Zero(4,5*size+1);
                    Eigen::VectorXd t = measurements[i][j][p]({0,1});
                    Eigen::MatrixXd Ainum = nummeric_div(t,par);

                    A_insert({0,1,2,3},{0,1+5*j,2+5*j,3+5*j,4+5*j,5+5*j}) << Ainum;
                    A_i.push_back(A_insert);

                }

            }

        }
    }

    return A_i;

}


void parameters::add_delta(std::vector<Eigen::VectorXd> delta_b,Eigen::VectorXd delta_a){

    R_approx_res = approx();
    std::vector<std::vector<std::vector<Eigen::VectorXd>>>().swap(measurements_res);
    copy(measurements.begin(), measurements.end(), back_inserter(measurements_res));
    R_approx_res = R_approx;

    int size = adj.cols;
    int c1 = 0;

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            Eigen::MatrixXd hom = ret_hom(i, j);

            if (.5 <= adj.at<double>(i,j)){

                //std::cout <<"delta0"<<delta_a[0] <<"\n";
                //std::cout <<"dx"<<delta_a[5*j+4+0] <<"\n";
                //std::cout <<"dy"<<delta_a[5*j+4+1] <<"\n";
                R_approx.K[i][j](0,0) = R_approx.K[i][j](0,0) + delta_a[0];
                R_approx.K[i][j](1,1) = R_approx.K[i][j](1,1) + delta_a[0];
                R_approx.K[i][j](0,2) = R_approx.K[i][j](0,2) + delta_a[5*j+4+0];
                R_approx.K[i][j](1,2) = R_approx.K[i][j](1,2) + delta_a[5*j+4+1];

                R_approx.K[i][j](0,2) = R_approx.K[i][j](0,2) + delta_a[5*j+4+0];
                R_approx.K[i][j](1,2) = R_approx.K[i][j](1,2) + delta_a[5*j+4+1];


                for(int k = 0 ;k < 3 ; k++){
                    std::cout << "deltav" << R_approx.R_v[i][j](k,0)<<"\n";
                    R_approx.R_v[i][j](k,0) = R_approx.R_v[i][j](k,0) + delta_a[5*j+1+k];
                }

                R_approx.K_inv[i][j] = R_approx.K[i][j].inverse().eval();
                Eigen::MatrixXd v(3,3);
                v << 0,-R_approx.R_v[i][j](2,0),R_approx.R_v[i][j](1,0),R_approx.R_v[i][j](2,0),0,-R_approx.R_v[i][j](0,0),-R_approx.R_v[i][j](1,0),R_approx.R_v[i][j](0,0),0;


                //cv::Mat v_rot;
                //cv::Mat v_rotMat;
                //cv::eigen2cv(R_approx.R_v[i][j],v_rot);

                //cv::Rodrigues(v_rot, v_rotMat);
                //cv::cv2eigen(v_rotMat,R_approx.R[i][j]);

                R_approx.R[i][j] = v.exp();

                for (int p = 0;p < measurements[i][j].size();p++){

                    Eigen::VectorXd temp(3);
                    measurements[i][j][p]({0,1}) = measurements[i][j][p]({0,1}) + delta_b[c1];

                    temp({0,1}) = measurements[i][j][p]({0,1});
                    temp[2] = 1;
                    temp = hom * temp;
                    temp = temp / temp[2];
                    measurements[i][j][p]({3,4,5}) = temp;

                    c1++;
                }

            }
        }
    }
}

void parameters::reset(){

    R_approx = approx();
    R_approx = R_approx_res;

    std::vector<std::vector<std::vector<Eigen::VectorXd>>>().swap(measurements);
    copy(measurements_res.begin(), measurements_res.end(), back_inserter(measurements));

}


std::vector<std::vector< cv::Matx33d >> parameters::ret_kmat(){

    std::vector<std::vector<cv::Matx33d>> hom_mat(adj.rows, std::vector<cv::Matx33d>(adj.cols, cv::Matx33d::eye()));

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                Eigen::MatrixXd hom = R_approx.normalizerTi[i][j] * R_approx.K[i][j] * R_approx.normalizerTj[i][j].inverse();
                cv::eigen2cv(hom,hom_mat[i][j]);

            }
        }
    }

    return hom_mat;

}


std::vector<std::vector< cv::Matx33d >> parameters::ret_hmat(){

    //std::vector<std::vector< cv::Matx33f >> hom_mat(adj.rows, std::vector<cv::Matx33f>(adj.cols, cv::Matx33f));
    std::vector<std::vector<cv::Matx33d>> hom_mat(adj.rows, std::vector<cv::Matx33d>(adj.cols, cv::Matx33d::eye()));

    for (int i = 0;i < adj.rows;i++){
        for(int j = i;j < adj.cols;j++){

            if (.5 <= adj.at<double>(i,j)){

                Eigen::MatrixXd hom = ret_hom(i, j);
                hom = R_approx.normalizerTj[i][j].inverse() * hom * R_approx.normalizerTi[i][j];
                cv::eigen2cv(hom,hom_mat[i][j]);

            }
        }
    }

    return hom_mat;

}


}




