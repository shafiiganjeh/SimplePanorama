#include "_bundle_adjust_tools.h"

namespace bund {

using Eigen::MatrixXf;
using Eigen::VectorXf;


void write_to_eigen(Eigen::VectorXd &v,const cv::Vec2d &cv_v,int n,int st){

    for(int i = 0;i < n;i++){

        v[st+i] = cv_v[i];

    }

}


E_func::E_func(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const cv::Mat &adjm){

    cv::Mat adj = adjm  + adjm.t();
    for (int i = 0;i < adj.rows;i++){
        for(int j = 0;j < adj.cols;j++){

            if (0 < adj.at<double>(i,j)){

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

    double eps = 1e-8;
    Eigen::MatrixXd rot(3,3);
    Eigen::Vector3d u;

    double theta = rotv(0,0)*rotv(0,0)+rotv(1,0)*rotv(1,0)+rotv(2,0)*rotv(2,0);
    if(theta < eps){
    rot << 1,-rotv(2,0),rotv(1,0),rotv(2,0),1,-rotv(0,0),-rotv(1,0),rotv(0,0),1;
    return rot;
    }

    theta = sqrt(theta);
    double norm = 1/theta;
    u << rotv(0,0),rotv(1,0),rotv(2,0);
    u = u*norm;

    Eigen::Matrix3d K;
    K << 0, -u.z(), u.y(),
         u.z(), 0, -u.x(),
         -u.y(), u.x(), 0;

    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    const Eigen::Matrix3d K_squared = K * K;

    return Eigen::Matrix3d::Identity() + sin_theta * K + (1 - cos_theta) * K_squared;
}


Eigen::Vector3d get_rotvec(Eigen::MatrixXd rotm){

    Eigen::Vector3d v;

    double eps = 1e-8;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(rotm, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d R_solved = svd.matrixU() * (svd.matrixV().transpose());

    if (R_solved.determinant() < 0){
        R_solved *= -1;
    }

    v << R_solved(2,1) - R_solved(1,2),R_solved(0,2) - R_solved(2,0),R_solved(1,0) - R_solved(0,1);

	double s = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

	if (s < eps) {
		v = Eigen::Vector3d::Zero();
	} else {

		double cos_ = (R_solved(0,0) + R_solved(1,1) + R_solved(2,2) - 1) * 0.5;
		cos_ = cos_ > 1. ? 1. : cos_ < -1. ? -1. : cos_;
		double theta = acos(cos_);

		double mul = 1.0 / s * theta;
		v[0] *= mul; v[1] *= mul; v[2] *= mul;
	}
	return v;
}

Eigen::MatrixXd parameters::ret_hom(int i, int j){

    Eigen::MatrixXd Hom = K[j] * rot[i].transpose() * rot[j] * K_inv[i];
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

parameters::parameters(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,const class imgm::pan_img_transform &T,int threads){

    //double focal = T.focal;

    adj = *T.adj;
    adj = adj  + adj.t();
    float eps = 5e-5;

    adjust_yn.resize(adj.rows);
    std::fill(adjust_yn.begin(), adjust_yn.end(), true );

    //std::cout<<"adj :"<< adj<<" \n";

    for(int i = 0 ; i < T.rot.size();i++){

        rot.push_back(T.rot[i]);
        rot_vec.push_back(get_rotvec(T.rot[i]));

        Eigen::Vector2d pvec;
        pvec <<T.K[i](0,2),T.K[i](1,2);
        principal_vec.push_back(pvec);
        focal.push_back(T.K[i](0,0));
        K.push_back(T.K[i]);
        K_inv.push_back(T.K[i].inverse().eval());

    }

    std::vector<std::vector<std::vector<Eigen::VectorXd>>> measure_mat(adj.rows, std::vector<std::vector<Eigen::VectorXd>>(adj.cols));
    std::vector<int> idx(2);

    for (int i = 0;i < adj.rows;i++){
        for(int j = 0;j < adj.cols;j++){

            if (0 <  adj.at<double>(i,j)){
                Eigen::MatrixXd hom = ret_hom(i, j);

                for (int p = 0;p < match[i][j].size();p++){
                    Eigen::VectorXd temp(3);
                    Eigen::VectorXd measure = Eigen::VectorXd::Zero(6);

                    write_to_eigen(measure, static_cast<cv::Point2d>(kp[j].keypoint[match[i][j][p].trainIdx].pt),2,0);
                    measure[2] = 1;

                    write_to_eigen(temp, static_cast<cv::Point2d>(kp[j].keypoint[match[i][j][p].trainIdx].pt),2,0);
                    temp[2] = 1;
                    temp = hom * temp;
                    temp = temp / temp[2];
                    measure({3,4,5}) = temp;


                    measure_mat[i][j].push_back(measure);

                }
            }
        }
    }

    measurements = measure_mat;

    std::vector<int> thread_idx(3);

    for (int i = 0;i < adj.rows;i++){
        for(int j = 0;j < adj.cols;j++){

            if (0 < adj.at<double>(i,j)){

                for (int p = 0;p < measurements[i][j].size();p++){

                    thread_idx[0] = i;
                    thread_idx[1] = j;
                    thread_idx[2] = p;

                    thread_parts.push_back(thread_idx);

                }
            }
        }
    }

    threads_vector = maths::splitVector(thread_parts, threads);
    std::vector<int> sizes(threads_vector.size());
    sizes[0] = 0;
    int c = 0;
    for(int s = 1;s < sizes.size();s++){
        c = c + threads_vector[s - 1].size();
        sizes[s] = c;
    }

    thread_sizes = sizes;

}


std::vector<Eigen::VectorXd> parameters::ret_measurements(){

    std::vector<Eigen::VectorXd> M;

    for (int i = 0;i < adj.rows;i++){
        for(int j = 0;j < adj.cols;j++){

            if (0 < adj.at<double>(i,j)){

                for (int p = 0;p < measurements[i][j].size();p++){
                    M.push_back(measurements[i][j][p]);
                }
            }
        }
    }

    return M;
}


void parameters::calc_B_i(int thread,const std::vector<std::vector<Eigen::MatrixXd>> hom_mat,std::vector<Eigen::MatrixXd> &B_i){

    std::vector<std::vector<int>> thread_part = threads_vector[thread];

    for(int k = 0;k < thread_part.size();k++){

        int i = thread_part[k][0];
        int j = thread_part[k][1];
        int p = thread_part[k][2];

        Eigen::MatrixXd B_insert = Eigen::MatrixXd::Zero(4,2);
        B_insert({0,1},{0,1}) = Eigen::MatrixXd::Identity(2,2);
        Eigen::VectorXd t = measurements[i][j][p]({0,1,2});
        Eigen::VectorXd t_tr = hom_mat[i][j]*t;

        Eigen::MatrixXd B1(2,3);
        B1 << 1/t_tr[2],0,-t_tr[0]/(t_tr[2]*t_tr[2]),0,1/t_tr[2],-t_tr[1]/(t_tr[2]*t_tr[2]);
        B1 = B1 *  hom_mat[i][j]({0,1,2},{0,1});

        B_insert({2,3},{0,1}) = B1;

        B_i[k + thread_sizes[thread]] = B_insert;

    }

}


std::vector<Eigen::MatrixXd> parameters::ret_B_i(){

    std::vector<Eigen::MatrixXd> B_i(thread_parts.size());
    std::vector<std::vector<Eigen::MatrixXd>> hom_mat(adj.rows,std::vector<Eigen::MatrixXd>(adj.cols));

    for (int i = 0;i < adj.rows;i++){
        for(int j = 0;j < adj.cols;j++){

            if (0 < adj.at<double>(i,j)){

                hom_mat[i][j] = ret_hom(i,j);

            }
        }
    }

    //hier multithread

    std::vector<std::thread> thread_objects;

    for(int k = 0;k < threads_vector.size();k++){

        std::thread tobj(&parameters::calc_B_i,this,k,hom_mat,std::ref(B_i));
        thread_objects.push_back(std::move(tobj));

    }

    for(int k = 0;k < threads_vector.size();k++){

        thread_objects[k].join();

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
        for(int j = 0;j < adj.cols;j++){

            if (0 < adj.at<double>(i,j)){

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

    double eps = 1e-6;

    const double sq = rot_vec.squaredNorm();
    if(sq < eps){

        return Eigen::Matrix3d::Identity();

    }

    Eigen::Vector3d e_i = Eigen::Vector3d::Unit(axis);

    const Eigen::Vector3d rot_col = rot_mat.col(axis);
    const Eigen::Vector3d temp = e_i - rot_col;  // Verhindere matrix multiplication

    const Eigen::Vector3d v_cross = rot_vec.cross(temp);
    const Eigen::Vector3d scaled_rot_vec = rot_vec[axis] * rot_vec;

    Eigen::Matrix3d d_rot = Eigen::Matrix3d::Zero();
    d_rot.row(0) = Eigen::Vector3d(0, -scaled_rot_vec.z() - v_cross.z(), scaled_rot_vec.y() + v_cross.y());
    d_rot.row(1) = Eigen::Vector3d(scaled_rot_vec.z() + v_cross.z(), 0, -scaled_rot_vec.x() - v_cross.x());
    d_rot.row(2) = Eigen::Vector3d(-scaled_rot_vec.y() - v_cross.y(), scaled_rot_vec.x() + v_cross.x(), 0);

    d_rot = (d_rot * rot_mat) / sq;

    if (transposed){d_rot.transposeInPlace();}
    return d_rot;
}


void parameters::calc_A_i(int thread,int size,const std::vector<std::vector<Eigen::MatrixXd>> hom_mat,std::vector<Eigen::Matrix3d> D,std::vector<A_vec> &A_i){

    const auto& thread_part = threads_vector[thread];

    for(int km = 0;km < thread_part.size();km++){

        const int idx = km + thread_sizes[thread];
        int i = thread_parts[km + thread_sizes[thread]][0];
        int j = thread_parts[km + thread_sizes[thread]][1];
        int p = thread_parts[km + thread_sizes[thread]][2];

        const Eigen::Matrix3d& hom = hom_mat[i][j];
        Eigen::VectorXd t = measurements[i][j][p].head<3>();
        Eigen::VectorXd t_tr = hom * t;
        const double t_tr2 = t_tr[2];
        const double denom = 1.0 / (t_tr2 * t_tr2);

        const Eigen::Matrix3d K_inv_i = K_inv[i];
        const Eigen::Matrix3d N = rot[i].transpose() * rot[j] * K_inv_i;
        const Eigen::Matrix3d M = K[j] * N;
        const Eigen::Matrix3d P = rot[j] * K_inv_i;
        const Eigen::Matrix3d Q = K[j] * rot[i].transpose();


        Eigen::MatrixXd A_insert_i = Eigen::MatrixXd::Zero(4,D.size() + 3);
        Eigen::MatrixXd A_insert_j = Eigen::MatrixXd::Zero(4,D.size() + 3);

        const double t0 = t_tr[0];
        const double t1 = t_tr[1];
        Eigen::Vector3d f_i;
        double x_comp, y_comp;

        for (int d = 0; d < D.size(); d++) {

            f_i.noalias() = (M * D[d] * K_inv_i * (-1.0)) * t;
            x_comp = (f_i[0] * t_tr2 - t0 * f_i[2]) * denom;
            y_comp = (f_i[1] * t_tr2 - t1 * f_i[2]) * denom;
            A_insert_i(2, d) = x_comp;
            A_insert_i(3, d) = y_comp;

            f_i.noalias() = (D[d] * N) * t;
            x_comp = (f_i[0] * t_tr2 - t0 * f_i[2]) * denom;
            y_comp = (f_i[1] * t_tr2 - t1 * f_i[2]) * denom;
            A_insert_j(2, d) = x_comp;
            A_insert_j(3, d) = y_comp;

        }

        for (int k = 0; k < 3; k++) {

            Eigen::Matrix3d Hi = K[j] * (get_dR(rot_vec[i], rot[i], k, true) * P);
            f_i.noalias() = Hi * t;
            x_comp = (f_i[0] * t_tr2 - t0 * f_i[2]) * denom;
            y_comp = (f_i[1] * t_tr2 - t1 * f_i[2]) * denom;
            A_insert_i(2, D.size() + k) = x_comp;
            A_insert_i(3, D.size() + k) = y_comp;

            Hi = Q * (get_dR(rot_vec[j], rot[j], k, false) * K_inv_i);
            f_i.noalias() = Hi * t;
            A_insert_j(2, D.size() + k) = (f_i[0] * t_tr2 - t0 * f_i[2]) * denom;
            A_insert_j(3, D.size() + k) = (f_i[1] * t_tr2 - t1 * f_i[2]) * denom;
        }

        A_i[idx].idx_i = i;
        A_i[idx].idx_j = j;
        A_i[idx].Ai = A_insert_i;
        A_i[idx].Aj = A_insert_j;
        A_i[idx].size = size;

    }

}


std::vector<A_vec> parameters::ret_A_i(){

    int size = rot_vec.size();
    //std::vector<Eigen::MatrixXd> A_i(thread_parts.size());
    std::vector<A_vec> A_i(thread_parts.size());

    std::vector<std::vector<Eigen::MatrixXd>> hom_mat(adj.rows,std::vector<Eigen::MatrixXd>(adj.cols));

    for (int i = 0;i < adj.rows;i++){
        for(int j = 0;j < adj.cols;j++){

            if (0 < adj.at<double>(i,j)){

                hom_mat[i][j] = ret_hom(i,j);

            }
        }
    }

    Eigen::Matrix3d df;
    df << 1,0,0,0,1,0,0,0,0;

    Eigen::Matrix3d dx;
    dx << 0,0,1,0,0,0,0,0,0;

    Eigen::Matrix3d dy;
    dy << 0,0,0,0,0,1,0,0,0;

    std::vector<Eigen::Matrix3d> D = {df,dx,dy};

    //hier multithread

    std::vector<std::thread> thread_objects;

    for(int k = 0;k < threads_vector.size();k++){

        std::thread tobj(&parameters::calc_A_i,this,k,size,hom_mat,D,std::ref(A_i));
        thread_objects.push_back(std::move(tobj));

    }

    for(int k = 0;k < threads_vector.size();k++){

        thread_objects[k].join();

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

    Eigen::MatrixXd Hom = Kj * Ri.transpose() * Rj * Ki_inv;

    x = Hom * x;
    x = x/x[2];

    Eigen::VectorXd ret(4);
    ret({0,1}) = inp_v;
    ret({2,3}) = x({0,1});

    return ret;
}



Eigen::MatrixXd nummeric_div(const Eigen::VectorXd &inp_vf,const Eigen::VectorXd &parf){

    Eigen::VectorXd inp_v = inp_vf.cast<double>();

    double eps = 1e-6;
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


void parameters::calc_A_i_num(int thread,int size,std::vector<A_vec> &A_i){

    std::vector<std::vector<int>> thread_part = threads_vector[thread];

    for(int km = 0;km < thread_part.size();km++){

        const int idx = km + thread_sizes[thread];
        int i = thread_parts[km + thread_sizes[thread]][0];
        int j = thread_parts[km + thread_sizes[thread]][1];
        int p = thread_parts[km + thread_sizes[thread]][2];

        Eigen::VectorXd par(12);
        par[0] = focal[i];
        par[1] = principal_vec[i][0];
        par[2] = principal_vec[i][1];
        par({3,4,5}) = rot_vec[i]({0,1,2},{0});

        par[6] = focal[j];
        par[7] = principal_vec[j][0];
        par[8] = principal_vec[j][1];
        par({9,10,11}) = rot_vec[j]({0,1,2},{0});

        Eigen::VectorXd t = measurements[i][j][p]({0,1});
        Eigen::MatrixXd Ainum = nummeric_div(t,par);
        Eigen::MatrixXd A_insert_i = Eigen::MatrixXd::Zero(4,6);
        Eigen::MatrixXd A_insert_j = Eigen::MatrixXd::Zero(4,6);

        A_insert_i << Ainum({0,1,2,3},{0,1,2,3,4,5});
        A_insert_j << Ainum({0,1,2,3},{6,7,8,9,10,11});

        A_i[idx].idx_i = i;
        A_i[idx].idx_j = j;
        A_i[idx].Ai = A_insert_i;
        A_i[idx].Aj = A_insert_j;
        A_i[idx].size = size;

        }

    }


std::vector<A_vec> parameters::ret_A_i_num(){

    int size = rot_vec.size();

    std::vector<A_vec> A_i(thread_parts.size());

    std::vector<std::thread> thread_objects;

    for(int k = 0;k < threads_vector.size();k++){

        std::thread tobj(&parameters::calc_A_i_num,this,k,size,std::ref(A_i));
        thread_objects.push_back(std::move(tobj));

    }

    for(int k = 0;k < threads_vector.size();k++){

        thread_objects[k].join();

    }

    return A_i;

}



void parameters::add_delta(std::vector<Eigen::VectorXd> delta_b,Eigen::VectorXd delta_a,bool add_rot){
    double eps = 1e-6;

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


    for(int i = 0;i < rot.size();i++){
        if(!(rot[i].isIdentity(eps))){

            rot_vec[i](0,0) = rot_vec[i](0,0) + delta_a[3+i*6];
            rot_vec[i](1,0) = rot_vec[i](1,0) + delta_a[4+i*6];
            rot_vec[i](2,0) = rot_vec[i](2,0) + delta_a[5+i*6];

            rot[i] = get_rot(rot_vec[i]);

        }

            focal[i] = focal[i] + delta_a[i*6];
            principal_vec[i] = principal_vec[i] + delta_a({1+i*6,2+i*6});

        K[i] = to_K(focal[i],principal_vec[i]);
        K_inv[i] = to_K_inv(focal[i],principal_vec[i]);


    }

    int c = 0;
    for (int i = 0;i < adj.rows;i++){
        for(int j = 0;j < adj.cols;j++){

            if (0 < adj.at<double>(i,j)){

                Eigen::MatrixXd hom = ret_hom(i, j);
                for (int p = 0;p < measurements[i][j].size();p++){

                    Eigen::VectorXd temp(3);
                    measurements[i][j][p]({0,1}) = measurements[i][j][p]({0,1}) + delta_b[c];

                    temp({0,1}) = measurements[i][j][p]({0,1});
                    temp[2] = 1;
                    temp = hom * temp;
                    temp = temp/temp[2];
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

        for(int j = 0;j < adj.cols;j++){

            Eigen::MatrixXd hom = ret_hom(i, j);
            Eigen::MatrixXd hom_inv = ret_hom(j, i);
            Eigen::MatrixXf homf = hom.cast <float> ();
            Eigen::MatrixXf homfinv = hom_inv.cast <float> ();

            cv::eigen2cv(homf,hom_mat[i][j]);
            cv::eigen2cv(homfinv,hom_mat[j][i]);

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
