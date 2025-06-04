
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <unsupported/Eigen/MatrixFunctions>
#include <opencv2/core/eigen.hpp>
#include "_maths.h"

namespace full {


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


    Eigen::Vector2d transform2ds(Eigen::MatrixXd &hom,Eigen::Vector2d &point){
        Eigen::Vector3d pointT;
        pointT<<point[0],point[1],1;
        pointT = hom*pointT;
        pointT = pointT/pointT[2];

        return pointT({0,1});
    }


    struct parameters{

        cv::Mat adj;

        std::vector<double> x_train;
        std::vector<double> x_query;
        std::vector<double> y_train;
        std::vector<double> y_query;

        std::vector<double> y_train_t;
        std::vector<double> x_train_t;


        std::vector<std::vector<int>> idx;
        Eigen::MatrixXd J;
        Eigen::MatrixXd JtJ;
        Eigen::VectorXd residual;
        std::vector<Eigen::MatrixXd> K;
        std::vector<Eigen::MatrixXd> R;
        std::vector<Eigen::Vector3d> R_vec;

        void Rtovec(){
            std::vector<Eigen::Vector3d> vect;
            for(int i = 0;i<R.size();i++){

                vect.push_back(get_rotvec(R[i]));

            }
            R_vec = vect;
        }


        void transform(){
            std::vector<std::vector<Eigen::MatrixXd>> hom(adj.rows, std::vector<Eigen::MatrixXd>(adj.cols));

            std::vector<double> y_temp;
            std::vector<double> x_temp;

            for (int i = 0;i < adj.rows;i++){
                for(int j = 0;j < adj.cols;j++){

                    if (0.001 <= adj.at<double>(i,j)){

                        hom[i][j]  = K[j] * R[i].transpose() * R[j] * K[i].inverse();

                    }

                }

            }

            for(int n = 0 ;n<idx.size();n++){

                int i = idx[n][0];
                int j = idx[n][1];

                Eigen::Vector2d point;
                point << x_train[n],y_train[n];

                Eigen::Vector2d pointT = transform2ds(hom[i][j],point);

                x_temp.push_back(pointT[0]);
                y_temp.push_back(pointT[1]);

            }

            y_train_t = y_temp;
            x_train_t = x_temp;

        }

    };


    struct keypoints{

        std::vector<cv::KeyPoint> keypoint;
        cv::Mat descriptor;

    };


    Eigen::MatrixXd ret_hom(int i, int j,struct parameters & par){

        Eigen::MatrixXd Hom = par.K[j] * par.R[j] * par.R[i].transpose() * par.K[i].inverse();
        return Hom;
    }


    Eigen::MatrixXd to_K(Eigen::Vector3d kvec){

        Eigen::MatrixXd v(3,3);
        v << kvec[0],0,kvec[1],0,kvec[0],kvec[2],0,0,1;

        return v;
    }


    Eigen::Vector3d to_K_vec(Eigen::MatrixXd &K){

        Eigen::Vector3d kvec;
        kvec << K(0,0),K(0,2),K(1,2);

        return kvec;
    }


    Eigen::Vector2d get_residualsX(int index,struct parameters & par){

        Eigen::Vector2d point;
        point[0] = par.x_query[index] - par.x_train_t[index];
        point[1] = par.y_query[index] - par.y_train_t[index];

        return point;
    }


    Eigen::MatrixXd get_rot(Eigen::Vector3d rotv){

        double eps = 1e-8;
        Eigen::MatrixXd rot(3,3);
        Eigen::Vector3d u;

        double theta = rotv[0]*rotv[0]+rotv[1]*rotv[1]+rotv[2]*rotv[2];
        if(theta < eps){
        rot << 1,-rotv[2],rotv[1],rotv[2],1,-rotv[0],-rotv[1],rotv[0],1;
        return rot;
        }

        theta = sqrt(theta);
        double norm = 1/theta;
        u << rotv[0],rotv[1],rotv[2];
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


    Eigen::MatrixXd par_to_hom(Eigen::Vector3d& kiv,Eigen::Vector3d& kjv,Eigen::Vector3d& riv,Eigen::Vector3d& rjv){

        Eigen::MatrixXd ki = to_K(kiv);
        Eigen::MatrixXd kj = to_K(kjv);
        Eigen::MatrixXd ri = get_rot(riv);
        Eigen::MatrixXd rj = get_rot(rjv);

        Eigen::MatrixXd Hom = kj * rj * ri.transpose() * ki.inverse();

        return Hom;
    }


    void createJakobi(struct parameters & par){

        double eps = 1e-5;

        Eigen::MatrixXd Jadd = Eigen::MatrixXd::Zero(par.J.rows(),par.J.cols());
        Eigen::MatrixXd Jsub = Eigen::MatrixXd::Zero(par.J.rows(),par.J.cols());

        std::vector<Eigen::Vector3d> R_vec = par.R_vec;
        std::vector<Eigen::Vector3d> K_vec;



        for(int n = 0 ;n<par.K.size();n++){

            K_vec.push_back(to_K_vec(par.K[n]));

        }

        for(int n = 0 ;n<par.idx.size();n++){

            int i = par.idx[n][0];
            int j = par.idx[n][1];

            Eigen::VectorXd var(12);
            var<<K_vec[i],R_vec[i],K_vec[j],R_vec[j];

            std::vector<int> ni(6) ;
            std::iota (std::begin(ni), std::end(ni), 6*i);

            std::vector<int> nj(6) ;
            std::iota (std::begin(nj), std::end(nj), 6*j);

            ni.insert( ni.end(), nj.begin(), nj.end() );

            for(int k = 0 ;k<12;k++){
                double temp = var[k];
                var[k] = var[k] + eps;

                Eigen::Vector3d K_veci = var({0,1,2});
                Eigen::Vector3d R_veci = var({3,4,5});
                Eigen::Vector3d K_vecj = var({6,7,8});
                Eigen::Vector3d R_vecj = var({9,10,11});

                Eigen::Vector2d pt;
                Eigen::Vector2d pq;
                pt << par.x_train[n],par.y_train[n];
                pq << par.x_query[n],par.y_query[n];

                Eigen::MatrixXd homm = par_to_hom(K_veci,K_vecj,R_veci,R_vecj);
                pt = transform2ds(homm,pt);
                pt = pq - pt;


                Jadd(2*n,ni[k]) = pt[0];
                Jadd(2*n+1,ni[k]) = pt[1];

                var[k] = temp;
            }

            for(int k = 0 ;k<12;k++){
                double temp = var[k];
                var[k] = var[k] - eps;

                Eigen::Vector3d K_veci = var({0,1,2});
                Eigen::Vector3d R_veci = var({3,4,5});
                Eigen::Vector3d K_vecj = var({6,7,8});
                Eigen::Vector3d R_vecj = var({9,10,11});

                Eigen::Vector2d pt;
                Eigen::Vector2d pq;
                pt << par.x_train[n],par.y_train[n];
                pq << par.x_query[n],par.y_query[n];

                Eigen::MatrixXd homm = par_to_hom(K_veci,K_vecj,R_veci,R_vecj);
                pt = transform2ds(homm,pt);
                pt = pq - pt;

                Jsub(2*n,ni[k]) = pt[0];
                Jsub(2*n+1,ni[k]) = pt[1];

                var[k] = temp;
            }

        }


        par.J = (Jadd - Jsub)/(2*eps);
        par.JtJ = (par.J.transpose()*par.J).eval();

    }

    //iterating nummerically
    std::pair<double,double> iterate(struct parameters & par,double lambda,int id){

        Eigen::VectorXd residual(2*par.idx.size());
        par.transform();
        double before = 0;
        //get residuals and average error
        for(int i = 0;i<par.idx.size();i++){

            Eigen::Vector2d res = get_residualsX(i,par);
            residual[2*i] = res[0];
            residual[2*i+1] = res[1];
            before = before + sqrt(res[0]*res[0]+res[1]*res[1]);

        }
        par.residual = residual;
        before = before / par.idx.size();
        //define jacobian size
        par.J = Eigen::MatrixXd::Zero(2*par.idx.size(), 6 * par.adj.rows);
        par.JtJ = Eigen::MatrixXd::Zero(6 * par.adj.rows, 6 * par.adj.rows);

        double foc = lambda * 1.;
        double rad = lambda*.1;

        Eigen::VectorXd augmvec(6);

        augmvec <<foc,foc,foc,rad,rad,rad;
        //calculate jacobian numerically
        auto dres = par.J.transpose() * residual;
        par.Rtovec();
        createJakobi(par);

        //add lambda terms to diagonal elements of the jacobian as required by the Levenberg-Marquardt algorithm
        for(int i = 0; i < par.adj.rows;i++){
            for(int j = 0 ; j < 6;j++){

                par.JtJ(6*i + j,6*i + j) = par.JtJ(6*i + j,6*i + j) + lambda * augmvec[j];

            }
        }

        //Levenberg-Marquardt equation then add delta terms to rotation and cammera parameters
        Eigen::VectorXd delta = par.JtJ.colPivHouseholderQr().solve(dres).eval();

        for(int i = 0; i < par.adj.rows;i++){

            Eigen::VectorXd k_veciter = to_K_vec(par.K[i]);
            k_veciter = k_veciter - delta({0+6*i,1+6*i,2+6*i});

            par.R_vec[i] = par.R_vec[i] - delta({3+6*i,4+6*i,5+6*i});

            par.R[i] = get_rot(par.R_vec[i]);

            par.K[i] = to_K(k_veciter);


        }
        //check the new error value
        par.transform();
        double after = 0;

        for(int i = 0;i<par.idx.size();i++){

            Eigen::Vector2d res = get_residualsX(i,par);
            after = after + sqrt(res[0]*res[0]+res[1]*res[1]);

        }

        after = after / par.idx.size();

        return {before,after};
    }

    /*main function input :
    *kp: keypoints of each image
    *match: match matrix containing matches between image i and j
    *adjx adjacency matrix adjx(i,j) > 0 indicates matches between images i and j
    *K cammera matrices with a good starting value.
    *R rotation matrices with a good starting value.
    * id /not implemented yet can be ignored
    *
   */

    /*main function output :
    *struct parameters containing optimized rotation and cammera matrices.
    *
   */
   struct parameters main_fkt(const std::vector<maths::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match,std::vector<Eigen::MatrixXd> &K,std::vector<Eigen::MatrixXd> &R,cv::Mat adjx,int id){

        struct parameters par;
        par.K = K;
        par.R = R;
        Eigen::Matrix3d hom_id = Eigen::Matrix3d::Identity();

        cv::Mat adj = adjx + adjx.t();
        par.adj = adj;

        for (int i = 0;i < adj.rows;i++){
            for(int j = 0;j < adj.cols;j++){

                if (0.001 <= adj.at<double>(i,j)){

                    for(int kp_idx = 0; kp_idx < match[i][j].size(); ++kp_idx) {

                        int q_idx = match[i][j][kp_idx].queryIdx; //i
                        int t_idx = match[i][j][kp_idx].trainIdx; //j
                        std::vector<int> idx_push(2);
                        idx_push[0] = i;
                        idx_push[1] = j;
                        par.idx.push_back(idx_push);

                        Eigen::Vector2d point;
                        point << (double)kp[i].keypoint[q_idx].pt.x,(double)kp[i].keypoint[q_idx].pt.y;

                        //point = transform2ds(hom_id,point);

                        par.x_query.push_back(point[0]);
                        par.y_query.push_back(point[1]);

                        par.x_train.push_back(kp[j].keypoint[t_idx].pt.x);
                        par.y_train.push_back(kp[j].keypoint[t_idx].pt.y);

                    }

                }
            }
        }

        int counter = 0;
        double lambda = 5.;
        for(int i = 0;i < 50;i++){

            std::vector<Eigen::MatrixXd> K_save = par.K;
            std::vector<Eigen::MatrixXd> R_save = par.R;
            par.Rtovec();
            std::pair<double,double> loss = iterate(par,lambda,id);

            if(loss.first < loss.second){
                counter++;
                par.R = R_save;
                par.K = K_save;
                lambda = lambda * 10;
            }else{
                lambda = lambda / 10;
                std::cout<<"\n"<<"loss before:  "<<loss.first<<"\n";
                std::cout<<"\n"<<"loss after:  "<<loss.second<<"\n";
            }

            if(counter > 5){
                break;
            }

        }

        return par;
    }



}
