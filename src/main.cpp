
//#include <gtk/gtk.h>
#include <opencv2/opencv.hpp>
#include "_panorama.h"
#include "_maths.h"
#include "_img_manipulation.h"
#include "_image.h"
#include "_gtk_vars.h"
#include "_main_windows.h"
#include "_bundle_adjust_tools.h"
#include <Eigen/Dense>
//#include "_gain_compensation.h"
#include "_blending.h"
#include <opencv2/core/eigen.hpp>
#include "_bundle_adjust_main.h"

namespace fs = std::filesystem;
/*
//struct main_window_ main_window;
*/

struct approx{

    std::vector<std::vector<Eigen::Matrix3f>> K;
    std::vector<std::vector<Eigen::Matrix3f>> R;
    std::vector<std::vector<Eigen::MatrixXf>> R_v;

};

double cv_frobeniusNorm(const cv::Matx33d &mat){

    cv::Matx33d output = mat.mul(mat);

    cv::Scalar sum = cv::sum(output);
    cv::sqrt(sum,sum);

    return sum[0];
}


struct approx approximate_R(const std::vector<std::vector< cv::Matx33f >> &hom_mat,const imgm::pan_img_transform &Tr,float focal){

    struct approx H_approx;
    std::vector<std::vector<Eigen::Matrix3f>> rot_approx(hom_mat.size(), std::vector<Eigen::Matrix3f>(hom_mat[0].size(), Eigen::Matrix3f::Identity()));

    std::vector<std::vector<Eigen::Matrix3f>> foc_approx(hom_mat.size(), std::vector<Eigen::Matrix3f>(hom_mat[0].size(), Eigen::Matrix3f::Identity()));

    std::vector<std::vector<Eigen::MatrixXf>> rotV_approx(hom_mat.size(), std::vector<Eigen::MatrixXf>(hom_mat[0].size()));



    for (int i = 0;i < (*Tr.adj).rows;i++){
        for(int j = i;j < (*Tr.adj).cols;j++){

            if (.5 <= (*Tr.adj).at<double>(i,j)){

                cv::Matx33d homij(hom_mat[i][j]);

                struct maths::translation trans = maths::get_translation((*Tr.img_address)[i], (*Tr.img_address)[j],hom_mat[i][j]);
                cv::Matx33d foca(focal, 0, 0,
                                0, focal, 0,
                                0, 0, 1);

                cv::Matx33d trs(trans.T);
                foca(0,2) = trs(0,2);
                foca(1,2) = trs(1,2);

                std::vector<cv::Mat> Rs;
                std::vector<cv::Mat> Ts;
                cv::decomposeHomographyMat(hom_mat[i][j], foca, Rs, Ts, cv::noArray());

                cv::Matx33f focaf = static_cast<cv::Matx33f>(foca);
                cv::cv2eigen(focaf,foc_approx[i][j]);

                cv::Mat homap_0 = homij - foca * Rs[0] * foca.inv();
                cv::Mat homap_1 = homij - foca * Rs[2] * foca.inv();

                double h0 = cv_frobeniusNorm(homap_0);
                double h1 = cv_frobeniusNorm(homap_1);

                for(int i = 0;i < Rs.size();i++){

                    cv::Mat homap_0 = foca * Rs[i] * foca.inv();
                    double s = homij.dot(homap_0) / homij.dot(homap_0);
                    cv::Mat hom_est_scaled = homap_0 * s;
                    double h_dif = cv::norm(homij-hom_est_scaled, cv::NORM_L2);
                    std::cout <<"dif "<<h_dif<<"\n";
                }

                cv::Matx33f rotf =  (h0 < h1) ? static_cast<cv::Matx33f>(Rs[2]) : static_cast<cv::Matx33f>(Rs[0]);
                cv::cv2eigen(rotf,rot_approx[i][j]);

                cv::Mat rot_v;
                cv::Rodrigues(rotf, rot_v);
                cv::cv2eigen(rot_v,rotV_approx[i][j]);

            }
        }
    }

    H_approx.K = foc_approx;
    H_approx.R = rot_approx;
    H_approx.R_v = rotV_approx;

    return H_approx;

}

Eigen::VectorXd d_func(const Eigen::VectorXd &inp_v,const Eigen::VectorXd &par){

    Eigen::Vector3d x;
    x({0,1}) = inp_v;
    x[2] = 1;


    Eigen::Matrix3d K;
    K <<par[0],0,par[4],0,par[0],par[5],0,0,1;


    Eigen::MatrixXd v(3,3);
    v << 0,-par[3],par[2],par[3],0,-par[1],-par[2],par[1],0;
    v = v.exp();

    x = ( K * v * K.inverse() ) * x;

    x = x/x[2];
    Eigen::VectorXd ret(4);
    ret({0,1}) = inp_v;
    ret({2,3}) = x({0,1});

    return ret;
}


Eigen::MatrixXf nummeric_div(const Eigen::VectorXf &inp_vf,const Eigen::VectorXf &parf){

    Eigen::VectorXd inp_v = inp_vf.cast<double>();
    Eigen::VectorXd par = parf.cast<double>();

    float eps = 1e-5;
    Eigen::VectorXd test = d_func(inp_v,par);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(test.size(), par.size());

    for(int i = 0;i < par.size(); i++){

        volatile float h = par[i] * eps;
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

    Eigen::MatrixXf ret = D.cast<float>();
    return ret;

}


int main(int argc, char **argv) {

            std::string path = "/home/sd_bert/projects/Panorama/test";
            std::vector<std::string> path_list;

            for (const auto & entry : fs::directory_iterator(path)){
                std::string temp_string = entry.path().string();

                path_list.push_back(temp_string);

            }


            img::images test(path_list);
            test.load_images();
            //test.images_to_cylinder(900);
            test.calculate_keypoints(1);


            std::vector<maths::keypoints> key_p = test.get_keypoints();

            //std::cout <<key_p.size();

            std::vector<cv::Mat> imags = test.get_images();

            //cv::Mat tetetst = imgm::project(imags[0],400,150);




            std::cout << "preprocessing done"<<"\n";
            maths::graph_thread::set_mat(imags,key_p);
            maths::thread threads = maths::graph_thread::get_threads(1);

            maths::graph_thread t1;
            //maths::graph_thread t2;
            //maths::graph_thread t3;

            std::thread thread_obj1(&maths::graph_thread::cal_adj, &t1, imags,threads[0]);
            //std::thread thread_obj2(&maths::graph_thread::cal_adj, &t2, imags,threads[1]);
            //std::thread thread_obj3(&maths::graph_thread::cal_adj, &t3, imags,threads[2]);

            thread_obj1.join();
            //thread_obj2.join();
            //thread_obj3.join();

            cv::Mat adj = t1.return_adj_mat();
            std::vector<std::vector< cv::Matx33f >> hom_mat = t1.return_Hom_mat();
            std::vector<std::vector<std::vector<cv::DMatch>>> matchMat = t1.return_match_mat();
            float focal = 1000;
            float focal2 = focal;
            imgm::pan_img_transform Tr(&adj,&imags);
            Tr.focal = 1000;
            imgm::calc_stitch_from_adj(Tr,hom_mat);

            double labda = .0001;
            test.images_to_cylinder(900);
            imags = test.get_images();
            class bundm::adjuster testad(key_p,matchMat,adj,800,labda,hom_mat,Tr);
            struct bundm::inter_par teees = testad.iterate();

            cv::Mat panorama = imgm::stitch(imags[0], imags[1],teees.hom[0][1]);
            cv::imshow("Image Display", panorama);
            cv::waitKey(0);
            //struct maths::translation trans = maths::get_translation(imags[0], imags[1],hom_mat[0][1]);
            //class bund::parameters sanity(key_p,matchMat,adj,1200,hom_mat,Tr);
            //std::vector<std::vector< cv::Matx33d >> hom_b = sanity.ret_hmat();

            //cv::Mat panorama = imgm::stitch(imags[0], imags[1],homtest);
            //cv::imshow("Image Display", panorama);
            //cv::waitKey(0);

/*


            std::cout <<"translation "<<homtest<<"\n";
            homtest = K * homtest * K2.inv();

            //cv::Mat rot = tests * sqqr;
            cv::Mat panorama = imgm::stitch(imags[0], imags[1],homtest);
            cv::imshow("Image Display", panorama);
            cv::waitKey(0);


            std::vector<cv::Mat> Rs;
            std::vector<cv::Mat> Ts;

            cv::Matx33d foca(focal, 0, 0,
                0, focal, 0,
                0, 0, 1);

            cv::decomposeHomographyMat(hom_mat[0][1], foca, Rs, Ts, cv::noArray());

            std::cout <<"translation "<<Ts[2]<<"\n";

            //cv::Mat panorama = imgm::stitch(imags[0], imags[1],hom_mat[0][1]);

            //cv::imshow("Image Display", panorama);
            //cv::waitKey(0);


            class bund::parameters sanity(key_p,matchMat,adj,1200,hom_mat,Tr);

            class bund::parameters sanity(key_p,matchMat,adj,1200,hom_mat,Tr);
            class bund::E_func err(key_p,matchMat,adj);
            std::vector<Eigen::MatrixXd> Bif = sanity.ret_B_i_num();
            std::vector<Eigen::MatrixXd> Bi = sanity.ret_B_i();
            std::vector<Eigen::MatrixXd> Ai = sanity.ret_A_i();
            std::vector<Eigen::MatrixXd> Aif = sanity.ret_A_i_num();
            std::cout<<"\n"<<"sanitynum: " <<Aif[0]<<"\n";
            std::cout<<"\n"<<"sanityex: " <<Ai[0]<<"\n";

            cv::Mat panorama;

            class bundm::adjuster testad(key_p,matchMat,adj,800,labda,hom_mat,Tr);

            panorama = imgm::stitch(imags[0], imags[1],testad.iter.hom[0][1]);
            cv::imshow("Image Display", panorama);
            cv::waitKey(0);

            struct bundm::inter_par testpars = testad.iterate();
            panorama = imgm::stitch(imags[0], imags[1],testpars.hom[0][1]);
            cv::imshow("Image Display", panorama);
            cv::waitKey(0);

            std::vector<std::vector< cv::Matx33d >> kmat = testad.ret_k();

            std::cout <<"kmat "<<kmat[0][1]<<"\n";

            class bund::E_func err(key_p,matchMat,adj);
            struct bund::approx app = sanity.ret_all();
            std::vector<Eigen::VectorXf> ms = sanity.ret_measurements();
            std::vector<Eigen::VectorXf> ervf = err.error(ms,app);



            Eigen::VectorXf sum = Eigen::VectorXf::Zero(4);
            float smmm = 0;
            //std::cout<<"summe: " <<ervf[100]<<"\n";

            for (int i = 0;i<ervf.size();i++){
                smmm = smmm + ervf[i].norm();

            }

            std::cout<<"summe: " <<smmm<<"\n";



            class bundm::adjuster testad(Tr.H_1j,key_p,matchMat,adj,1000,labda,hom_mat,Tr);
            struct bundm::inter_par testpars = testad.iterate();


            class bundm::adjuster testad(key_p,matchMat,adj,1000,labda,hom_mat,Tr);
            struct bundm::inter_par testpars = testad.iterate();
            cv::Mat panorama = imgm::stitch(imags[0], imags[1],testpars.hom[0][1]);
            std::vector<std::vector< cv::Matx33d >> kmat = testad.ret_k();

            std::cout <<"\n"<< kmat[0][1]<<"\n";

            cv::imshow("Image Display", panorama);
            cv::waitKey(0);




            std::vector<Eigen::MatrixXd> Ai;
            std::vector<Eigen::MatrixXd> Bi;
            for (int i = 0;i<Bif.size();i++){
                Ai.push_back(Aif[i].cast<double>());
                Bi.push_back(Bif[i].cast<double>());

            }



            Eigen::MatrixXd U = Eigen::MatrixXd::Zero(Ai[0].cols(),Ai[0].cols());

            for(int i = 0;i<Ai.size();i++){

               U = U + Ai[i].transpose() * Ai[i];

            }

            std::vector<Eigen::MatrixXd> Vi;

            for(int i = 0;i<Ai.size();i++){

             Vi.push_back( Bi[i].transpose() * Bi[i] );
             Vi[i].diagonal() = Vi[i].diagonal() * (1 + labda);

            }




            std::vector<Eigen::MatrixXd> Wi;

            for(int i = 0;i<Ai.size();i++){

                Wi.push_back( Ai[i].transpose() * Bi[i] );

            }

            std::vector<Eigen::VectorXf> ms = sanity.ret_measurements();
            struct bund::approx app = sanity.ret_all();
            std::vector<Eigen::VectorXf> ervf = err.error(ms,app);
            std::vector<Eigen::VectorXd> erv;

            for (int i = 0;i<ervf.size();i++){
                erv.push_back(ervf[i].cast<double>());

            }

            Eigen::VectorXd ei = Eigen::VectorXd::Zero(Ai[0].cols());

            for(int i = 0;i<Ai.size();i++){

                ei = ei + Ai[i].transpose() * erv[i];

            }

            std::vector<Eigen::VectorXd> bei;

            for(int i = 0;i<Bi.size();i++){

                bei.push_back( Bi[i].transpose() * erv[i] );

            }

            std::vector<Eigen::MatrixXd> Yi;

            for(int i = 0;i<Bi.size();i++){

                Yi.push_back( Wi[i] * Vi[i].inverse() );

            }


            U.diagonal() = U.diagonal() * (1 + labda);



            Eigen::VectorXd tsum = Eigen::VectorXd::Zero(Yi[0].rows());

            for(int i = 0;i<bei.size();i++){

               tsum = tsum + Yi[i] * bei[i];

            }

            tsum = ei - tsum;


            Eigen::MatrixXd lsum = Eigen::MatrixXd::Zero(Wi[0].rows(),Yi[0].rows());
            for(int i = 0;i<bei.size();i++){

               lsum = lsum + Yi[i] * Wi[i].transpose() ;

            }
            lsum = U - lsum;


            Eigen::VectorXd da = lsum.colPivHouseholderQr().solve(tsum );
            std::cout<<"ei: " << da <<"\n";



            int I = 1;
int J = 2;

            class bundm::adjuster testad(Tr.H_1j,key_p,matchMat,adj,1000,labda,hom_mat,Tr);
            struct bundm::inter_par testpars = testad.iterate();
            std::vector<std::vector< cv::Matx33f >> k = testad.ret_k();

            std::cout<<"k :" <<k[1][2]<<"\n";

            cv::Mat panorama = imgm::stitch(imags[0], imags[2],testpars.hom[0][1]*testpars.hom[1][2]);

            cv::imshow("Image Display", panorama);
            cv::waitKey(0);

            //imgm::calc_stitch_from_adj(Tr,testpars.hom);
            //blnd::simple_blend(Tr,imags);



            Eigen::MatrixXf homap =  appr.K[I][J] * appr.R[I][J] * appr.K[I][J].inverse();
            cv::Mat Homeo;
            cv::eigen2cv(homap,Homeo);

            cv::Mat panorama = imgm::stitch(imags[1], imags[2],Homeo);
            cv::imshow("Image Display", panorama);
            cv::waitKey(0);

            //approximate_R(hom_mat,Tr,1000);




            cv::imshow("Image Display", panorama);
            cv::waitKey(0);


            class bund::parameters sanity(Tr.H_1j,key_p,matchMat,adj,900,hom_mat);
            std::vector<std::vector< cv::Matx33f >> hom_b = sanity.ret_hmat();

            class bundm::adjuster testad(Tr.H_1j,key_p,matchMat,adj,800,labda,hom_mat,Tr);
            struct bundm::inter_par testpars = testad.iterate();

            imgm::pan_img_transform Tr2(adj);
            Tr2 = imgm::calc_stitch_from_adj(imags,testpars.hom ,adj);
            blnd::simple_blend(Tr2,imags);



            std::cout<<"point1" <<"\n"<< matchMat[0][2][10].queryIdx<<"\n";
            std::cout<<"point1" <<"\n"<< key_p[0].keypoint[matchMat[0][2][10].queryIdx].pt <<"\n";

            std::cout<<"point2" <<"\n"<< matchMat[0][2][10].queryIdx<<"\n";
            std::cout<<"point2" <<"\n"<< key_p[2].keypoint[33].pt <<"\n";

            cv::Vec3f poin = {205,285,1};
            cv::circle(imags[0], key_p[0].keypoint[matchMat[0][2][10].queryIdx].pt,20, cv::Scalar(255,255,255),10, 8,0);

            poin = hom_mat[2][0]*poin;
            poin = poin/poin[2];
            std::cout<<"point" <<"\n"<< poin<<"\n";

            cv::circle(imags[2], key_p[2].keypoint[matchMat[0][2][10].trainIdx].pt,20, cv::Scalar(255,255,255),10, 8,0);
            cv::imshow("Image Display",imags[2]);
            cv::waitKey(0);



            imgm::pan_img_transform Tr(adj);

            blnd::simple_blend(Tr,imags);







        build_window(argc,argv,&main_window);
        gtk_main();
*/
}
