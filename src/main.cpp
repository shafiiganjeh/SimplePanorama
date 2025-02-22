
#include <gtk/gtk.h>
#include <opencv2/opencv.hpp>
#include "_panorama.h"
#include "_maths.h"
#include "_img_manipulation.h"
#include "_image.h"
#include "_gtk_vars.h"
#include "_main_windows.h"
#include "_bundle_adjust_tools.h"
#include <Eigen/Dense>
#include "_gain_compensation.h"
#include "_blending.h"
#include <opencv2/core/eigen.hpp>
#include "_bundle_adjust_main.h"

namespace fs = std::filesystem;
/*
//struct main_window_ main_window;
*/
std::vector<Eigen::MatrixXf> ret_vi(const std::vector<Eigen::MatrixXf> &bvec){

     std::vector<Eigen::MatrixXf> Vi;
     for(int i = 0;i < bvec.size();i++){

         Vi.push_back(bvec[i].transpose()*bvec[i]);

    }

     return Vi;
}

Eigen::MatrixXd ret_u(const std::vector<Eigen::MatrixXf> &avecf){

    std::vector<Eigen::MatrixXd> avec;

    for (int i = 0;i<avecf.size();i++){
      Eigen::MatrixXd cast = avecf[i].cast<double>();
      avec.push_back(cast) ;

    }


    Eigen::MatrixXd U;
    int c = avec[0].cols();
    U = Eigen::MatrixXd::Zero(c,c);

     for(int i = 0;i < avec.size();i++){

         U = U + avec[i].transpose()*avec[i];

    }

     return U;
}

Eigen::MatrixXf ret_uf(const std::vector<Eigen::MatrixXf> &avec){



    Eigen::MatrixXf U;
    int c = avec[0].cols();
    U = Eigen::MatrixXf::Zero(c,c);

     for(int i = 0;i < avec.size();i++){

         U = U + avec[i].transpose()*avec[i];

    }

     return U;
}

std::vector<Eigen::MatrixXf> ret_Wi(const std::vector<Eigen::MatrixXf> &bvec,const std::vector<Eigen::MatrixXf> &avec){

    std::vector<Eigen::MatrixXf> W;

    for(int i = 0;i < avec.size();i++){

         W.push_back(avec[i].transpose() * bvec[i]);

    }

    return W;
}

std::vector<Eigen::VectorXf> ret_Eb(const std::vector<Eigen::MatrixXf> &bvec,const std::vector<Eigen::VectorXf> &e_vec){

    std::vector<Eigen::VectorXf> Ev;

    for(int i = 0;i < bvec.size();i++){

         Ev.push_back(bvec[i].transpose() * e_vec[i]);

    }

    return Ev;
}

Eigen::VectorXf ret_Ea(const std::vector<Eigen::MatrixXf> &avec,const std::vector<Eigen::VectorXf> &e_vec){

    Eigen::VectorXf Ev = Eigen::VectorXf::Zero(avec[0].cols());

    for(int i = 0;i < avec.size();i++){

        Ev = Ev + avec[i].transpose() * e_vec[i];

    }

    return Ev;
}

std::vector<Eigen::MatrixXf> ret_Y(const std::vector<Eigen::MatrixXf> &W,std::vector<Eigen::MatrixXf> V,float labda){

    std::vector<Eigen::MatrixXf> Y;
    for (int p = 0;p < V.size();p++){

        V[p].diagonal() = V[p].diagonal()*(1 + labda);
        Y.push_back(W[p]*V[p].inverse());

    }

    return Y;
}

Eigen::VectorXf ret_delta_a(Eigen::MatrixXf &uvecf,const std::vector<Eigen::MatrixXf> &Yvec,const std::vector<Eigen::MatrixXf> &wvec,Eigen::VectorXf &eA_vec,const std::vector<Eigen::VectorXf> &eB_vec,float labda){
Eigen::VectorXf rr;

    uvecf.diagonal() = uvecf.diagonal()*(1 + labda);


    Eigen::MatrixXf sum_wy = Eigen::MatrixXf::Zero(Yvec[0].rows(),wvec[0].rows());
    Eigen::VectorXf sum_YEb = Eigen::VectorXf::Zero(Yvec[0].rows());

    for(int i = 0;i<wvec.size();i++){

        sum_wy = sum_wy + Yvec[i]*wvec[i].transpose();
        sum_YEb = sum_YEb + Yvec[i] * eB_vec[i];

    }


    uvecf = uvecf - sum_wy;
    //std::cout <<"sum_wy: "<<"\n"<<uvecf<<"\n";
    Eigen::VectorXf x = uvecf.colPivHouseholderQr().solve(eA_vec - sum_YEb);


    return x;
}


std::vector<Eigen::VectorXf> ret_delta_b(std::vector<Eigen::MatrixXf> vvec,const std::vector<Eigen::VectorXf> &eB_vec,const std::vector<Eigen::MatrixXf> &wvec,const Eigen::VectorXf &delta_a,float labda){

    std::vector<Eigen::VectorXf> delta_b;

    for (int p = 0;p < vvec.size();p++){

        vvec[p].diagonal() = vvec[p].diagonal()*(1 + labda);
        delta_b.push_back( vvec[p].inverse() * (eB_vec[p] - wvec[p].transpose() * delta_a) );

    }
    return delta_b;
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
            //test.images_to_cylinder(1100);
            test.calculate_keypoints();


            std::vector<maths::keypoints> key_p = test.get_keypoints();

            //std::cout <<key_p.size();

            std::vector<cv::Mat> imags = test.get_images();

            //cv::Mat tetetst = imgm::project(imags[0],400,150);




            std::cout << "preprocessing done"<<"\n";
            maths::graph_thread::set_mat(imags,key_p);
            maths::thread threads = maths::graph_thread::get_threads(3);

            maths::graph_thread t1;
            maths::graph_thread t2;
            maths::graph_thread t3;

            std::thread thread_obj1(&maths::graph_thread::cal_adj, &t1, imags,threads[0]);
            std::thread thread_obj2(&maths::graph_thread::cal_adj, &t2, imags,threads[1]);
            std::thread thread_obj3(&maths::graph_thread::cal_adj, &t3, imags,threads[2]);

            thread_obj1.join();
            thread_obj2.join();
            thread_obj3.join();

            cv::Mat adj = t1.return_adj_mat();
            std::vector<std::vector< cv::Matx33f >> hom_mat = t1.return_Hom_mat();
            std::vector<std::vector<std::vector<cv::DMatch>>> matchMat = t1.return_match_mat();
            float focal = 900;
            imgm::pan_img_transform Tr(adj);
            Tr = imgm::calc_stitch_from_adj(imags,testpars.hom,adj);

            float labda = .001;

            class bund::parameters sanity(Tr.H_1j,key_p,matchMat,adj,900);
            std::vector<std::vector< cv::Matx33f >> hom_b = sanity.ret_hmat();





            std::vector<Eigen::VectorXf> ms = sanity.ret_measurements();
            std::vector<Eigen::MatrixXf> bvec = sanity.ret_B_i();
            std::vector<Eigen::MatrixXf> vvec = ret_vi(bvec);
            class bund::E_func err(key_p,matchMat,adj);
            std::vector<Eigen::VectorXf> meas = err.get_measurements();
            std::vector<Eigen::VectorXf> e_vec = err.error(ms);
            std::vector<Eigen::VectorXf> eB_vec = ret_Eb(bvec,e_vec);
            std::vector<Eigen::MatrixXf> avec = sanity.ret_A_i();
            Eigen::MatrixXf uvecf = ret_uf(avec);
            Eigen::VectorXf eA_vec = ret_Ea(avec,e_vec);
            std::vector<Eigen::MatrixXf> wvec = ret_Wi(bvec,avec);
            std::vector<Eigen::MatrixXf> Yvec = ret_Y(wvec,vvec,labda);



            class bundm::adjuster testad(Tr.H_1j,key_p,matchMat,adj,800,labda);
            struct bundm::inter_par testpars = testad.iterate();



            //blnd::simple_blend(Tr,imags);


            //Tr = imgm::calc_stitch_from_adj(imags,hom_mat ,adj);


            /*
            class bundm::adjuster testad(Tr.H_1j,key_p,matchMat,adj,800,hom_mat,labda);
            struct bundm::inter_par testpars = testad.iterate();

            std::cout <<"test eA_vec: "<<"\n"<<testpars.delta_a<<"\n";
            std::cout <<"test delta_b: "<<"\n"<<testpars.delta_b[120]<<"\n";



            std::vector<std::vector<int>> idx_set = err.ret_idx_set();


            //Eigen::MatrixXd uvec = ret_u(avec);





            std::cout <<"wvec: "<<"\n"<<vvec[1]({0,1,2},{0,1,2})<<"\n";
            std::cout <<"wvec: "<<"\n"<<vvec[1]({0,1,2},{0,1,2}).determinant()<<"\n";
            std::cout <<"bvec: "<<"\n"<<bvec[1]<<"\n";





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




            Tr = imgm::calc_stitch_from_adj(imags,hom_mat ,adj);
            Eigen::Matrix3f matd;
            cv::cv2eigen(Tr.H_1j[0], matd);

            class bund::parameters sanity(Tr.H_1j,key_p,matchMat,adj,800,hom_mat);

            class bund::E_func err(key_p,matchMat,adj);

            std::vector<Eigen::MatrixXf> bvec = sanity.ret_B_i();
            std::vector<Eigen::MatrixXf> avec = sanity.ret_A_i();
            std::vector<Eigen::VectorXf> ms = sanity.ret_measurements();

            std::vector<Eigen::VectorXf> e_vec = err.error(ms);

            Eigen::MatrixXf uvec = ret_u(avec);
            std::vector<Eigen::MatrixXf> vvec = ret_vi(bvec);
            std::vector<Eigen::MatrixXf> wvec = ret_Wi(bvec,avec);



            std::cout <<"w: "<<"\n"<<vvec[11]<<"\n";

/*






            imgm::pan_img_transform Tr(adj);
            Tr = imgm::calc_stitch_from_adj(imags,hom_mat ,adj);
            blnd::simple_blend(Tr,imags);


            int height = Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].yend - Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].ystart + 1;
            int wide = Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].xend - Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].xstart + 1;

            cv::Mat panorama = cv::Mat::zeros(height,wide,imags[0].type());


            for (int i = 0;i <Tr.stitch_order.size();i++){
                cv::Mat tmp = cv::Mat::zeros(height,wide,imags[0].type());
                cv::warpPerspective(imags[Tr.stitch_order[i]], tmp, Tr.img2pan[Tr.stitch_order[i]], panorama.size());

                panorama.copyTo(panorama, tmp);

            }

            cv::imshow("Image Display",panorama);
            cv::waitKey(0);



           // blnd::simple_blend(tes,imags);



            cv::Mat panorama = imags[tes.stitch_order[0]];

            std::vector<cv::Vec3f> pt;
            pt.push_back({400,300,1});

            for (int i = 1; i < tes.stitch_order.size();i++){
                std::cout<<"stohigh: "<< tes.translation[tes.stitch_order[i]].yend - tes.translation[tes.stitch_order[i]].ystart + 1<<"\n";
                //std::cout<<"stowide: "<< yend - ystart + 1<<"\n";
                panorama = imgm::stitch(panorama, imags[tes.stitch_order[i]], tes.img2pan[tes.stitch_order[i]],pt);

            }



            cv::imshow("Image Display", panorama);
            cv::waitKey(0);



*/




            return 0;
/*






            pan::panorama P_test;
            P_test.init(path_list,ORDER_AS_IS);

            cv::Mat asd = imgm::file_to_cv(path_list[0]);




        build_window(argc,argv,&main_window);
        gtk_main();
*/
}
