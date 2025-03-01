
#include <gtk/gtk.h>
#include <opencv2/opencv.hpp>
#include "_panorama.h"
#include "_maths.h"
#include "_img_manipulation.h"
#include "_image.h"
#include "_gtk_vars.h"
#include "_main_windows.h"
//#include "_bundle_adjust_tools.h"
#include <Eigen/Dense>
//#include "_gain_compensation.h"
//#include "_blending.h"
#include <opencv2/core/eigen.hpp>
//#include "_bundle_adjust_main.h"

namespace fs = std::filesystem;
/*
//struct main_window_ main_window;

*/
struct main_window_ main_window;
struct approx{

    std::vector<std::vector<Eigen::Matrix3f>> K;
    std::vector<std::vector<Eigen::Matrix3f>> R;
    std::vector<std::vector<Eigen::MatrixXf>> R_v;

};



int main(int argc, char **argv) {


        build_window(argc,argv,&main_window);
        gtk_main();




/*
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





        build_window(argc,argv,&main_window);
        gtk_main();
*/
}
