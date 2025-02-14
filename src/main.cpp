
#include <gtk/gtk.h>
#include <opencv2/opencv.hpp>
#include "_panorama.h"
#include "_maths.h"
#include "_img_manipulation.h"
#include "_image.h"
#include "_gtk_vars.h"
#include "_main_windows.h"
#include "_bundle_adjust.h"
#include <Eigen/Dense>
#include "_gain_compensation.h"
#include "_blending.h"

namespace fs = std::filesystem;
/*
//struct main_window_ main_window;
*/

void write_to_eigen(Eigen::VectorXf &v, cv::Vec2f &cv_v,int n,int st){

    for(int i = 0;i < n;i++){

        v[st+i] = cv_v[i];

    }

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
            //test.images_to_cylinder(950.0);
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

            std::vector<float> G;
            G =  gain::gain_compensation(imags,adj,hom_mat);

            std::cout <<adj<<"\n";
            //float foc = maths::focal_from_hom(hom_mat,adj);
            //std::cout<<"focal is: "<< foc <<"\n";

            cv::Vec3f q = maths::eucl2hom_point_2D(key_p[0].keypoint[matchMat[0][1][0].queryIdx].pt);
            cv::Vec3f t = maths::eucl2hom_point_2D(key_p[1].keypoint[matchMat[0][1][0].trainIdx].pt);
            std::vector<cv::Vec3f> tv(1);
            tv[0] = t;

            std::cout<<"kp q: " << q <<"\n";
            std::cout<<"kp t: " << t <<"\n";

            std::vector<cv::Vec3f> r =  maths::applyH_2D(tv, hom_mat[0][1], maths::GEOM_TYPE_POINT);
            std::cout<<"kp t: " << r[0] <<"\n";



            //std::cout<<"n matches: " << matchMat[0][1][0].queryIdx <<"\n";

            class bund::E_func cl_test(key_p,matchMat,adj);

            std::vector<Eigen::VectorXf> m_vec = cl_test.get_measurements();
            Eigen::MatrixXf v(3,3);
            int a = 3;

            v << a,2,3,4,5,6,7,8,a;

            std::cout<<"mvec: " << v <<"\n";



/*



            for (int i = 0;i<imags.size();i++){


               imags[i] = imags[i] * G[i];

            }


            cv::Mat path_lenght = cv::Mat::ones(adj.rows, adj.cols, CV_64F);
            path_lenght = path_lenght - path_lenght.mul(adj);

            std::cout<< path_lenght <<"\n";
            cv::Mat row_sum;
            cv::reduce(adj, row_sum, 1, cv::REDUCE_SUM, CV_64F);
            double min=0, max=0;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(row_sum, &min, &max, &minLoc, &maxLoc);
            std::cout<<"row sum"<< row_sum <<"\n";
            std::cout<<"max"<< maxLoc.y <<"\n";


            std::vector<std::pair<int, std::vector<int>>> tesst = maths::bfs_ordered_with_neighbors(path_lenght, maxLoc.y);


            for (int j = 0;j<tesst.size();j++){

                std::cout<< "node: "<< tesst[j].first <<" connect: ";
                for (int i: tesst[j].second){
                    std::cout << i << ' ';}
                std::cout<<"\n";
            }

            std::map<int, std::pair<int,double>> table = maths::path_table(path_lenght,tesst,maxLoc.y);

            for (auto const& [key, val] : table)
            {
                std::cout << key        // string (key)
                        << ':'
                        << val.first        // string's value
                        << std::endl;
            }

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


            std::vector<cv::Vec3f> poin;

            poin.push_back({169.532,159.708,1});
            poin = maths::applyH_2D(poin,  tes.pan2img[2], maths::GEOM_TYPE_POINT);

            //std::cout<<poin[0];
            //cv::Point2f a(poin[0][0]/poin[0][2], poin[1][0]/poin[0][2]);

            poin[0] = poin[0]/poin[0][2];
            std::cout << poin[0]<<"\n";
            cv::Point2f at(poin[0][0], poin[0][1]);
//imags[2]
//panorama
            cv::circle(imags[2],at,20,CV_RGB(50, 50,122),10);

            cv::imshow("Image Display", imags[2]);
            cv::waitKey(0);




/*

2715

            //cv::Mat pan = imgm::stitch(imags[0], imags[1], H);




            Eigen::MatrixXd b(2,3);
            b << 92, 60, 100,
                 80, 30, 70;

            Eigen::MatrixXd con_XY;
            con_XY = cov_mat_XY(b,b);

            std::cout << con_XY <<"\n";





            std::vector<std::vector<int>> angles = {
            {0, 1, 1, 0, 0, 1},
            {1, 0, 1, 1, 0, 0},
            {1, 1, 0, 1, 1, 0},
            {0, 1, 1, 0, 1, 1},
            {0, 0, 1, 1, 0, 1},
            {1, 0, 0, 1, 1, 0}
            };

            cv::Mat matAngles(angles.size(), angles.at(0).size(), CV_64F);
            for(int i=0; i<matAngles.rows; ++i)
                for(int j=0; j<matAngles.cols; ++j)
                    matAngles.at<double>(i, j) = angles.at(i).at(j);

             //std::cout << matAngles << " ";










            maths::graph_thread::set_mat(imags,key_p);
            maths::thread threads = maths::graph_thread::get_threads(4);

            maths::graph_thread t1;
            maths::graph_thread t2;
            maths::graph_thread t3;
            maths::graph_thread t4;

            std::thread thread_obj1(&maths::graph_thread::cal_adj, &t1, imags,threads[0]);
            std::thread thread_obj2(&maths::graph_thread::cal_adj, &t2, imags,threads[1]);
            std::thread thread_obj3(&maths::graph_thread::cal_adj, &t3, imags,threads[2]);
            std::thread thread_obj4(&maths::graph_thread::cal_adj, &t4, imags,threads[3]);

            thread_obj1.join();
            thread_obj2.join();
            thread_obj3.join();
            thread_obj4.join();

            std::cout<< t1.return_mat() <<"\n";

            cv::Mat vset = t1.return_mat();

            std::vector<int> connected_vertices = dfs(vset, 1);



            for (int i = 0; i < connected_vertices.size(); ++i) {
                std::cout<< connected_vertices[i] <<" ";
            }


            cv::Mat ttest= t1.return_mat();
           // pan_from_mat(ttest,imags,key_p);











            cv::Mat pan = imgm::stitch(imags[1], imags[0], H);

            kp1 = maths::extract_keypoints(imags[2]);
            kp2 = maths::extract_keypoints(pan);

            match = maths::match_keypoints(kp1,kp2);
            H = maths::find_homography(kp1,kp2,match,1000,4);
            pan = imgm::stitch(imags[2],pan,  H);

            cv::imshow("Image Display", pan);
            cv::waitKey(0);



 *                         kp1 = kpmat[i];
                        kp2 = kpmat[j];

                        simmat.at<double>(i,j) = match_quality(kp1,imags[i],kp2,imags[j]);
 *
            std::vector<cv::DMatch> match = maths::match_keypoints(kp1,kp2);

        std::cout << match.size()<<"\n";
        cv::Matx33f  H = maths::find_homography(kp1,kp2,match,1000,4);




        cv::Mat pan = imgm::stitch(imags[2], imags[0], H);

         cv::imshow("Image Display", pan);
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
