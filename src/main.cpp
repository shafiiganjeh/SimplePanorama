
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



namespace fs = std::filesystem;
/*
//struct main_window_ main_window;
*/




cv::Mat project(cv::Mat& imags,int &xc,int &yc){

    const int rows = imags.rows;
    const int cols = imags.cols;
    const int size = std::max(rows,cols);


    std::vector<float> v_123(size);
    std::iota(v_123.begin(), v_123.end(), 0);
    std::vector<float> v_111(size,1);

    Eigen::Matrix<float, 1, Eigen::Dynamic> a(Eigen::Map<Eigen::RowVectorXf> (v_123.data(),size));
    Eigen::Matrix<float, Eigen::Dynamic, 1> b(Eigen::Map<Eigen::RowVectorXf> (v_123.data(),size));

    Eigen::MatrixXf map_x(cols, rows);
    Eigen::MatrixXf map_y(rows, cols);

    map_x = a(Eigen::all,Eigen::seq(0,cols - 1)).replicate(1,rows);
    map_y = a(Eigen::all,Eigen::seq(0,rows - 1)).replicate(cols,1);

    //map_y = a.replicate(rows,1);

    int f = 500;

    for (int i = 0 ; i<rows*cols;i++){

        map_y(i) = (map_y(i)-yc)/cos((map_x(i)-xc)/f) + yc;
        map_x(i) = tan((map_x(i)-xc)/f) * f + xc;

    }

    cv::Mat dst(imags.size(), imags.type());
    cv::Mat vec_x(imags.size(), CV_32FC1,map_x.data());
    cv::Mat vec_y(imags.size(), CV_32FC1,map_y.data());

//std::cout << vec_x<<"\n";
//std::cout << vec_y<<"\n";

    cv::remap(imags, dst,vec_x, vec_y, cv::INTER_LINEAR);

    return dst;
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
            test.images_to_cylinder();
            test.calculate_keypoints();

            std::vector<maths::keypoints> key_p = test.get_keypoints();

            //std::cout <<key_p.size();

            std::vector<cv::Mat> imags = test.get_images();

            //cv::Mat tetetst = imgm::project(imags[0],400,150);

            //cv::imshow("Image Display", tetetst);
            //cv::waitKey(0);


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
            //std::cout<< t1.return_mat() <<"\n";

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


            std::vector<std::pair<int, std::vector<int>>> tesst = maths::bfs_ordered_with_neighbors(adj, maxLoc.y);


            for (int j = 0;j<tesst.size();j++){

                std::cout<< "node: "<< tesst[j].first <<" connect: ";
                for (int i: tesst[j].second){
                    std::cout << i << ' ';}
                std::cout<<"\n";
            }

            std::map<int, std::pair<int,double>> table = maths::path_table(adj,tesst,maxLoc.y);

            for (auto const& [key, val] : table)
            {
                std::cout << key        // string (key)
                        << ':'
                        << val.first        // string's value
                        << std::endl;
            }

            imgm::stitch_adj(imags,hom_mat ,adj);


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
