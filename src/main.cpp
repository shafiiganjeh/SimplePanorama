
#include <gtk/gtk.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <string>
#include "_panorama.h"
#include "_maths.h"
#include "_img_manipulation.h"
#include "_image.h"
#include "_gtk_vars.h"
#include "_main_windows.h"

namespace fs = std::filesystem;

//struct main_window_ main_window;

int main(int argc, char **argv) {

            std::string path = "/home/sd_bert/projects/Panorama/test";
            std::vector<std::string> path_list;

            for (const auto & entry : fs::directory_iterator(path)){
                std::string temp_string = entry.path().string();

                path_list.push_back(temp_string);

            }

            img::images test(path_list);
            test.load_images();

            std::vector<cv::Mat> imags = test.get_images();

            maths::keypoints kp1;
            maths::keypoints kp2;

            std::vector<maths::keypoints> kpmat;

            for (int c = 0;c < imags.size();c++){

                kpmat.push_back(maths::extract_keypoints(imags[c]));

            }


            cv::Mat simmat = cv::Mat::zeros(imags.size(), imags.size(), CV_64FC1);

            for (int i = 0;i < imags.size();i++){

                for (int j = i;j < imags.size();j++){

                    if (i == j){

                        simmat.at<double>(i,j) = 1;

                    }else{

                        kp1 = kpmat[i];
                        kp2 = kpmat[j];

                        simmat.at<double>(i,j) = match_quality(kp1,imags[i],kp2,imags[j]);

                    }

                }
            }



            std::cout << simmat<<"\n";

/*
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
