
#include <gtk/gtk.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <string>
#include "_panorama.h"
#include "_maths.h"
#include "_img_manipulation.h"
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

            pan::panorama P_test;
            P_test.init(path_list,ORDER_AS_IS);

            cv::Mat asd = imgm::file_to_cv(path_list[0]);



/*
        build_window(argc,argv,&main_window);
        gtk_main();
*/
}
