
#include <gtk/gtk.h>
#include <opencv2/opencv.hpp>
#include "_panorama.h"
#include "_util.h"
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

struct main_window_ main_window;


int main(int argc, char **argv) {


        //build_window(argc,argv,&main_window);
        //gtk_main();

            pan::config conf;
            conf.focal = 700;



            std::string path = "/home/sd_bert/projects/Panorama/test";
            std::vector<std::string> path_list;

            for (const auto & entry : std::filesystem::directory_iterator(path)){
                std::string temp_string = entry.path().string();

                path_list.push_back(temp_string);

            }

            pan::panorama test(path_list);


            test.stitch_panorama(8,conf);

}
