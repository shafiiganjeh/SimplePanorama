
#include <gtk/gtk.h>
#include "_gtk_vars.h"
#include "_main_windows.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "_panorama.h"
#include <filesystem>

int main(int argc, char **argv) {

        struct main_window_ main_window;

        build_window(argc,argv,&main_window);
        gtk_main();
/*
for(int i = 0;i<10;i++){
            struct pan::config conf;
            conf.focal = 700;

            std::string path = "/home/sd_bert/projects/Panorama/test";
            std::vector<std::string> path_list;

            for (const auto & entry : std::filesystem::directory_iterator(path)){
                std::string temp_string = entry.path().string();

                path_list.push_back(temp_string);

            }

            pan::panorama test(path_list);
            class util::Timer timer;

            test.stitch_panorama(conf);}
 */
}


