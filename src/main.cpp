
#include <gtk/gtk.h>
#include <glib-object.h>
#include "_gtk_vars.h"
#include "_main_windows.h"
#include <filesystem>
#include "_config_parser.h"
//#include "_panorama.h"

std::filesystem::path get_executable_path();


int main(int argc, char **argv) {

        struct main_window_ main_window;
        pan::config current_conf;
        conf::ConfigParser test(&current_conf);

        std::filesystem::path exe_path = get_executable_path();
        std::filesystem::file_status s;

        main_window._path_ex= exe_path.parent_path();
        main_window._path_css= exe_path.parent_path();
        main_window._path_conf= exe_path.parent_path();

        main_window._path_conf /= "config";
        main_window._path_css /= "res";main_window._path_css /= "gtk.css";


        if (std::filesystem::status_known(s) ? std::filesystem::exists(s) : std::filesystem::exists(main_window._path_conf)){
            std::cout <<"Reading config file.\n";
            test.read_cfg(main_window._path_conf);
            test.write_cfg(main_window._path_conf);
        }else{
            std::cout <<"Creating config file.\n";
            test.write_cfg(main_window._path_conf);
            test.read_cfg(main_window._path_conf);
        }

        build_window(argc,argv,&main_window,&current_conf);
        gtk_main();

/*
            struct pan::config conf;
            conf.focal = 700;

            std::string path = "./test";
            std::vector<std::string> path_list;

            for (const auto & entry : std::filesystem::directory_iterator(path)){
                std::string temp_string = entry.path().string();

                path_list.push_back(temp_string);

            }

            pan::panorama test(path_list);
            test.stitch_panorama(&conf);
            cv::Mat tes = test.get_preview();
            //test.test(&conf);
            cv::imshow("Display window", tes);
            cv::waitKey(0);
*/
}

#ifdef __linux__

#include <unistd.h>

std::filesystem::path get_executable_path() {
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    if (count != -1) {
        return std::filesystem::path(std::string(result, count));
    }
    return {};
}

#elif defined(_WIN32)

#include <windows.h>

std::filesystem::path get_executable_path() {
    wchar_t buffer[MAX_PATH];
    GetModuleFileNameW(nullptr, buffer, MAX_PATH);
    return std::filesystem::path(buffer);
}

#endif


