
#ifndef IMGMAN_H
#define IMGMAN_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

namespace imgm {

    class images {

        public:

            images(std::vector<std::string> files);

            void load_images(std::vector<std::string> f_list);

            void clear_images();

        private:

            std::vector<std::string> f_list;
            std::vector<cv::Mat> img_data;

    };


    cv::Mat resize_image( const cv::Mat& img, int target_width = 300);

    cv::Mat file_to_cv(std::string path);



}
#endif

