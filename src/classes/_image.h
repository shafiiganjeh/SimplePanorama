

#ifndef IMG_H
#define IMG_H

#include <opencv2/opencv.hpp>
#include "_img_manipulation.h"
#include <iostream>
#include <fstream>

namespace img {

    class images {

        public:

            images(std::vector<std::string> files);

            void load_images();

            void clear_images();

            std::vector<cv::Mat> get_images();

            std::vector<std::string> f_list;

        private:



        protected:

            std::vector<cv::Mat> img_data;

    };

}

#endif
