

#ifndef IMG_H
#define IMG_H

#include <opencv2/opencv.hpp>
#include "_img_manipulation.h"
#include "_gain_compensation.h"
#include "_maths.h"
#include <iostream>
#include <fstream>

namespace img {


    class images {

        public:

            images(std::vector<std::string> files);

            void load_images();

            void clear_images();

            std::vector<cv::Mat> get_images();

            std::vector<std::string> get_f_list();

            void calculate_keypoints(int threads = 3);

            std::vector<maths::keypoints> get_keypoints();

            void images_to_cylinder(float f);

            void gain_compensation(std::vector<cv::Mat> &imags,const cv::Mat& adj,std::vector<std::vector< cv::Matx33f >>& Hom_mat);


        protected:

            std::vector<maths::keypoints> keypnts;
            std::vector<cv::Mat> img_data;
            std::vector<std::string> f_list;

    };

}

#endif
