

#ifndef IMG_H
#define IMG_H

#include <opencv2/opencv.hpp>
#include "_img_manipulation.h"
#include "_gain_compensation.h"
#include "_util.h"
#include "_homography.h"
#include <iostream>
#include <fstream>
#include "_homography.h"

namespace img {


    class images {

        public:

            images(std::vector<std::string> files);

            std::vector<cv::Mat> load_connected_images(const std::vector<bool> &load);

            template <typename T>
            auto load_connected_images(std::vector<T>& load)
                -> std::enable_if_t<std::is_arithmetic_v<T> && !std::is_same_v<T, bool>, std::vector<cv::Mat>>{

                std::vector<bool> temp;
                temp.reserve(load.size());
                for (const T& x : load) {
                    temp.push_back(x > T(0));
                }

                return load_connected_images(temp);
            }


            void clear_images();

            std::vector<cv::Mat> get_images();

            std::vector<std::string> get_f_list();

            void calculate_keypoints(int threads = 3);

            std::vector<util::keypoints> get_keypoints();

            void images_to_cylinder(float f);

            void images_to_cylinder(std::vector<double> f , std::vector<cv::Vec2f> im_center);


        protected:

            std::vector<util::keypoints> keypnts;
            std::vector<cv::Mat> img_data;
            std::vector<std::string> f_list;

    };

}

#endif
