

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

            ~images() {

                std::vector<cv::Mat>().swap(img_data);
            }

            void add_images(std::vector<std::string>& files);

            template <typename T>
            auto add_images(const T& files)-> std::enable_if_t<std::is_convertible_v<T, std::string>, void>{

                std::vector<std::string> temp;
                temp.push_back(files);  // Convert single string to vector
                add_images(temp);
            }

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

            void load_resized(int max_size);

            void clear_images();

            std::vector<cv::Mat>* get_image_addresses();

            std::vector<std::string> get_f_list();

            void calculate_keypoints(int threads = 1,std::atomic<double> * frct = NULL,std::atomic<bool> * cancel = NULL);

            std::vector<util::keypoints> get_keypoints();

            void images_to_cylinder(float f);

            void images_to_cylinder(std::vector<double> f , std::vector<cv::Vec2f> im_center);


        protected:

            std::vector<util::keypoints> keypnts;
            std::vector<cv::Mat> img_data;
            std::vector<std::string> f_list;
            std::vector<std::string> loaded;

    };

}

#endif
