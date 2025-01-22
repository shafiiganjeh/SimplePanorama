
#ifndef PANO_H
#define PANO_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "_maths.h"

#define ORDER_AS_IS 1

namespace pan{


class panorama {

    public:
        void init(std::vector<std::string> files,unsigned int order_mode);

        unsigned int order_mode;

        cv::Mat panorama_image;

    private:

        std::vector<int> image_order;
        std::vector<std::string> f_list;
        std::vector<maths::keypoints> img_keypoints;
        std::vector<cv::Mat> img_data;


        std::vector<cv::Mat> load_images(std::vector<std::string> f_list);

        std::vector<maths::keypoints> extract_keypoints(std::vector<cv::Mat> img_data);

};

}

#endif

