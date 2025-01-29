
#ifndef PANO_H
#define PANO_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "_maths.h"
#include "_img_manipulation.h"
#include "_image.h"

#define ORDER_AS_IS 1

namespace pan{


class panorama : public img::images {

    public:

        unsigned int order_mode = ORDER_AS_IS;

        cv::Mat panorama_image;

    private:

        std::vector<int> image_order;
        std::vector<maths::keypoints> img_keypoints;

        void extract_keypoints(std::vector<cv::Mat> img_data);
};

}

#endif

