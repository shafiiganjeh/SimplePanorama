

#ifndef MATHS_H
#define MATHS_H

#include <opencv2/opencv.hpp>
#include <numeric>

namespace maths {

    struct keypoints{
        std::vector<cv::Mat> kp;
    };

}

#endif
