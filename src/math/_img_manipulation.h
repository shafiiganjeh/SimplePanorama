
#ifndef IMGMAN_H
#define IMGMAN_H

#include <opencv2/opencv.hpp>

namespace imgm {

cv::Mat resize_image( const cv::Mat& img, int target_width = 300);

}
#endif

