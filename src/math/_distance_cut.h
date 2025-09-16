
#ifndef DIST_H
#define DIST_H

#include <opencv2/opencv.hpp>
#include <vector>


namespace dcut {

std::vector<cv::Mat> dist_cut(const std::vector<cv::Mat>& masks,const std::vector<cv::Point>& top_lefts);

std::vector<cv::Mat> distance_transform(const std::vector<cv::Mat>& masks);


}

#endif

