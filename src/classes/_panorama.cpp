
#ifndef PAN_H
#define PAN_H

#include "_panorama.h"

namespace pan{
/*
    void panorama::init(std::vector<std::string> files,unsigned int mode){

        order_mode = mode;
        if (mode == ORDER_AS_IS){

            image_order.reserve(files.size());
            std::iota(image_order.begin(), image_order.end(), 1);

        }

        f_list = files;

    }
*/

    void panorama::extract_keypoints(std::vector<cv::Mat> img_data){

        cv::Mat greyMat;

        for (const auto & img_matrix : img_data){

            cv::cvtColor(img_matrix, greyMat, cv::COLOR_BGR2GRAY);


        }

    }

}
#endif
