
#ifndef PANO_H
#define PANO_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "_maths.h"
#include "_img_manipulation.h"
#include "_image.h"
#include "_blending.h"
#include "_bundle_adjust_main.h"
#include "_stitch.h"

#include <cmath>      /* sin */

#define PI 3.14159265

#define ORDER_AS_IS 1
using thread = std::vector<std::vector<std::vector<int>>>;

namespace pan{

    enum Blending {
        NO_BLEND,
        SIMPLE_BLEND,
    };

    struct config{

        bool gain_compensation = false;
        bool bundle_adjust = false;
        int blend = SIMPLE_BLEND;
        float focal = 1500;
        int init_size = 800;
        float lambda = .0001;

    };

class panorama : public img::images {

    public:

        panorama(std::vector<std::string> files) : img::images(files) {}
        void load_resized(int width);
        void get_adj_par(int threads = 3);
        cv::Mat get_adj();
        std::vector<std::vector<std::vector<cv::DMatch>>> get_match_mat();
        std::vector<std::vector< cv::Matx33f >> get_hom_mat();

        void create_panorama(int threads,struct config conf);

    private:

        std::vector<int> image_order;
        cv::Mat panorama_image;

        cv::Mat adj;
        std::vector<std::vector< cv::Matx33f >> hom_mat;
        std::vector<std::vector<std::vector<cv::DMatch>>> match_mat;

};

}

#endif

