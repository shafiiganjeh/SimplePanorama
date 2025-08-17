
#ifndef PANO_H
#define PANO_H

#include <opencv2/opencv.hpp>
#include <optional>
#include <vector>
#include <string>
#include "_util.h"
#include "_img_manipulation.h"
#include "_image.h"
#include "_blending.h"
#include "_bundle_adjust_main.h"
#include "_stitch.h"
#include "_homography.h"
#include <opencv2/core/eigen.hpp>
#include <limits>
#include "_graph_cut.h"

#include <cmath>      /* sin */

#define PI 3.14159265

#define ORDER_AS_IS 1
using thread = std::vector<std::vector<std::vector<int>>>;

namespace pan{

    enum Blending {
        NO_BLEND,
        SIMPLE_BLEND,
        MULTI_BLEND,
    };

    struct config{

        std::vector<bool> use;
        bool gain_compensation = true;
        bool cut = true;
        int blend = MULTI_BLEND;
        float focal = 1500;
        int init_size = 800;
        float lambda = .0001;
        int max_images_per_match = 5;

    };


    struct blend_data{

        std::vector<cv::Mat> imgs;
        std::vector<cv::Point> corners;
        std::vector<cv::Mat> msks;
        std::vector<cv::Mat> msks_cut;
        std::vector<int> ord;

    };


class stitch_parameters{

    public:

        stitch_parameters(struct stch::stitch_result&& res,class panorama* pan): owned_res(std::move(res)),pan_address(pan){}

        void set_config(struct config& conf);

        cv::Mat get_preview(struct config& conf);

        cv::Mat  return_full(struct config& conf);


    private:

        cv::Mat blend(struct blend_data& temp,struct config& conf);

        std::vector<double> gain;
        std::vector<cv::Mat> mask_cut;
        struct stch::stitch_result owned_res;
        class panorama* pan_address;

};


class panorama : public img::images {

    public:

        panorama(std::vector<std::string> files) : img::images(files) {}
        void load_resized(int max_size);
        void get_adj_par(int threads = 3);
        cv::Mat get_adj();
        std::vector<std::vector<std::vector<cv::DMatch>>> get_match_mat();
        std::vector<std::vector< cv::Matx33f >> get_hom_mat();

        void stitch_panorama(int threads,struct config& conf);

    private:

        std::optional<class stitch_parameters> stitched;
        struct config conf_local;
        std::vector<int> image_order;
        cv::Mat preview;

        cv::Mat adj;
        std::vector<std::vector< cv::Matx33f >> hom_mat;
        std::vector<std::vector<std::vector<cv::DMatch>>> match_mat;

};

}

#endif

