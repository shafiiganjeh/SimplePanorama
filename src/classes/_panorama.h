
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
#include "_gtk_vars.h"
#include "_distance_cut.h"
#include <cmath>      /* sin */

#define PI 3.14159265

#define ORDER_AS_IS 1
using thread = std::vector<std::vector<std::vector<int>>>;

namespace pan{

    #define BLENDING_ENUM \
        X(NO_BLEND, 0)     \
        X(SIMPLE_BLEND, 1) \
        X(MULTI_BLEND, 2)  \
        X(_enum_sizeoff_, 3)

    enum Blending {
        #define X(name, value) name = value,
        BLENDING_ENUM
        #undef X
    };


    const char* BlendingToString(int value);
    int StringToBlending(const std::string& str);

    struct config{
        //system
        int threads = 8;
        int init_size = 800; //set calc size
        //blending
        Blending blend = MULTI_BLEND;
        bool gain_compensation = true;
        bool cut = false;
        bool cut_seams = true;
        //MULTI_BLEND
        int bands = 3;
        double sigma_blend = 7;
        //adjustment
        float focal = 700; //initial focal if estimation fails
        float lambda = .05; //initial lambda
        //matching
        int max_images_per_match = 5;
        int max_keypoints = 250;
        int RANSAC_iterations = 1500;
        int x_margin = 4;

        float min_overlap = .15;
        float overlap_inl_match = .1;
        float overlap_inl_keyp = .005;
        float conf = .025;

        //SIFT
        int nfeatures = 0;
        int nOctaveLayers = 4;
        double contrastThreshold = 3e-2;
        double edgeThreshold = 6;
        double sigma_sift = 1.4142;

        //non settings
        std::vector<bool> use;

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

        stitch_parameters(struct stch::stitch_result&& res,class panorama* pan,struct progress_bar_* prog = NULL): owned_res(std::move(res)),pan_address(pan),progress(prog){}

        void set_config(struct config& conf,std::atomic<bool>* cancel_var = NULL);

        cv::Mat get_preview(struct config& conf);
        cv::Size get_preview_size(struct config& conf);

        cv::Mat  return_full(struct config& conf);

    private:

        cv::Mat blend(struct blend_data& temp,struct config& conf);
        struct progress_bar_* progress = NULL;
        std::vector<double> gain;
        std::vector<cv::Mat> mask_cut;
        struct stch::stitch_result owned_res;
        class panorama* pan_address;

};


class panorama : public img::images {

    public:

        panorama(std::vector<std::string> files,struct progress_bar_* prog = NULL) : img::images(files),progress(prog) {}

        void get_adj_par(int threads = 1);
        cv::Mat get_adj();

        std::vector<std::vector<std::vector<cv::DMatch>>> get_match_mat();
        std::vector<std::vector< cv::Matx33f >> get_hom_mat();

        bool stitch_panorama(struct config* conf);

        cv::Mat get_preview();
        cv::Mat get_panorama(cv::Rect ROI = cv::Rect());

        void cancel();
        struct config conf_local;
        util::match_conf conf_m;

        void test(struct config* conf);

    private:

        cv::Mat panorama_full;
        bool pan_exist = false;

        std::optional<class stitch_parameters> stitched;
        std::vector<int> image_order;

        cv::Mat adj;
        std::vector<std::vector< cv::Matx33f >> hom_mat;
        std::vector<std::vector<std::vector<cv::DMatch>>> match_mat;

        struct progress_bar_* progress = NULL;
        std::atomic<bool> cancel_var = false;

};

}

#endif

