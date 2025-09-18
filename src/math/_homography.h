
#ifndef HOM_H
#define HOM_H

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <opencv2/core/eigen.hpp>
#include "_util.h"

namespace util {

    struct Homography find_homography(const struct keypoints &kp1,const struct keypoints &kp2,const std::vector<cv::DMatch> &match,int max_iter,int sample,const cv::Mat &img1,const cv::Mat &img2,double error_margin);

    struct match_conf{

        //matching
        int max_images_per_match = 5;
        int RANSAC_iterations = 1500;
        int max_keypoints = 100;
        int x_margin = 5;

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

    };

    using thread = std::vector<std::vector<std::vector<int>>>;



    struct keypoints{

        std::vector<cv::KeyPoint> keypoint;
        cv::Mat descriptor;

    };


    struct keypoints extract_keypoints_detGFTT_descSIFT(const cv::Mat &img,int maxCorners = 1000,double qualityLevel = 0.01,double minDistance = 3.0,int blockSize = 3,bool useHarrisDetector = false,double k = 0.04,int nFeatures = 1000,int nOctaveLayers = 4,double contrastThreshold = 0.04,double edgeThreshold = 7.0,double sigma = 1.4142135623);


    struct Homography{

        cv::Matx33f H;
        cv::Matx33f norm_b;
        cv::Matx33f norm_a;

        int area_kp = 0;
        int area_match = 0;
        double overlap_perc = 0;

    };


    enum GeometryType {
        GEOM_TYPE_POINT,
        GEOM_TYPE_LINE,
    };

    class adj_calculator{

    public:

        adj_calculator(const std::vector<cv::Mat> & imgs,const std::vector<keypoints> &key_p,struct  match_conf* conf = NULL,std::atomic<double>* fadress = NULL,std::atomic<bool>* cancel = NULL);
        std::vector<size_t> find_n_smallest_indices(const std::vector<double>& rank, int n);

        void cal_adj(const std::vector<cv::Mat> & imgs,int T);
        void get_threads(int n);
        void get_match_number_matrix(int T);
        void heuristic_match_filter(int n);
        cv::Mat adj;
        cv::Mat adj_test;
        std::vector<keypoints> kpmat;
        std::vector<std::vector< cv::Matx33f >> hom_mat;
        std::vector<std::vector<std::vector<cv::DMatch>>> match_mat;

    private:

        struct match_conf conf_local;
        std::atomic<double>* f_adress;
        std::atomic<bool>* c_adress;
        double add = 0;

        thread TR;
        std::vector<std::vector<std::vector<cv::DMatch>>> match_mat_raw;
        std::vector<cv::DMatch> clean_matches(const struct keypoints &kp1,const struct keypoints &kp2,std::vector<cv::DMatch> match,const  cv::Matx33f &H,double margin);
        float match_quality(const struct keypoints &kp1,const cv::Mat img1,const struct keypoints &kp2,const cv::Mat img2,int row,int col);

};

    cv::Matx33f decondition_homography2D(const cv::Matx33f &T_base, const cv::Matx33f &T_attach, const cv::Matx33f &H);
    float N_outliers(const struct keypoints &kp1,const struct keypoints &kp2,std::vector<cv::DMatch> &match,const  cv::Matx33f &H,std::vector<float> &T);
    double homography_loss(const struct keypoints &kp1,const struct keypoints &kp2,const std::vector<cv::DMatch> &match , const cv::Matx33f &H ,double error_margin);
    bool hom_sanity(cv::Matx33f hom,const cv::Mat &img1,const cv::Mat &img2);
    cv::Vec3f eucl2hom_point_2D(const cv::Vec2f& p);
    cv::Vec2f hom2eucl_point_2D(const cv::Vec3f& p);
    std::vector<cv::Vec3f> applyH_2D(const std::vector<cv::Vec3f>& geomObjects, const cv::Matx33f &H, GeometryType type);
    cv::Matx33f Normalize2D(const std::vector<cv::Vec3f> &points);
    cv::Mat_<float> getDesignMatrix_homography2D(const std::vector<cv::Vec3f> &conditioned_base, const std::vector<cv::Vec3f> &conditioned_attach);
    cv::Matx33f solve_homography2D(const cv::Mat_<float> &A);
    std::pair<std::vector<cv::DMatch>, std::vector<cv::DMatch>> match_keypoints(const struct keypoints &kp1,const struct keypoints &kp2);
    std::vector<keypoints> extrace_kp_vector(const std::vector<cv::Mat> & imgs,std::vector<int> idx,match_conf* conf = NULL);
    struct keypoints extract_keypoints(const cv::Mat &img,int nfeatures = 0,int nOctaveLayers = 4,double contrastThreshold = 3e-2,double edgeThreshold = 6,double sigma = 1.4142);


}

#endif
