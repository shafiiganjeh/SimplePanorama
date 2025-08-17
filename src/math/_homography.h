
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

    using thread = std::vector<std::vector<std::vector<int>>>;

    struct keypoints{

        std::vector<cv::KeyPoint> keypoint;
        cv::Mat descriptor;

    };

    struct Homography{

        cv::Matx33f H;
        cv::Matx33f norm_b;
        cv::Matx33f norm_a;

    };

    enum GeometryType {
        GEOM_TYPE_POINT,
        GEOM_TYPE_LINE,
    };

    class adj_calculator{

    public:

        adj_calculator(const std::vector<cv::Mat> & imgs,const std::vector<keypoints> &key_p);
        std::vector<size_t> find_n_smallest_indices(const std::vector<double>& rank, int n);

        void cal_adj(const std::vector<cv::Mat> & imgs,int T);
        void get_match_number_matrix(int T);
        void heuristic_match_filter(int n);

        float match_quality(const struct keypoints &kp1,const cv::Mat img1,const struct keypoints &kp2,const cv::Mat img2,int row,int col);

        void get_threads(int n);

        std::vector<cv::DMatch> clean_matches(const struct keypoints &kp1,const struct keypoints &kp2,std::vector<cv::DMatch> match,const  cv::Matx33f &H,const std::vector<float> &T12);

        cv::Mat adj;
        cv::Mat adj_test;
        std::vector<keypoints> kpmat;
        std::vector<std::vector< cv::Matx33f >> hom_mat;

        std::vector<std::vector<std::vector<cv::DMatch>>> match_mat_raw;
        std::vector<std::vector<std::vector<cv::DMatch>>> match_mat;

        thread TR;

};

    cv::Matx33f decondition_homography2D(const cv::Matx33f &T_base, const cv::Matx33f &T_attach, const cv::Matx33f &H);
    float N_outliers(const struct keypoints &kp1,const struct keypoints &kp2,std::vector<cv::DMatch> &match,const  cv::Matx33f &H,std::vector<float> &T);
    double homography_loss(const struct keypoints &kp1,const struct keypoints &kp2,const std::vector<cv::DMatch> &match , const cv::Matx33f &H );
    bool hom_sanity(cv::Matx33f hom,const cv::Mat &img1,const cv::Mat &img2);
    cv::Vec3f eucl2hom_point_2D(const cv::Vec2f& p);
    cv::Vec2f hom2eucl_point_2D(const cv::Vec3f& p);
    std::vector<cv::Vec3f> applyH_2D(const std::vector<cv::Vec3f>& geomObjects, const cv::Matx33f &H, GeometryType type);
    cv::Matx33f Normalize2D(const std::vector<cv::Vec3f> &points);
    cv::Mat_<float> getDesignMatrix_homography2D(const std::vector<cv::Vec3f> &conditioned_base, const std::vector<cv::Vec3f> &conditioned_attach);
    cv::Matx33f solve_homography2D(const cv::Mat_<float> &A);
    std::pair<double, double> computeOverlapPercentages(const cv::Mat& imgA,const cv::Mat& imgB,Eigen::MatrixXd& H);
    std::pair<std::vector<cv::DMatch>, std::vector<cv::DMatch>> match_keypoints(const struct keypoints &kp1,const struct keypoints &kp2);
    std::vector<keypoints> extrace_kp_vector(const std::vector<cv::Mat> & imgs,std::vector<int> idx);
    struct keypoints extract_keypoints(const cv::Mat &img,int nfeatures = 0,int nOctaveLayers = 4,double contrastThreshold = 3e-2,double edgeThreshold = 6,double sigma = 1.4142);


}

#endif
