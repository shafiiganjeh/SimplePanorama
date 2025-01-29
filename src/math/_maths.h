

#ifndef MATHS_H
#define MATHS_H

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <numeric>
#include <algorithm>
#include <random>


namespace maths {

    struct keypoints{
        std::vector<cv::KeyPoint> keypoint;
        cv::Mat descriptor;
    };

    enum GeometryType {
    GEOM_TYPE_POINT,
    GEOM_TYPE_LINE,
    };


    struct keypoints extract_keypoints(const cv::Mat &img,int nfeatures = 0,int nOctaveLayers = 3,double contrastThreshold = 0.04,double edgeThreshold = 10,double sigma = 1.6);

    std::vector<cv::DMatch> match_keypoints(const struct keypoints &kp1,const struct keypoints &kp2);

    cv::Matx33f Normalize2D(const std::vector<cv::Vec3f> &points);

    std::vector<cv::Vec3f> applyH_2D(const std::vector<cv::Vec3f>& geomObjects, const cv::Matx33f &H, GeometryType type);

    cv::Vec3f eucl2hom_point_2D(const cv::Vec2f& p);

    cv::Mat_<float> getDesignMatrix_homography2D(const std::vector<cv::Vec3f> &conditioned_base, const std::vector<cv::Vec3f> &conditioned_attach);

    cv::Matx33f solve_homography2D(const cv::Mat_<float> &A);

    cv::Matx33f decondition_homography2D(const cv::Matx33f &T_base, const cv::Matx33f &T_attach, const cv::Matx33f &H);

    std::vector<int> randomN(int n, int m);

    double homography_loss(const struct keypoints &kp1,const struct keypoints &kp2,const std::vector<cv::DMatch> &match , const cv::Matx33f &H );

    cv::Matx33f find_homography(const struct keypoints &kp1,const struct keypoints &kp2,const std::vector<cv::DMatch> &match,int max_iter = 10,int sample = 4);

    float match_quality(const struct keypoints &kp1,const cv::Mat img1,const struct keypoints &kp2,const cv::Mat img2);

}

#endif
