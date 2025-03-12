

#ifndef MATHS_H
#define MATHS_H

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <numeric>
#include <algorithm>
#include <random>
#include <thread>
#include <future>
#include <vector>
#include <stack>
#include <unordered_set>
#include <map>

namespace maths {

    struct adj_str {
            cv::Mat adj;
            int nodes;
        };

    struct keypoints{

        std::vector<cv::KeyPoint> keypoint;
        cv::Mat descriptor;

    };

    struct Homography{

        cv::Matx33f H;
        cv::Matx33f norm_b;
        cv::Matx33f norm_a;

    };

    struct translation{
        cv::Matx33f T;
        float xstart;
        float xend;
        float ystart;
        float yend;
    };

    enum GeometryType {
    GEOM_TYPE_POINT,
    GEOM_TYPE_LINE,
    };

    using thread = std::vector<std::vector<std::vector<int>>>;

    struct keypoints extract_keypoints(const cv::Mat &img,int nfeatures = 0,int nOctaveLayers = 3,double contrastThreshold = 0.06,double edgeThreshold = 10,double sigma = 1.6);

    std::pair<std::vector<cv::DMatch>, std::vector<cv::DMatch>> match_keypoints(const struct keypoints &kp1,const struct keypoints &kp2);

    cv::Matx33f Normalize2D(const std::vector<cv::Vec3f> &points);

    std::vector<cv::Vec3f> applyH_2D(const std::vector<cv::Vec3f>& geomObjects, const cv::Matx33f &H, GeometryType type);

    cv::Vec3f eucl2hom_point_2D(const cv::Vec2f& p);

    cv::Vec2f hom2eucl_point_2D(const cv::Vec3f& p);

    cv::Mat_<float> getDesignMatrix_homography2D(const std::vector<cv::Vec3f> &conditioned_base, const std::vector<cv::Vec3f> &conditioned_attach);

    float N_outliers(const struct keypoints &kp1,const struct keypoints &kp2,std::vector<cv::DMatch> &match,const  cv::Matx33f &H,std::vector<float> &T);

    cv::Matx33f solve_homography2D(const cv::Mat_<float> &A);

    cv::Matx33f decondition_homography2D(const cv::Matx33f &T_base, const cv::Matx33f &T_attach, const cv::Matx33f &H);

    std::vector<int> randomN(int n, int m);

    double homography_loss(const struct keypoints &kp1,const struct keypoints &kp2,const std::vector<cv::DMatch> &match , const cv::Matx33f &H );

    struct Homography find_homography(const struct keypoints &kp1,const struct keypoints &kp2,const std::vector<cv::DMatch> &match,int max_iter,int sample,const cv::Mat &img1,const cv::Mat &img2);

    float match_quality(const struct keypoints &kp1,const cv::Mat img1,const struct keypoints &kp2,const cv::Mat img2);

    std::vector<maths::keypoints> extrace_kp_vector(const std::vector<cv::Mat> & imgs,std::vector<int> idx);

    struct translation get_translation(const cv::Mat &base, const cv::Mat &attach,const cv::Matx33f &H);

    std::map<int, std::pair<int,double>> path_table(const cv::Mat& adj,const std::vector<std::pair<int, std::vector<int>>> &nodes,int start);

    std::vector<int> dfs(cv::Mat& graph, int source);

    std::vector<std::pair<int, std::vector<int>>> bfs_ordered_with_neighbors(const cv::Mat& adj, int i);

    float focal_from_hom(const std::vector<std::vector< cv::Matx33f >> & H_mat,const cv::Mat &source_adj);

    template <typename T>
    std::vector<std::vector<T>> splitVector(const std::vector<T>& vec, int n);

    std::vector<struct adj_str> extract_adj(const cv::Mat &adj);


    class adj_calculator{

    public:

        const double MAHALANOBIS_THRESHOLD = std::sqrt(5.991);

        adj_calculator(const std::vector<cv::Mat> & imgs,const std::vector<maths::keypoints> &key_p);

        void cal_adj(const std::vector<cv::Mat> & imgs,int T);

        float match_quality(const struct maths::keypoints &kp1,const cv::Mat img1,const struct maths::keypoints &kp2,const cv::Mat img2,int row,int col);

        void get_threads(int n);

        std::vector<cv::DMatch> clean_matches(const struct maths::keypoints &kp1,const struct maths::keypoints &kp2,std::vector<cv::DMatch> match,const  cv::Matx33f &H,const std::vector<float> &T12);

        std::vector<cv::DMatch> filterOutliersWithMahalanobis(const std::vector<cv::KeyPoint>& kp1,const std::vector<cv::KeyPoint>& kp2,const std::vector<cv::DMatch>& matches,const std::vector<cv::DMatch>& ransac_matches,const cv::Matx33f& H, float chi2_threshold = 10 );

        cv::Mat adj;
        std::vector<maths::keypoints> kpmat;
        std::vector<std::vector< cv::Matx33f >> hom_mat;
        std::vector<std::vector<std::vector<cv::DMatch>>> match_mat;
        thread TR;

};


}

#endif
