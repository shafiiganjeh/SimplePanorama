
#ifndef PROJ_H
#define PROJ_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "_blending.h"
#include "_util.h"
#include <cmath>

namespace proj {


struct proj_data {
    std::vector<cv::Mat> imgs;
    std::vector<cv::Mat> msks;
    std::vector<cv::Point> corners;
};


struct bounding_data {
    std::vector<cv::Rect> org_bbox;
    std::vector<cv::Rect> tr_bbox;
};

struct polar {
    float phi;
    float r;
};


struct kartesian {
    float x;
    float y;
};


struct kartesian Polar2Kartesian(struct polar & p);


struct polar Kartesian2Polar(struct kartesian & k);


struct warped{

    cv::Mat imgs;
    cv::Point corners;

};


class projection {

        public:
            virtual ~projection() = default;
            virtual struct warped project(const Eigen::MatrixXd & R, const Eigen::MatrixXd & K, const cv::Mat & img) const = 0;
            virtual void change_focal(float f)  = 0;

};


class spherical_proj : public projection {

    private:

        cv::Ptr<cv::detail::SphericalWarper> warper;
        float focal;

    public:

        spherical_proj(float f):focal(f){

            warper = cv::makePtr<cv::detail::SphericalWarper>(f);

        }

        struct warped project(const Eigen::MatrixXd & R, const Eigen::MatrixXd & K, const cv::Mat & img) const override;

        void change_focal(float f) override {focal = f;
            warper = cv::makePtr<cv::detail::SphericalWarper>(f);}

};


class cylindrical_proj : public projection {

    private:

        cv::Ptr<cv::detail::CylindricalWarper> warper;
        float focal;

    public:

        cylindrical_proj(float f):focal(f){

            warper = cv::makePtr<cv::detail::CylindricalWarper>(f);

        }

        struct warped project(const Eigen::MatrixXd & R, const Eigen::MatrixXd & K, const cv::Mat & img) const override;

        void change_focal(float f) override {focal = f;
            warper = cv::makePtr<cv::detail::CylindricalWarper>(f);
        }

};


class sten_proj : public projection {

    private:

        cv::Ptr< cv::detail::RotationWarper> warper;
        float focal;
        cv::Point ansatz;
        float radius;
        struct bounding_data get_bounding_box(struct proj_data &proj,util::RadialNormalizer & norm,bool quadratic = true);
        int precision = 1000;

    public:

        bool has_circle = false;
        float return_rad(float scale = 1){return radius*scale;}
        cv::Point return_ansatz(float scale = 1){
            cv::Point temp;
            temp.x = ansatz.x * scale;
            temp.y = ansatz.y * scale;
            return temp;}

        sten_proj(float f):focal(f){

            warper = cv::makePtr<cv::detail::StereographicWarper>(f);

        }

        struct warped project(const Eigen::MatrixXd & R, const Eigen::MatrixXd & K, const cv::Mat & img) const override;

        void disk_reproj(struct proj_data &proj,bool quadratic = true);
        void inpaint(cv::Mat & img,const cv::Point& ansatz,float radius);
        std::pair<cv::Point,float> estimate_circle(struct proj_data &proj);
        void force_circle(cv::Point a,float r){

            ansatz = a;
            radius = r;

        }

        void change_focal(float f) override {focal = f;
            warper = cv::makePtr<cv::detail::StereographicWarper>(f);
        }

};

struct proj_data get_proj_parameters(
    const std::vector<cv::Mat>& images,
    std::vector<Eigen::MatrixXd>& R,
    std::vector<Eigen::MatrixXd>& K,
    std::vector<double> &con,
    std::shared_ptr<projection> projector,
    bool get_masks = true);
}


#endif
