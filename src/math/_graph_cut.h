
#ifndef GRAPH_H
#define GRAPH_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <iostream>
#include "graph.h"
#include "_graph_cut_helper.h"
#include "_util.h"

namespace gcut {


    struct edge_value{

        float v;
        float h;

    };


    class gradient {
        public:
            virtual ~gradient() = default;
            virtual edge_value read(int i, int j) const = 0;
    };


    class simple_gradient : public gradient {

        private:

            float mask_constant = 255;
            cv::Mat a_dif;
            const cv::Mat& M1;
            const cv::Mat& M2;
            uchar read_Mat(int i,const cv::Mat* mat) const;

        public:

            simple_gradient(const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& mask1, const cv::Mat& mask2):  M1(mask1),M2(mask2){

                CV_Assert(img1.size() == img2.size() && img1.type() == CV_8UC1 && img2.type() == CV_8UC1);
                CV_Assert(mask1.size() == img1.size() && mask1.type() == CV_8U);
                CV_Assert(mask2.size() == img1.size() && mask2.type() == CV_8U);

                cv::absdiff(img1, img2, a_dif);

            }

            edge_value read(int i, int j) const override;

    };


    class scharr_gradient : public gradient {

        private:

            float eps = 1e-6f;
            float mask_constant = 255;
            cv::Mat a_dif;
            cv::Mat M1;
            cv::Mat M2;

            cv::Mat grad1_x, grad1_y, grad2_x, grad2_y;

            float read_Mat(int i,const cv::Mat* mat) const;


        public:

            scharr_gradient(const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& mask1, const cv::Mat& mask2){

                CV_Assert(img1.size() == img2.size() && img1.type() == CV_8UC1 && img2.type() == CV_8UC1);
                CV_Assert(mask1.size() == img1.size() && mask1.type() == CV_8U);
                CV_Assert(mask2.size() == img1.size() && mask2.type() == CV_8U);

                cv::Mat f_img1, f_img2;
                img1.convertTo(f_img1, CV_32F);
                img2.convertTo(f_img2, CV_32F);
                mask1.convertTo(M1, CV_32F);
                mask2.convertTo(M2, CV_32F);

                cv::Scharr(f_img1, grad1_x, CV_32F, 1, 0);
                cv::Scharr(f_img1, grad1_y, CV_32F, 0, 1);
                cv::Scharr(f_img2, grad2_x, CV_32F, 1, 0);
                cv::Scharr(f_img2, grad2_y, CV_32F, 0, 1);

                cv::absdiff(f_img1, f_img2, a_dif);

            }

            edge_value read(int i, int j) const override;

    };


    class EdgeWeight {
        private:
            const gradient& weightFunc;

        public:
            EdgeWeight(const gradient& wf) : weightFunc(wf){}

            edge_value getWeight(int i, int j) const {
                return weightFunc.read(i, j);
            }
    };

    cv::Mat computeOverlapMatrix(const std::vector<cv::Rect>& rois);

    std::vector<cv::Mat> graph_cut(const std::vector<cv::Mat>& images,const std::vector<cv::Mat>& masks,const std::vector<cv::Point>& top_lefts,std::vector<int>& seq,std::atomic<double>* f_adress = NULL,std::atomic<bool>* c_adress = NULL);


    std::vector<int> define_graph_full(class gcut::graph_object &obj,class EdgeWeight &reader);


    cv::Mat computeCut(
        const cv::Mat& img1,
        const cv::Mat& img2,
        const cv::Mat& mask1,
        const cv::Mat& mask2);



}

#endif
