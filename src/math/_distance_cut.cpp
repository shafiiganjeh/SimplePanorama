
#include "_distance_cut.h"

namespace dcut {


    std::vector<cv::Mat> dist_cut(const std::vector<cv::Mat>& masks,const std::vector<cv::Point>& top_lefts){

        std::vector<cv::Mat> transformed = distance_transform(masks);

        std::vector<cv::Rect> image_roi;
        for (int i = 0; i < masks.size(); i++) {
            cv::Rect roi(
                top_lefts[i].x,
                top_lefts[i].y,
                masks[i].size().width,
                masks[i].size().height
            );
            image_roi.push_back(roi);
        }

        std::vector<cv::Mat> cut_mask;

        for(int i = 0;i < masks.size();i++){
            cv::Mat temp = masks[i].clone();

            for(int j = 0;j < masks.size();j++){

                cv::Rect overlap = image_roi[i] bitand image_roi[j];
                if(overlap.empty() or (i == j)) continue;

                cv::Rect overlapi(overlap.x - image_roi[i].x,overlap.y -image_roi[i].y,overlap.width,overlap.height);
                cv::Rect overlapj(overlap.x - image_roi[j].x,overlap.y -image_roi[j].y,overlap.width,overlap.height);

                cv::Mat diff = transformed[i](overlapi) - transformed[j](overlapj);
                cv::Mat sub_mask;
                cv::threshold(-diff, sub_mask, 0, 1, cv::THRESH_BINARY);

                sub_mask.convertTo(sub_mask, masks[i].type());
                sub_mask = (1 - sub_mask);

                temp(overlapi) = temp(overlapi).mul(sub_mask);

            }
            temp.convertTo(temp, masks[i].type());
            cut_mask.push_back(temp);

        }

            return cut_mask;
    }





    std::vector<cv::Mat> distance_transform(const std::vector<cv::Mat>& masks){

        std::vector<cv::Mat> ret;

        for(int i = 0;i < masks.size();i++){

            cv::Mat transformed;
            cv::distanceTransform(masks[i], transformed, cv::DIST_L2, cv::DIST_MASK_5, CV_32F);
            transformed = transformed /255;
            ret.push_back(transformed);

        }


        return ret;

    }



}

