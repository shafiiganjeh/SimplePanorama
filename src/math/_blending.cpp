
#include "_blending.h"


namespace blnd {

void simple_blend(const class imgm::pan_img_transform &Tr,const std::vector<cv::Mat> &imags){

    float cnst = 1e-4;
    int height = Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].yend - Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].ystart + 1;
    int wide = Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].xend - Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].xstart + 1;

    cv::Mat panorama = cv::Mat::zeros(height,wide,CV_32FC3);
    cv::Mat panorama_mask = cv::Mat::zeros(height,wide,CV_32FC3);
    panorama_mask= panorama_mask+ cnst;
    panorama= panorama+ cnst;

    cv::Mat dstT1;
    cv::Mat dstT2;
    int img_TYPE = imags[0].type();
    std::cout<< "image size: " <<panorama.size()<<"\n";

    cv::Mat temp_display;

    for (int i = 0;i < imags.size();i++){

        cv::Mat Msk;
        cv::Mat color_img;

        cv::warpPerspective(imags[i], color_img, Tr.img2pan[i],panorama.size() , cv::INTER_LINEAR);
        cv::cvtColor(color_img, Msk, cv::COLOR_BGR2GRAY);
        cv::threshold(Msk, Msk, 3, 255, cv::THRESH_BINARY);

        cv::distanceTransform(Msk, Msk, cv::DIST_L2, 3);
        normalize(Msk, Msk, 0, 1.0, cv::NORM_MINMAX);
        cv::Mat Msk_3channel;
        cv::Mat t[] = {Msk, Msk, Msk};
        cv::merge(t, 3, Msk_3channel);

        Msk_3channel.convertTo(Msk_3channel, CV_32F);
        color_img.convertTo(color_img, CV_32F);

        panorama = panorama + color_img.mul(Msk_3channel);
        panorama_mask = (panorama_mask + Msk_3channel);

    }

    panorama = panorama.mul(1/panorama_mask);
    panorama.convertTo(panorama, imags[0].type());
    cv::imshow("Image Display",panorama);
    cv::waitKey(0);

}


}

