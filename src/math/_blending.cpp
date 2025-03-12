
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

void no_blend(const class imgm::pan_img_transform &Tr,const std::vector<cv::Mat> &imags){

    float cnst = 1e-4;
    int height = Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].yend - Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].ystart + 1;
    int wide = Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].xend - Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].xstart + 1;

    cv::Mat panorama = cv::Mat::zeros(height,wide,CV_32FC3);
    cv::Mat img;

    for (int i = 0;i < imags.size();i++){
        std::cout<<Tr.img2pan[i];
        cv::warpPerspective(imags[i],img, Tr.img2pan[i],panorama.size(),cv::INTER_LINEAR);
        img.copyTo(panorama, img);

    }



    cv::imshow("Image Display",panorama);
    cv::waitKey(0);

}


cv::Mat createSurroundingMask(const cv::Mat& inputImage, bool invert = false, uchar thresholdValue = 10) {
    if (inputImage.empty()) {
        return cv::Mat();
    }

    // [Rest of original processing remains the same...]
    cv::Mat gray;
    cv::cvtColor(inputImage, gray, cv::COLOR_BGR2GRAY);

    cv::Mat thresh;
    cv::threshold(gray, thresh, thresholdValue, 255, cv::THRESH_BINARY_INV);

    cv::Mat floodFilled = thresh.clone();
    int width = floodFilled.cols;
    int height = floodFilled.rows;

    // [Flood fill operations remain unchanged...]
    for (int x = 0; x < width; ++x) {
        if (floodFilled.at<uchar>(0, x) == 255) {
            cv::floodFill(floodFilled, cv::Point(x, 0), 0, nullptr, cv::Scalar(), cv::Scalar(), 4 | cv::FLOODFILL_FIXED_RANGE);
        }
        if (floodFilled.at<uchar>(height - 1, x) == 255) {
            cv::floodFill(floodFilled, cv::Point(x, height - 1), 0, nullptr, cv::Scalar(), cv::Scalar(), 4 | cv::FLOODFILL_FIXED_RANGE);
        }
    }

    for (int y = 0; y < height; ++y) {
        if (floodFilled.at<uchar>(y, 0) == 255) {
            cv::floodFill(floodFilled, cv::Point(0, y), 0, nullptr, cv::Scalar(), cv::Scalar(), 4 | cv::FLOODFILL_FIXED_RANGE);
        }
        if (floodFilled.at<uchar>(y, width - 1) == 255) {
            cv::floodFill(floodFilled, cv::Point(width - 1, y), 0, nullptr, cv::Scalar(), cv::Scalar(), 4 | cv::FLOODFILL_FIXED_RANGE);
        }
    }

    cv::Mat mask;
    cv::subtract(thresh, floodFilled, mask);

    // New inversion logic
    if (invert) {
        cv::bitwise_not(mask, mask);
    }

    return mask;
}


cv::Point2i get_corner(const cv::Mat &img,const cv::Matx33f &hom){

    std::vector<cv::Vec2f> cor;
    cor.push_back(cv::Vec2f(0,0));
    cor.push_back(cv::Vec2f(0,img.rows));
    cor.push_back(cv::Vec2f(img.cols,0));

    cv::perspectiveTransform(cor, cor, hom);
    int xstart = (int)std::min( cor[0][0], cor[1][0]);
    int ystart = (int)std::min( cor[0][1], cor[2][1]);

    cv::Point2i ret = cv::Point(xstart, ystart);

    return ret;
}


cv::Size get_size(const cv::Mat &img,const cv::Matx33f &hom){

    std::vector<cv::Vec2f> cor;

    cor.push_back(cv::Vec2f(0,0));
    cor.push_back(cv::Vec2f(0,img.rows));
    cor.push_back(cv::Vec2f(img.cols,0));
    cor.push_back(cv::Vec2f(img.cols,img.rows));

    cv::perspectiveTransform(cor, cor, hom);


    float xstart = std::min( cor[0][0], cor[1][0]);
    float xend   = std::max( cor[2][0], cor[3][0]);
    float ystart = std::min( cor[0][1], cor[2][1]);
    float yend   = std::max( cor[1][1], cor[3][1]);

    cv::Size sz;
    sz.width = cvRound(abs(xstart - xend));
    sz.height = cvRound(abs(ystart - yend));

    return sz;
}


std::vector<cv::Mat> seam_finder(std::vector<cv::Mat> sources,std::vector<cv::Point2i> corners){

    std::vector<cv::UMat> images;
    std::vector<cv::UMat> masks;
    for(int i = 0;i<sources.size();i++){
        cv::UMat copy;
        sources[i].copyTo(copy);
        images.push_back(copy);
        images[i].convertTo(images[i], CV_32FC3, 1/255.0);
        cv::UMat copy_mask;
        cv::Mat msk = createSurroundingMask(sources[i],true, 10);
        msk.copyTo(copy_mask);
        masks.push_back(copy_mask);
    }

    //cv::Ptr<cv::detail::SeamFinder> seam_finder = new cv::detail::GraphCutSeamFinder(cv::detail::GraphCutSeamFinderBase::COST_COLOR, 0.11, 33);
    cv::Ptr<cv::detail::SeamFinder> seam_finder = new cv::detail::GraphCutSeamFinder();
    seam_finder->find(images, corners, masks);

    std::vector<cv::Mat> cvmasks;
    for(int i = 0;i<sources.size();i++){
        cv::Mat m;
        masks[i].getMat(cv::ACCESS_READ).copyTo(m);
        cvmasks.push_back(m);
        masks[i].release();
    }

    return cvmasks;
}


void graph_blend(const class imgm::pan_img_transform &Tr,const std::vector<cv::Mat> &imags){

    float cnst = 1e-4;
    int height = Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].yend - Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].ystart + 1;
    int wide = Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].xend - Tr.translation[Tr.stitch_order[Tr.stitch_order.size()-1]].xstart + 1;

    std::vector<cv::Point2i> corners;
    std::vector<cv::Mat> img;

    for (int i = 0;i < imags.size();i++){
        cv::Point2i a = get_corner(imags[i],Tr.img2pan[i]);

        cv::Matx33f T = cv::Matx33f::zeros();
        T(0, 0) = 1;
        T(1, 1) = 1;
        T(2, 2) = 1;
        T(0, 2) = -a.x;
        T(1, 2) = -a.y;

        cv::Size sz = get_size(imags[i],Tr.img2pan[i]);
        cv::Mat im;
        cv::warpPerspective(imags[i], im, T * Tr.img2pan[i],sz , cv::INTER_LINEAR);

        corners.push_back(a);
        img.push_back(im);
    }

    cv::Mat panorama = cv::Mat::zeros(height+1,wide+1,img[0].type());
    std::vector<cv::Mat> masks = seam_finder(img,corners);
    for (int i = 0;i < imags.size();i++){
        img[i].copyTo(panorama(cv::Range(corners[i].y, img[i].rows + corners[i].y), cv::Range(corners[i].x, img[i].cols + corners[i].x)),masks[i]);

    }


    cv::imshow("Image Display",panorama);
    cv::waitKey(0);

}


}

