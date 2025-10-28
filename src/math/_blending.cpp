
#include "_blending.h"

namespace blnd {

//same as cv::erode but with costum kernel and superior boundry handling
void erode(const cv::Mat& tmp, cv::Mat& dst, const cv::Mat& kernel) {
    cv::Mat src = tmp.clone();
    dst.create(src.size(), src.type());
    int kHeight = kernel.rows;
    int kWidth = kernel.cols;
    int kCenterY = kHeight / 2;
    int kCenterX = kWidth / 2;

    std::vector<std::pair<int, int>> kernelOffsets;
    for (int ky = 0; ky < kHeight; ky++) {
        for (int kx = 0; kx < kWidth; kx++) {
            if (kernel.at<uchar>(ky, kx) != 0) {
                kernelOffsets.push_back({ky - kCenterY, kx - kCenterX});
            }
        }
    }

    const int rows = src.rows;
    const int cols = src.cols;

    #pragma omp parallel for
    for (int y = 0; y < rows; y++) {
        uchar* dst_row = dst.ptr<uchar>(y);

        for (int x = 0; x < cols; x++) {
            uchar minVal = 255;

            for (const auto& offset : kernelOffsets) {
                int ny = y + offset.first;
                int nx = x + offset.second;

                if (ny >= 0 && ny < rows && nx >= 0 && nx < cols) {
                    uchar val = src.at<uchar>(ny, nx);
                    if (val == 0) {
                        minVal = 0;
                        break;  // found black pixel
                    }
                    minVal = std::min(minVal, val);
                } else {
                    minVal = 0;  // Outside is black
                    break;       // Early exit
                }
            }
            dst_row[x] = minVal;
        }
    }
}


cv::Mat getStructuringElementDiamond(int size) {

    if (size <= 0) {
        throw std::invalid_argument("Diamond kernel size must be positive");
    }
    if (size % 2 == 0) {
        throw std::invalid_argument("Diamond kernel size must be odd");
    }

    int radius = size / 2;
    cv::Mat kernel = cv::Mat::zeros(size, size, CV_8U);
    int center = radius;

    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            int dist = std::abs(y - center) + std::abs(x - center);
            if (dist <= radius) {
                kernel.at<uchar>(y, x) = 1;
            }
        }
    }

    return kernel;
}


//aka alpha blending
cv::Mat simple_blend(const std::vector<cv::Mat>& images,const std::vector<cv::Mat>& masks,const std::vector<cv::Point>& top_lefts) {

    if (images.empty() or images.size() != masks.size() or images.size() != top_lefts.size()) {
        throw std::runtime_error("Input consistency!");
    }

    struct util::size_data p_size = util::get_pan_dimension(top_lefts,images);

    cv::Mat accumulated_color = cv::Mat::zeros(p_size.dims, CV_32FC3);
    cv::Mat accumulated_alpha = cv::Mat::zeros(p_size.dims, CV_32FC1);

    //define a ROI in the panorama and caluclate the alpha mask based on distance transform via adding color values and alpha mask values.
    for (size_t i = 0; i < images.size(); i++) {
        const cv::Mat& img = images[i];
        const cv::Mat& mask = masks[i];
        cv::Point tl = top_lefts[i];
        cv::Point canvas_tl(tl.x - p_size.min_x, tl.y - p_size.min_y);

        cv::Rect roi(canvas_tl.x, canvas_tl.y, img.cols, img.rows);
        roi = roi & cv::Rect(0, 0, p_size.dims.width, p_size.dims.height);
        if (roi.width <= 0 || roi.height <= 0) continue;

        cv::Rect img_roi(roi.x - canvas_tl.x, roi.y - canvas_tl.y, roi.width, roi.height);
        cv::Mat img_cropped = img(img_roi).clone();
        cv::Mat mask_cropped = mask(img_roi).clone();

        cv::Mat dt;
        cv::distanceTransform(mask_cropped, dt, cv::DIST_L2, cv::DIST_MASK_5, CV_32F);
        cv::Mat mask_float;
        normalize(dt, mask_float, 0.0, 1.0, cv::NORM_MINMAX);

        cv::Mat img_float;
        img_cropped.convertTo(img_float, CV_32F, 1.0 / 255.0);

        cv::Mat acc_color_roi = accumulated_color(roi);
        cv::Mat acc_alpha_roi = accumulated_alpha(roi);

        cv::Mat one_minus_alpha;
        cv::subtract(cv::Scalar(1.0), acc_alpha_roi, one_minus_alpha);

        cv::Mat mask_3c;
        cv::Mat one_minus_alpha_3c;
        cv::cvtColor(mask_float, mask_3c, cv::COLOR_GRAY2BGR);
        cv::cvtColor(one_minus_alpha, one_minus_alpha_3c, cv::COLOR_GRAY2BGR);

        cv::Mat new_color = img_float.mul(mask_3c);
        cv::Mat blended_color = new_color.mul(one_minus_alpha_3c);

        cv::Mat blended_alpha = mask_float.mul(one_minus_alpha);

        acc_color_roi += blended_color;
        acc_alpha_roi += blended_alpha;
    }

    //divide the accumulated colors with alpha values for final result
    cv::Mat result_float = cv::Mat::zeros(p_size.dims.height, p_size.dims.width, CV_32FC3);
    for (int y = 0; y < p_size.dims.height; ++y) {
        for (int x = 0; x < p_size.dims.width; ++x) {
            float a = accumulated_alpha.at<float>(y, x);
            if (a > 0) {
                cv::Vec3f color = accumulated_color.at<cv::Vec3f>(y, x);
                result_float.at<cv::Vec3f>(y, x) = color / a;
            }
        }
    }

    cv::Mat result;
    result_float.convertTo(result, CV_8UC3, 255.0);

    return result;
}


//simple copy to panorama for testing
cv::Mat no_blend(const std::vector<cv::Mat>& images,const std::vector<cv::Mat>& masks,const std::vector<cv::Point>& top_lefts){

    struct util::size_data dim = util::get_pan_dimension(top_lefts,images);

    cv::Mat panorama(
        dim.dims.height,
        dim.dims.width,
        CV_8UC3,
        cv::Scalar(0, 0, 0)
    );

    for (int i = 0; i < images.size(); i++) {

        cv::Rect roi(
            top_lefts[i].x - dim.min_x,  // x offset
            top_lefts[i].y - dim.min_y,  // y offset
            images[i].size().width,
            images[i].size().height
        );

        images[i].copyTo(panorama(roi),masks[i]);

    }

    return panorama;
}



cv::Mat multi_blend(const std::vector<cv::Mat>& images,const std::vector<cv::Mat>& masks,const std::vector<cv::Mat>& masks_orig,const std::vector<cv::Point>& top_lefts,int bands,double sigma){

    struct util::size_data p_size = util::get_pan_dimension(top_lefts,images);

    cv::Mat accumulated_color = cv::Mat::zeros(p_size.dims, CV_32FC3);
    cv::Mat accumulated_alpha = cv::Mat::zeros(p_size.dims, CV_32FC1);

    for(int i = 0 ; i < bands ; i++){

        double sigma_band = sqrt(2*(bands - i)+1) * sigma;
        cv::Size kernel_size;
        kernel_size.height = 2 * ceil(3 * sigma) + 1;
        kernel_size.width = 2 * ceil(3 * sigma) + 1;

        for(int j = 0 ; j < masks.size() ; j++){

            cv::Mat i_conv = images[j];
            i_conv.convertTo(i_conv, CV_32FC3);
            cv::Mat I_temp;

            cv::Mat w_conv = masks[j];
            w_conv.convertTo(w_conv, CV_32FC3);

            cv::GaussianBlur(i_conv,I_temp,kernel_size,sigma_band,sigma_band,cv::BORDER_REFLECT );
            cv::GaussianBlur(w_conv,w_conv,kernel_size,sigma_band,sigma_band,cv::BORDER_REFLECT );
            w_conv = w_conv/ 255;

            if (i == bands - 1){

                I_temp = i_conv - I_temp;

            }else if(i > 0){

                double sigma_prev = sqrt(2*(bands - i - 1)+1) * sigma;
                cv::Mat prev_I;
                cv::GaussianBlur(i_conv,prev_I,kernel_size,sigma_prev,sigma_prev,cv::BORDER_REFLECT );
                I_temp = I_temp - prev_I;

            }

            cv::Mat org_mask_temp = masks_orig[j].clone();

            cv::bitwise_not(org_mask_temp, org_mask_temp);

            w_conv.setTo(0,org_mask_temp);

            cv::Mat color_tmp;
            imgm::elementwiseOperation(I_temp,w_conv,color_tmp);

            cv::Point tl = top_lefts[j];
            cv::Point canvas_tl(tl.x - p_size.min_x, tl.y - p_size.min_y);
            cv::Rect roi(canvas_tl.x, canvas_tl.y, images[j].cols, images[j].rows);

            cv::add(accumulated_color(roi),color_tmp,accumulated_color(roi));
            cv::add(accumulated_alpha(roi),w_conv,accumulated_alpha(roi));

        }

    }

    imgm::elementwiseOperation(accumulated_color,accumulated_alpha,accumulated_color,imgm::DIVIDE);
    const float divisor = 255 / bands;
    accumulated_color = accumulated_color / divisor;

    return accumulated_color;

}

cv::Mat createSurroundingMask_backup(const cv::Mat& inputImage, bool invert, uchar thresholdValue) {
    if (inputImage.empty()) {
        return cv::Mat();
    }
    int thresholdType;
    if(not invert){
        thresholdType = cv::THRESH_BINARY_INV;
    }else{
        thresholdType = cv::THRESH_BINARY;
    }

    cv::Mat gray;
    cv::cvtColor(inputImage, gray, cv::COLOR_BGR2GRAY);

    cv::threshold( gray, gray, 1, 255, thresholdType);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    erode(gray, gray, kernel);


    return gray;

}

cv::Mat createSurroundingMask(const cv::Mat& inputImage, bool invert, uchar thresholdValue) {
    if (inputImage.empty()) {
        return cv::Mat();
    }

    cv::Mat gray;
    cv::cvtColor(inputImage, gray, cv::COLOR_BGR2GRAY);

    cv::Mat thresh;
    cv::threshold(gray, thresh, thresholdValue, 255, cv::THRESH_BINARY_INV);

    cv::Mat floodFilled = thresh.clone();
    int width = floodFilled.cols;
    int height = floodFilled.rows;

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

    //cv::Mat kernel = cv::getStructuringElement(cv::MORPH_DIAMOND, cv::Size(5, 5)); not implemented in older opencv versions
    cv::Mat kernel = getStructuringElementDiamond(5);

    cv::erode(gray, gray, kernel);

    if (invert) {
        cv::bitwise_not(mask, mask);
    }

    return mask;
}


}

