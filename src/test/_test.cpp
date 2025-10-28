

#include "_test.h"

namespace test {



std::vector<cv::Mat> equalizeIntensities(const std::vector<cv::Mat>& images,
                                        const std::vector<cv::Mat>& masks,
                                        const std::vector<cv::Point>& top_lefts,
                                        float ratio) {

    const int num_images = images.size();
    std::vector<cv::Mat> images_adjusted(num_images);
    std::vector<cv::Mat> masks_transformed = dcut::distance_transform(masks);

    struct util::size_data p_size = util::get_pan_dimension(top_lefts, images);

    std::vector<cv::Mat> masks_resized;
    masks_resized.reserve(num_images);
    std::vector<cv::Rect> image_roi;
    image_roi.reserve(num_images);
    std::vector<cv::Mat> intensity;
    intensity.reserve(num_images);
    std::vector<cv::Mat> intensity_dist;
    intensity_dist.reserve(num_images);

    const cv::Size kernel_size(13, 13);
    const double epsilon = 0.00001;


    for (int i = 0; i < num_images; i++) {
        cv::Mat res_mask;
        cv::resize(masks[i], res_mask, cv::Size(), ratio, ratio, cv::INTER_LINEAR);
        masks_resized.push_back(std::move(res_mask));

        cv::resize(images[i], images_adjusted[i], cv::Size(), ratio, ratio, cv::INTER_LINEAR);
        cv::resize(masks_transformed[i], masks_transformed[i], cv::Size(), ratio, ratio, cv::INTER_LINEAR);

        cv::Mat gray, gray_masked;
        cv::cvtColor(images_adjusted[i], gray, cv::COLOR_BGR2GRAY);
        gray.convertTo(gray, CV_32F, 1.0 / 255.0);
        gray.copyTo(gray_masked, masks_resized[i]);

        intensity.push_back(std::move(gray_masked));

        cv::Mat int_temp;
        cv::multiply(intensity[i], masks_transformed[i], int_temp);
        intensity_dist.push_back(std::move(int_temp));

        cv::Rect roi(
            top_lefts[i].x - p_size.min_x,
            top_lefts[i].y - p_size.min_y,
            masks[i].size().width,
            masks[i].size().height
        );

        double R = static_cast<double>(images_adjusted[i].rows) / images[i].rows;
        double C = static_cast<double>(images_adjusted[i].cols) / images[i].cols;
        roi = util::scaleRect(roi, C, R);
        image_roi.push_back(roi);
    }

    for (int i = 0; i < num_images; i++) {
        cv::Mat alpha_temp = masks_transformed[i].clone();
        cv::Mat int_temp = intensity_dist[i].clone();

        const cv::Mat& current_mask = masks_resized[i];

        for (int j = 0; j < num_images; j++) {
            if (i == j) continue;

            cv::Rect overlap = image_roi[i] & image_roi[j];
            if (overlap.empty()) continue;

            cv::Rect overlapi(overlap.x - image_roi[i].x, overlap.y - image_roi[i].y,
                            overlap.width, overlap.height);
            cv::Rect overlapj(overlap.x - image_roi[j].x, overlap.y - image_roi[j].y,
                            overlap.width, overlap.height);

            cv::add(int_temp(overlapi), intensity_dist[j](overlapj),
                   int_temp(overlapi), current_mask(overlapi));
            cv::add(alpha_temp(overlapi), masks_transformed[j](overlapj),
                   alpha_temp(overlapi), current_mask(overlapi));
        }

        cv::Mat test;
        cv::add(alpha_temp, epsilon, alpha_temp);
        cv::divide(int_temp, alpha_temp, test);
        cv::add(test, epsilon, test);

        cv::divide(intensity[i], test, test);

        cv::Mat mask_inv;
        cv::bitwise_not(masks_resized[i], mask_inv);
        mask_inv.convertTo(mask_inv, CV_32FC1, 1.0/255.0);

        cv::add(test, mask_inv, test);  // In-place addition
        cv::GaussianBlur(test, test, kernel_size, 7, 7, cv::BORDER_REFLECT);

        images_adjusted[i] = std::move(test);
    }

    return images_adjusted;
}



void adjust_intensity(std::vector<cv::Mat>& images,const std::vector<cv::Mat>& intensities){

    for(int i = 0;i < images.size();i++){

        cv::Mat test;
        cv::resize(intensities[i], test, images[i].size(), 0, 0, cv::INTER_LINEAR);
        images[i].convertTo(images[i], CV_32FC3, 1.0 / 255.0);
        imgm::elementwiseOperation(images[i], test, images[i],imgm::DIVIDE);
        images[i].convertTo(images[i], CV_8UC3, 255.0);

    }

}


}

