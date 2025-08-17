
#include "_gain_compensation.h"

namespace gain {

//overlap intensity in the spherical projection.
std::vector<OverlapInfo> get_overlapp_intensity(
    const std::vector<cv::Mat>& warped_images,
    const std::vector<cv::Point>& corners,
    const cv::Mat& adj_pass) {

    cv::Mat adj = adj_pass;
    adj = adj + cv::Mat::eye(adj.rows,adj.cols,adj.type());

    std::vector<cv::Mat> gray_images;
    std::vector<cv::Mat> masks;

    for (const auto& img : warped_images) {
        cv::Mat gray;
        if (img.channels() == 3) {
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        } else if (img.channels() == 4) {
            cv::cvtColor(img, gray, cv::COLOR_BGRA2GRAY);
        } else {
            gray = img.clone();
        }
        gray_images.push_back(gray);
        masks.push_back(blnd::createSurroundingMask(img,true, 1));
    }

    std::vector<OverlapInfo> results;

    for (int i = 0; i < adj.rows; ++i) {
        for (int j = i; j < adj.cols; ++j) {
            if (adj.at<double>(i, j) > 0) {
                const cv::Point& corner_i = corners[i];
                const cv::Point& corner_j = corners[j];
                const cv::Mat& img_i = warped_images[i];
                const cv::Mat& img_j = warped_images[j];

                cv::Rect rect_i(corner_i.x, corner_i.y, img_i.cols, img_i.rows);
                cv::Rect rect_j(corner_j.x, corner_j.y, img_j.cols, img_j.rows);

                cv::Rect overlap_rect = rect_i & rect_j;
                if (overlap_rect.width <= 0 || overlap_rect.height <= 0) {
                    results.push_back({i, j, 0.0, 0.0, 0.0});
                    continue;
                }

                cv::Rect roi_i_local(overlap_rect.x - corner_i.x, overlap_rect.y - corner_i.y, overlap_rect.width, overlap_rect.height);
                cv::Rect roi_j_local(overlap_rect.x - corner_j.x, overlap_rect.y - corner_j.y, overlap_rect.width, overlap_rect.height);

                cv::Mat mask_i_roi = masks[i](roi_i_local);
                cv::Mat mask_j_roi = masks[j](roi_j_local);
                cv::Mat gray_i_roi = gray_images[i](roi_i_local);
                cv::Mat gray_j_roi = gray_images[j](roi_j_local);

                cv::Mat combined_mask;
                cv::bitwise_and(mask_i_roi, mask_j_roi, combined_mask);

                double area = cv::countNonZero(combined_mask);

                cv::Mat masked_i, masked_j;
                cv::bitwise_and(gray_i_roi, gray_i_roi, masked_i, combined_mask);
                cv::bitwise_and(gray_j_roi, gray_j_roi, masked_j, combined_mask);
                double I_i = cv::sum(masked_i)[0];
                double I_j = cv::sum(masked_j)[0];

                results.push_back({i, j, area, I_i, I_j});
            }
        }
    }

    return results;
}


    Eigen::MatrixXd calc_B(const std::vector<std::vector<double>> &N_ij,int size){

        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(size,size);
        for(int i = 0;i < size;i++){

            for(int j = 0;j < size;j++){

                B(i,i) += N_ij[i][j];

            }

        }
        return B;
    }


    Eigen::MatrixXd calc_A(const std::vector<std::vector<double>> &N_ij,const std::vector<std::vector<double>> &I_ij,int size){

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(size,size);
        for(int i = 0;i < size;i++){

            for(int j = 0;j < size;j++){

                A(i,i) += N_ij[i][j] * I_ij[i][j] * I_ij[i][j];

            }

        }
        return A;
    }


    Eigen::MatrixXd calc_C(const std::vector<std::vector<double>> &N_ij,const std::vector<std::vector<double>> &I_ij,int size){

        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(size,size);
        for(int i = 0;i < size;i++){

            for(int j = 0;j < size;j++){

                C(i,j) = N_ij[i][j] * I_ij[i][j] * I_ij[j][i];

            }


        }
        return C;
    }


    std::pair<Eigen::VectorXd,Eigen::MatrixXd> set_up_equations(const std::vector<cv::Mat> &imags,const cv::Mat& adj,const std::vector<cv::Point>& corners){

            double S_N_exp = 100;
            double S_g_exp = .01;
            int size = imags.size();

            std::vector<std::vector<double>> N_ij(size, std::vector<double>(size, 0));
            std::vector<std::vector<double>> I_ij(size, std::vector<double>(size, 0));
            std::vector<OverlapInfo> info = get_overlapp_intensity(imags,corners,adj);

            for (int i = 0;i < info.size();i++){

                N_ij[info[i].i][info[i].j] = info[i].area;
                N_ij[info[i].j][info[i].i] = info[i].area;
                I_ij[info[i].i][info[i].j] = info[i].I_j / info[i].area;
                I_ij[info[i].j][info[i].i] = info[i].I_i / info[i].area;

            }

            Eigen::MatrixXd B = calc_B(N_ij,size);
            Eigen::MatrixXd A = calc_A(N_ij,I_ij,size);
            Eigen::MatrixXd C = calc_C(N_ij,I_ij,size);

            Eigen::VectorXd G = (1/S_g_exp) * B.diagonal();

            Eigen::MatrixXd M = (2/S_N_exp) * (A - C) + (1/S_g_exp) * B;

            return std::pair<Eigen::VectorXd,Eigen::MatrixXd>(G,M);
    }


    std::vector<double> gain_compensation(const std::vector<cv::Mat> &imags,const cv::Mat& adj_pass,const std::vector<cv::Point>& corners){

        cv::Mat adj = adj_pass;
        adj = adj + cv::Mat::eye(adj.rows,adj.cols,adj.type());
        std::pair<Eigen::VectorXd,Eigen::MatrixXd> G;
        //setting up de/dg = 0 in matrix form equation (29)
        G = gain::set_up_equations(imags,adj,corners);

        //solve for all g _i vectors optimal brightness is then image_i / g_i
        Eigen::VectorXd x = G.second.colPivHouseholderQr().solve(G.first);

        std::vector<double> ret(x.data(), x.data() + x.size());
        return ret;

    }

}
