            double px = base.cols / 2;
            double py = base.rows / 2;

            double D = H(2,0) * px + H(2,1) * py + 1;
            double dxdX = (H(0,0) * D - (H(0,0) * px + H(0,1) * py + H(0,2)) * H(2,0)) / D * D;
            double dydX = (H(1,0) * D - (H(1,0) * px + H(1,1) * py + H(1,2)) * H(2,0)) / D * D;

            double angle = atan(dydX/dxdX);
            cv::Vec3d u(base.cols/2, base.rows/2, 1);

            //double px_tr = px * sqrt(H(0,0)*H(0,0) + H(1,0)*H(1,0));
            double px_tr = px * sqrt(dxdX * dxdX + dydX * dydX);

            //std::cout<<"\n"<<"x_tr: "<<px_tr<<" test "<<test<<"\n";

            u = H * u;
            u = u / u[2];

            cv::Mat rota;
            rota = rotate(-1*angle,u[0],u[1]);

            double Th = sin(angle) * px_tr;
            double Tw = px_tr - cos(angle) * px_tr;
            std::cout<<"\n"<<"l: "<<(base.cols / 2)<<" lt "<<px_tr<<"\n";
            cv::Matx33d T = cv::Matx33d::zeros();
            T(0, 0) = 1;
            T(1, 1) = 1;
            T(2, 2) = 1;
            T(0, 2) = Tw;
            T(1, 2) = Th;
            std::cout<<"\n"<<"Th: "<<Th<<" Tw "<<Tw<<"\n";
            rota = T * rota;

            cv::Matx33f matFloat = static_cast<cv::Matx33f>(rota);

            return matFloat;



            std::vector<cv::DMatch> adj_calculator::filterOutliersWithMahalanobis(const std::vector<cv::KeyPoint>& kp1,const std::vector<cv::KeyPoint>& kp2,const std::vector<cv::DMatch>& matches,const std::vector<cv::DMatch>& ransac_matches,const cv::Matx33f& H, float chi2_threshold) {
    // Step 1: Compute residuals from RANSAC inliers

    std::vector<cv::Point2f> residuals;
    std::vector<cv::DMatch> clean_matches;

    for (const auto& m : ransac_matches) {
        const cv::Point2f& pt1 = kp1[m.queryIdx].pt;
        const cv::Point2f& pt2 = kp2[m.trainIdx].pt;

        // Transform pt1 through homography
        cv::Mat pt1_h = (cv::Mat_<float>(3,1) << pt1.x, pt1.y, 1.0);
        cv::Mat pt2_proj_h = H * pt1_h;
        pt2_proj_h /= pt2_proj_h.at<float>(2); // Normalize

        cv::Point2f residual(
            pt2.x - pt2_proj_h.at<float>(0),
            pt2.y - pt2_proj_h.at<float>(1)
        );
        residuals.push_back(residual);
    }

    // Step 2: Compute mean and covariance matrix
    cv::Mat residuals_mat(residuals.size(), 2, CV_32F);
    for (size_t i = 0; i < residuals.size(); ++i) {
        residuals_mat.at<float>(i, 0) = residuals[i].x;
        residuals_mat.at<float>(i, 1) = residuals[i].y;
    }

    cv::Mat mean, cov;
    cv::calcCovarMatrix(residuals_mat, cov, mean,
                       cv::COVAR_NORMAL | cv::COVAR_ROWS | cv::COVAR_SCALE);
    cov /= residuals_mat.rows - 1; // Correct for sample covariance

    // Step 3: Regularize covariance matrix if needed
    float det = cv::determinant(cov);
    if (det < 1e-6) { // Near-singular covariance
        cov += cv::Mat::eye(cov.size(), cov.type()) * 1e-6;
    }

    // Step 4: Compute Mahalanobis distance for all matches
    cv::Mat inv_cov = cov.inv();
    inv_cov.convertTo(inv_cov, CV_32FC1);

    for (const auto& m : matches) {
        const cv::Point2f& pt1 = kp1[m.queryIdx].pt;
        const cv::Point2f& pt2 = kp2[m.trainIdx].pt;

        // Project pt1 through homography
        cv::Mat pt1_h = (cv::Mat_<float>(3,1) << pt1.x, pt1.y, 1.0);
        cv::Mat pt2_proj_h = H * pt1_h;
        pt2_proj_h /= pt2_proj_h.at<float>(2);

        // Compute residual
        cv::Mat residual = (cv::Mat_<float>(1,2) <<
            pt2.x - pt2_proj_h.at<float>(0),
            pt2.y - pt2_proj_h.at<float>(1));

        // Subtract mean
        residual -= mean;

        // Compute Mahalanobis distance
        std::cout <<residual.type()<<"\n";
        std::cout <<inv_cov.type()<<"\n";
        cv::Mat dist_sq_mat = residual * inv_cov * residual.t();
        float dist = dist_sq_mat.at<float>(0, 0);

        dist = std::sqrt(dist);

        if (dist <= chi2_threshold) {
            clean_matches.push_back(m);
        }

    }

    return clean_matches;
}


    std::vector<cv::DMatch> adj_calculator::clean_matches(const struct maths::keypoints &kp1,const struct maths::keypoints &kp2,std::vector<cv::DMatch> match,const  cv::Matx33f &H,const std::vector<float> &T){

        std::vector<cv::DMatch> clean_match;
        std::vector<cv::Vec2f> true_;
        std::vector<cv::Vec2f> pred_;

        for (int i = 0;i < match.size();i++){

            true_.push_back(kp1.keypoint[match[i].queryIdx ].pt);
            pred_.push_back(kp2.keypoint[match[i].trainIdx ].pt);
        }

        cv::perspectiveTransform(pred_, pred_, H);

        std::vector<cv::Vec2f> val;
        cv::absdiff(true_,pred_,val);

        int out = 0;

        for (int i = 0; i < val.size();i++){

            if (( val[i][0] < T[0] ) and ( val[i][1] < T[1] )){

                clean_match.push_back(match[i]);

            }

        }

        return clean_match;
    }
