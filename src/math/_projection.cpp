
#include "_projection.h"

namespace proj {


struct kartesian Polar2Kartesian(struct polar & p){
    struct kartesian k;

    k.x = p.r * cos(p.phi);
    k.y = p.r * sin(p.phi);

    return k;
}


struct polar Kartesian2Polar(struct kartesian & k){
    struct polar p;

    p.r = sqrt(k.x*k.x+k.y*k.y);
    p.phi = atan2(k.y,k.x);

    return p;
}


struct warped spherical_proj::project(const Eigen::MatrixXd & R, const Eigen::MatrixXd & K, const cv::Mat & img) const{

    struct warped SD;

    const int w_ref = img.cols;
    const int h_ref = img.rows;
    cv::Mat cvK, cvR;
    Eigen::Matrix3d R_adj = R;

    const double f_i = K(0, 0);
    const double s = focal / f_i;
    const double c_x = K(0, 2);
    const double c_y = K(1, 2);

    Eigen::Matrix3d K_adj;
    K_adj << f_i, 0,w_ref-c_x,
            0, f_i,h_ref-c_y,
            0, 0, 1;

    cv::eigen2cv(K_adj, cvK);
    cv::eigen2cv(R_adj, cvR);
    cvK.convertTo(cvK, CV_32F);
    cvR.convertTo(cvR, CV_32F);

    SD.corners = warper->warp(img, cvK, cvR, cv::INTER_LINEAR, cv::BORDER_CONSTANT, SD.imgs);

    return SD;
}


struct warped cylindrical_proj::project(const Eigen::MatrixXd & R, const Eigen::MatrixXd & K, const cv::Mat & img) const{

    struct warped SD;

    const int w_ref = img.cols;
    const int h_ref = img.rows;
    cv::Mat cvK, cvR;
    Eigen::Matrix3d R_adj = R;

    const double f_i = K(0, 0);
    const double s = focal / f_i;
    const double c_x = K(0, 2);
    const double c_y = K(1, 2);

    Eigen::Matrix3d K_adj;
    K_adj << f_i, 0,w_ref-c_x,
            0, f_i,h_ref-c_y,
            0, 0, 1;

    cv::eigen2cv(K_adj, cvK);
    cv::eigen2cv(R_adj, cvR);
    cvK.convertTo(cvK, CV_32F);
    cvR.convertTo(cvR, CV_32F);

    SD.corners = warper->warp(img, cvK, cvR, cv::INTER_LINEAR, cv::BORDER_CONSTANT, SD.imgs);

    return SD;
}


std::vector<cv::Point> create_border(const cv::Rect& R, int N) {
    std::vector<cv::Point> points;

    int width = R.width;
    int height = R.height;
    int perimeter = 2 * (width + height);

    float pointsPerUnit = (float)N / perimeter;

    int topPoints = (int)(width * pointsPerUnit);
    int rightPoints = (int)(height * pointsPerUnit);
    int bottomPoints = topPoints;
    int leftPoints = rightPoints;

    int x = R.x, y = R.y;

    // Top side
    float step = (float)width / (topPoints + 1);
    for (int i = 1; i <= topPoints; i++) {
        points.push_back(cv::Point(x + (int)(i * step), y));
    }

    // Right side
    step = (float)height / (rightPoints + 1);
    for (int i = 1; i <= rightPoints; i++) {
        points.push_back(cv::Point(x + width, y + (int)(i * step)));
    }


    step = (float)(width) / (bottomPoints + 1);
    for (int i = 1; i <= bottomPoints; i++) {
        points.push_back(cv::Point(x + width - (int)(i * step), y + height));
    }


    step = (float)(height) / (leftPoints + 1);
    for (int i = 1; i <= leftPoints; i++) {
        points.push_back(cv::Point(x, y + height - (int)(i * step)));
    }


    return points;
}


struct bounding_data sten_proj::get_bounding_box(struct proj_data &proj,util::RadialNormalizer & norm,bool quadratic){

    struct bounding_data ret;

    for(int i = 0;i < proj.corners.size();i++){

        std::vector<cv::Point> img_c(4);
        img_c[0] = cv::Point(proj.corners[i].x ,proj.corners[i].y);
        img_c[1] = cv::Point(proj.corners[i].x + proj.imgs[i].cols,proj.corners[i].y);
        img_c[2] = cv::Point(proj.corners[i].x + proj.imgs[i].cols,proj.corners[i].y + proj.imgs[i].rows);
        img_c[3] = cv::Point(proj.corners[i].x,proj.corners[i].y + proj.imgs[i].rows);

        cv::Rect bbox = cv::boundingRect(img_c);
        bbox.x = proj.corners[i].x;
        bbox.y = proj.corners[i].y;
        //std::cout<<"\n bbox "<<bbox;

        ret.org_bbox.push_back(bbox);

        std::vector<cv::Point> border = create_border(bbox, precision);
        std::vector<cv::Point2f> border_n = norm.normalize(border);

        for(int p = 0; p < border_n.size() ; p++){

            struct kartesian K;
            K.x = border_n[p].x;
            K.y = border_n[p].y;
            struct polar P = Kartesian2Polar(K);

            float exp;
            if(quadratic){
                exp = P.r*P.r;
            }else{
                exp = P.r;
            }
            if(exp > radius){
                P.r = (exp - radius)/(1-radius);
            }else{

                P.r = P.r;

            }

            K = Polar2Kartesian(P);
            cv::Point2f ps(K.x,K.y);
            border_n[p] = ps;

        }

        border = norm.denormalize(border_n);
        cv::Rect bbox_tr = cv::boundingRect(border);
        //std::cout<<"\n bbox_tr "<<bbox_tr;
        ret.tr_bbox.push_back(bbox_tr);

    }

    return ret;

}


void sten_proj::disk_reproj(struct proj_data &proj,bool quadratic){

    util::RadialNormalizer norm;
    struct util::size_data sd = util::get_pan_dimension(proj.corners,proj.imgs);
    std::vector<cv::Point> norm_points;

    ansatz.x = ansatz.x -(int)(sd.dims.width/2+1);
    ansatz.y = ansatz.y -(int)(sd.dims.height/2+1);

    for(int i = 0;i < proj.corners.size();i++){

        proj.corners[i] = proj.corners[i] - cv::Point(sd.min_x+(int)(sd.dims.width/2+1),sd.min_y+(int)(sd.dims.height/2+1));
        norm_points.push_back(proj.corners[i]);
        norm_points.push_back(cv::Point(proj.corners[i].x + proj.imgs[i].cols,proj.corners[i].y));
        norm_points.push_back(cv::Point(proj.corners[i].x + proj.imgs[i].cols,proj.corners[i].y + proj.imgs[i].rows));
        norm_points.push_back(cv::Point(proj.corners[i].x,proj.corners[i].y + proj.imgs[i].rows));

    }

    norm.computeParameters(ansatz,norm_points);

    cv::Point r_p = cv::Point(ansatz.x ,ansatz.y+ (int)radius);

    cv::Point2f r_pf = norm.normalizePoint(r_p);

    radius = sqrt(r_pf.x*r_pf.x + r_pf.y*r_pf.y);
    struct bounding_data b_dat = sten_proj::get_bounding_box(proj,norm,quadratic);

    for(int i = 0;i < proj.corners.size();i++){

        proj.corners[i].x = b_dat.tr_bbox[i].x;
        proj.corners[i].y = b_dat.tr_bbox[i].y;

        cv::Mat map_x(b_dat.tr_bbox[i].size(), CV_32FC1);
        cv::Mat map_y(b_dat.tr_bbox[i].size(), CV_32FC1);

        for (int y = 0; y < b_dat.tr_bbox[i].height; y++) {
            for (int x = 0; x < b_dat.tr_bbox[i].width; x++) {

                cv::Point point(x+ (int)b_dat.tr_bbox[i].x,y+ (int)b_dat.tr_bbox[i].y);
                cv::Point2f PN = norm.normalizePoint(point);

                struct kartesian K;

                K.x = PN.x;
                K.y = PN.y;

                struct polar P = Kartesian2Polar(K);

                float exp;
                int sub;
                if(quadratic){
                    exp = P.r*P.r;
                    sub = 2;
                }else{
                    exp = P.r;
                    sub = 1;
                }

                P.r = exp*(sub-radius) + radius;

                K = Polar2Kartesian(P);

                PN.x = K.x;
                PN.y = K.y;

                point = norm.denormalizePoint(PN);

                point.x = point.x - b_dat.org_bbox[i].x;
                point.y = point.y - b_dat.org_bbox[i].y;

                map_x.at<float>(y, x) = point.x;
                map_y.at<float>(y, x) = point.y;

            }

        }
/*
        cv::Mat test;
        cv::resize(proj.imgs[i],test,cv::Size(800,800));
        cv::imshow("Display window", test);
        cv::waitKey(0);
        std::cout<<"\n orgsize "<<proj.imgs[i].size();
        std::cout<<"\n orgbbsize "<<b_dat.org_bbox[i].size();
*/
        cv::remap(proj.imgs[i], proj.imgs[i], map_x, map_y, cv::INTER_CUBIC, cv::BORDER_CONSTANT);
/*
        cv::resize(proj.imgs[i],test,cv::Size(800,800));
        cv::imshow("Display window", test);
        cv::waitKey(0);

        std::cout<<"\n trsize "<<proj.imgs[i].size();
        std::cout<<"\n trbbsize "<<b_dat.tr_bbox[i].size();
*/
        cv::Mat masks_temp = blnd::createSurroundingMask(proj.imgs[i], true, 1);
        cv::erode(masks_temp, masks_temp, cv::Mat(), cv::Point(-1, -1), 3);
        proj.msks[i] = masks_temp;

    }


}


struct warped sten_proj::project(const Eigen::MatrixXd & R, const Eigen::MatrixXd & K, const cv::Mat & img) const{

    struct warped SD;

    const int w_ref = img.cols;
    const int h_ref = img.rows;
    cv::Mat cvK, cvR;
    Eigen::Matrix3d R_adj = R;

    const double f_i = K(0, 0);
    const double s = focal / f_i;
    const double c_x = K(0, 2);
    const double c_y = K(1, 2);

    Eigen::Matrix3d K_adj;
    K_adj << f_i, 0,w_ref-c_x,
            0, f_i,h_ref-c_y,
            0, 0, 1;

    cv::eigen2cv(K_adj, cvK);
    cv::eigen2cv(R_adj, cvR);
    cvK.convertTo(cvK, CV_32F);
    cvR.convertTo(cvR, CV_32F);

    SD.corners = warper->warp(img, cvK, cvR, cv::INTER_LINEAR, cv::BORDER_CONSTANT, SD.imgs);

    return SD;
}


void sten_proj::inpaint(cv::Mat & img,const cv::Point& ansatz,float radius){

        cv::Point R;

        R.x = ansatz.x - (int)(img.cols/8) ;
        R.y = ansatz.y - (int)(img.rows/8) ;
        cv::Rect roi(R.x, R.y, (int)(img.cols/4), (int)(img.rows/4));

        int intensity_threshold = 1;
        cv::Mat roiImage = img(roi);
        cv::Mat roiGray;
        cv::cvtColor(roiImage, roiGray, cv::COLOR_BGR2GRAY);
        cv::Mat roiMask;
        cv::threshold(roiGray, roiMask, intensity_threshold, 255, cv::THRESH_BINARY_INV);

        cv::Mat maskr = cv::Mat::zeros(img.size(), CV_8UC1);
        roiMask.copyTo(maskr(roi));

        /*test area
        for(int y=0;y<img(roi).rows;y++)
        {
            for(int x=0;x<img(roi).cols;x++)
            {
                img(roi).at<cv::Vec3b>(cv::Point(x,y)) = {0,0,0};
            }

        }
        */

        cv::inpaint(img, maskr, img, 3, cv::INPAINT_NS);

}


std::pair<cv::Point,float> sten_proj::estimate_circle(struct proj_data &proj){

    struct util::size_data dim = util::get_pan_dimension(proj.corners,proj.imgs);
    std::pair<cv::Point,float> result;

    cv::Mat test_mask(
        dim.dims.height,
        dim.dims.width,
        CV_8UC1,
        cv::Scalar(0)
    );

    for (int i = 0; i < proj.imgs.size(); i++) {

        cv::Rect roi(
            proj.corners[i].x - dim.min_x,  // x offset
            proj.corners[i].y - dim.min_y,  // y offset
            proj.imgs[i].size().width,
            proj.imgs[i].size().height
        );

        proj.msks[i].copyTo(test_mask(roi),proj.msks[i]);

    }

    struct util::ComponentResult c_res = util::analyzeComponentsWithCircles(test_mask, 100);

    float dist = sqrt(test_mask.cols*test_mask.cols+test_mask.rows*test_mask.rows);
    int best;
    float cutoff = (dist/2)*.5;
    float cutoff_dist = (dist/2)*.2;

    for(int j = 0;j < c_res.circles.size();j++){

        if(dist > c_res.circles[j].distanceFromCenter){

            dist = c_res.circles[j].distanceFromCenter;
            best = j;

        }
    }

    if((dist > cutoff_dist) or (c_res.circles[best].radius > cutoff)){

        std::cout <<"\n No midsection found. ";
        result.second = -1.0;

    }else{

        ansatz = c_res.circles[best].center;
        radius = c_res.circles[best].radius+3; //add little offset
        result.first = ansatz;
        result.second = radius;
        has_circle = true;

    }

    return result;
}


struct proj_data get_proj_parameters(
    const std::vector<cv::Mat>& images,
    std::vector<Eigen::MatrixXd>& R,
    std::vector<Eigen::MatrixXd>& K,
    std::vector<double> &con,
    std::shared_ptr<projection> projector,
    bool get_masks){

    struct proj_data par_stitch;

    for (int i = 0; i < images.size(); i++) {
        if(con[i] > 0){

            struct warped wrp = projector->project(R[i],K[i], images[i]);

            par_stitch.imgs.push_back(wrp.imgs);
            par_stitch.corners.push_back(wrp.corners);
            if(get_masks){

                cv::Mat masks_temp = blnd::createSurroundingMask(wrp.imgs, true, 1);
                cv::erode(masks_temp, masks_temp, cv::Mat(), cv::Point(-1, -1), 3);
                par_stitch.msks.push_back(masks_temp);

            }

        }

    }

    struct util::size_data p_size = util::get_pan_dimension(par_stitch.corners,par_stitch.imgs);

    return par_stitch;
}


}
