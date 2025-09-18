
#include "_homography.h"
#include "_img_manipulation.h"

namespace util {


    struct Homography find_homography(const struct keypoints &kp1,const struct keypoints &kp2,const std::vector<cv::DMatch> &match,int max_iter,int sample,const cv::Mat &img1,const cv::Mat &img2,double error_margin){

        struct Homography Hom;
        cv::Matx33f H(1, 0, 0,
                      0, 1, 0,
                      0, 0, 1);
        Hom.H = H;

        if (match.size() < 8){
               std::invalid_argument("non matching images");
        }

        double loss = homography_loss(kp1,kp2,match ,Hom.H ,error_margin);

        for (int i = 0;i < max_iter;i++){

            std::vector<cv::Vec3f> img;
            std::vector<cv::Vec3f> train;

            std::vector<int> samples = randomN(sample, match.size());

            for(const int& s : samples) {

                cv::Vec2f imgkp(kp1.keypoint[match[s].queryIdx ].pt);
                cv::Vec2f trainkp(kp2.keypoint[match[s].trainIdx ].pt );

                img.push_back(eucl2hom_point_2D(imgkp));
                train.push_back(eucl2hom_point_2D(trainkp));

            }

            cv::Matx33f T_b = Normalize2D(img);
            Hom.norm_b = T_b;
            cv::Matx33f T_a = Normalize2D(train);
            Hom.norm_a = T_a;

            img = applyH_2D(img,T_b,GEOM_TYPE_POINT);
            train = applyH_2D(train,T_a,GEOM_TYPE_POINT);

            cv::Mat_<float> DES = getDesignMatrix_homography2D(img, train);
            cv::Matx33f H_temp = solve_homography2D(DES);
            H_temp = decondition_homography2D(T_b, T_a, H_temp);


            double temp_loss = homography_loss(kp1,kp2,match ,H_temp,error_margin);

            if (temp_loss < loss){
                //std::cout << loss <<"\n";
                loss = temp_loss;

                Hom.H = H_temp;

            }

        }
        return Hom;
    }


    cv::Matx33f decondition_homography2D(const cv::Matx33f &T_base, const cv::Matx33f &T_attach, const cv::Matx33f &H){
        cv::Matx33f DC;
        DC = T_base.inv()*H;
        DC = DC*T_attach;

        return DC;
    }


    cv::Vec3f eucl2hom_point_2D(const cv::Vec2f& p){
        cv::Vec3f v;
        v[0]=p[0];
        v[1]=p[1];
        v[2]=1;

        return v;
    }


    cv::Vec2f hom2eucl_point_2D(const cv::Vec3f& p){
            if(p[3] == 0){
                throw std::invalid_argument("cannot convert.");
            }

            cv::Vec2f v;
            v[0]=p[0]/p[3];
            v[1]=p[1]/p[3];

            return v;
    }


    std::vector<cv::Vec3f> applyH_2D(const std::vector<cv::Vec3f>& geomObjects, const cv::Matx33f &H, GeometryType type){
        std::vector<cv::Vec3f> result;

        switch (type) {
            case GEOM_TYPE_POINT: {

            for (unsigned i = 0; i < geomObjects.size(); i++) {
                result.push_back({
                    H.val[0]*geomObjects[i][0]+H.val[1]*geomObjects[i][1]+H.val[2]*geomObjects[i][2],
                    H.val[3]*geomObjects[i][0]+H.val[4]*geomObjects[i][1]+H.val[5]*geomObjects[i][2],
                    H.val[6]*geomObjects[i][0]+H.val[7]*geomObjects[i][1]+H.val[8]*geomObjects[i][2]
                });

            }


            } break;
            case GEOM_TYPE_LINE: {

            cv::Matx33f HT;
            HT=H.inv().t();

            for (unsigned i = 0; i < geomObjects.size(); i++) {
                result.push_back({
                    HT.val[0]*geomObjects[i][0]+HT.val[1]*geomObjects[i][1]+HT.val[2]*geomObjects[i][2],
                    HT.val[3]*geomObjects[i][0]+HT.val[4]*geomObjects[i][1]+HT.val[5]*geomObjects[i][2],
                    HT.val[6]*geomObjects[i][0]+HT.val[7]*geomObjects[i][1]+HT.val[8]*geomObjects[i][2]
                });

            }


            } break;
            default:
                throw std::runtime_error("Unhandled geometry type!");
        }

    return result;
    }


    cv::Matx33f Normalize2D(const std::vector<cv::Vec3f> &points){
        cv::Vec3f TR(0,0,0);
        cv::Vec3f SC(0,0,0);

        float s;
        for (auto i : points){
            TR=TR+i;
        }

        s = points.size();
        TR = TR * (1/s);

        for (auto i : points){
            SC[0]=SC[0]+std::abs(TR[0]-i[0]);
            SC[1]=SC[1]+std::abs(TR[1]-i[1]);
        }
        SC = SC * (1/s);


        cv::Matx33f m(1/SC[0], 0, -TR[0]/SC[0],
                    0, 1/SC[1], -TR[1]/SC[1],
                    0, 0, 1);

        return m;
    }


    cv::Mat_<float> getDesignMatrix_homography2D(const std::vector<cv::Vec3f> &conditioned_base, const std::vector<cv::Vec3f> &conditioned_attach){

    cv::Mat v = cv::Mat_<float>::zeros(conditioned_base.size()*2*9,1);

        for (int i = 0; i < conditioned_base.size(); i++){
                v.at<float>(0+i*18,0) = -conditioned_base[i][2]*conditioned_attach[i][0];
                v.at<float>(1+i*18,0) = -conditioned_base[i][2]*conditioned_attach[i][1];
                v.at<float>(2+i*18,0) = -conditioned_base[i][2]*conditioned_attach[i][2];
                v.at<float>(3+i*18,0) = 0;
                v.at<float>(4+i*18,0) = 0;
                v.at<float>(5+i*18,0) = 0;
                v.at<float>(6+i*18,0) = conditioned_base[i][0]*conditioned_attach[i][0];
                v.at<float>(7+i*18,0) = conditioned_base[i][0]*conditioned_attach[i][1];
                v.at<float>(8+i*18,0) = conditioned_base[i][0]*conditioned_attach[i][2];
                v.at<float>(9+i*18,0) = 0;
                v.at<float>(10+i*18,0) = 0;
                v.at<float>(11+i*18,0) = 0;
                v.at<float>(12+i*18,0) = -conditioned_base[i][2]*conditioned_attach[i][0];
                v.at<float>(13+i*18,0) = -conditioned_base[i][2]*conditioned_attach[i][1];
                v.at<float>(14+i*18,0) = -conditioned_base[i][2]*conditioned_attach[i][2];
                v.at<float>(15+i*18,0) = conditioned_base[i][1]*conditioned_attach[i][0];
                v.at<float>(16+i*18,0) = conditioned_base[i][1]*conditioned_attach[i][1];
                v.at<float>(17+i*18,0) = conditioned_base[i][1]*conditioned_attach[i][2];
        }

        v = v.reshape(1,conditioned_base.size()*2);
        return v;
    }


    cv::Matx33f solve_homography2D(const cv::Mat_<float> &A){
        cv::SVD svd(A, cv::SVD::FULL_UV);
        cv::Mat M(svd.vt);
        M = M.t();
        M = M.col(8);

        cv::Matx33f H(M.at<float>(0), M.at<float>(1), M.at<float>(2),
                    M.at<float>(3), M.at<float>(4), M.at<float>(5),
                    M.at<float>(6), M.at<float>(7), M.at<float>(8));

        return -1*H;
    }


    float N_outliers(const struct keypoints &kp1,const struct keypoints &kp2,std::vector<cv::DMatch> &match,const  cv::Matx33f &H,float margin){

        std::vector<cv::Vec2f> true_;
        std::vector<cv::Vec2f> pred_;

        for (int i = 0;i < match.size();i++){

            true_.push_back(kp1.keypoint[match[i].queryIdx ].pt);
            pred_.push_back(kp2.keypoint[match[i].trainIdx ].pt);
        }

        cv::perspectiveTransform(pred_, pred_, H);

        int out = 0;

        for(int i = 0;i < match.size();i++){

            if (margin < cv::norm(true_[i]-pred_[i])){

                out++;

            }

        }

        return (float)out;
    }


    double homography_loss(const struct keypoints &kp1,const struct keypoints &kp2,const std::vector<cv::DMatch> &match , const cv::Matx33f &H,double error_margin){

        std::vector<cv::Vec2f> true_;
        std::vector<cv::Vec2f> pred_;

        for (int i = 0;i < match.size();i++){

            true_.push_back(kp1.keypoint[match[i].queryIdx ].pt);
            pred_.push_back(kp2.keypoint[match[i].trainIdx ].pt);
        }

        int inlier = 0;
        cv::perspectiveTransform(pred_, pred_, H);
        for (int i = 0;i < match.size();i++){

            inlier = inlier + (int)(cv::norm(true_[i]-pred_[i]) < error_margin);

        }

        return 1.0-((double)inlier/(double)match.size());

    }


    bool hom_sanity(cv::Matx33f H,const cv::Mat &img1,const cv::Mat &img2){

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                float val = H(i,j);
                if (std::isnan(val) || std::isinf(val))
                    return false;
            }
        }

        //reflection
        float det = H(0,0)*H(1,1) - H(0,1)*H(1,0);
        if (det <= 0){
           return false;
        }

        //skew

        if((H(2,0) > .003) or (H(2,1) > .003)){
            return false;
        }

        std::vector<cv::Vec3f> corners_img1 = {
                cv::Vec3f(0, 0, 1),
                cv::Vec3f(img1.cols, 0, 1),
                cv::Vec3f(img1.cols, img1.rows, 1),
                cv::Vec3f(0, img1.rows, 1)
            };

        std::vector<cv::Vec3f> proj_corners_homog = applyH_2D(corners_img1, H, GEOM_TYPE_POINT);
        std::vector<cv::Point2f> proj_corners;

        for (const auto& corner : proj_corners_homog) {
            if (std::abs(corner[2]) < 1e-6){
                return false;
            }
            cv::Point2f p(corner[0]/corner[2], corner[1]/corner[2]);
            proj_corners.push_back(p);
        }
        //is convex
        if (!cv::isContourConvex(proj_corners)){
            return false;
        }
        //is not too large
        float area = cv::contourArea(proj_corners);
        if (area < img1.total() / 200.0f){
            return false;
        }

        // points at "infinity"
        for (const auto& corner : proj_corners) {
            if (std::abs(corner.x) > 8000 * img2.cols or std::abs(corner.y) > 8000 * img2.rows){
                return false;
            }

        }




        return true;
    }


    adj_calculator::adj_calculator(const std::vector<cv::Mat> & imgs,const std::vector<keypoints> &key_p,struct match_conf* conf,std::atomic<double>* fadress,std::atomic<bool>* cancel){

            if(not (conf == NULL)){
              conf_local = *conf;
            }

            adj.create(imgs.size(), imgs.size(), CV_64F);
            adj = cv::Mat::zeros(imgs.size(), imgs.size(), CV_64F);

            f_adress = fadress;
            c_adress = cancel;

            kpmat.resize(key_p.size());
            kpmat = key_p;

            hom_mat.resize(imgs.size(), std::vector<cv::Matx33f>(imgs.size()));

            match_mat.resize(imgs.size(), std::vector<std::vector<cv::DMatch>>(imgs.size()));
            match_mat_raw.resize(imgs.size(), std::vector<std::vector<cv::DMatch>>(imgs.size()));

        }


    std::vector<size_t> adj_calculator::find_n_smallest_indices(const std::vector<double>& rank, int n) {
        std::vector<size_t> indices;
        indices.reserve(rank.size());
        for (size_t i = 0; i < rank.size(); ++i) {
            indices.push_back(i); // Manually fill indices with 0, 1, 2, ..., rank.size()-1
        }

        if (n <= 0) {
            return {};
        }

        if (n >= static_cast<int>(indices.size())) {
            return indices;
        }

        auto comparator = [&rank](size_t a, size_t b) { return rank[a] < rank[b]; };
        std::nth_element(indices.begin(), indices.begin() + n, indices.end(), comparator);

        indices.resize(n);
        return indices;
    }


    void adj_calculator::cal_adj(const std::vector<cv::Mat> & imgs,int T){

            for(const std::vector<int> & i : TR[T]){

                if(not (f_adress == NULL)){

                    f_adress->fetch_add(add/2, std::memory_order_relaxed);

                }

                if (i[0] == i[1]){

                    adj.at<double>(i[0],i[1]) = 0;

                }else{
                    double q;
                    if(c_adress and c_adress->load()){

                        q = 0;

                    }else{

                        q = match_quality(kpmat[i[0]],imgs[i[0]],kpmat[i[1]],imgs[i[1]],i[0],i[1]);
                    }

                    if(q > 0){

                        adj.at<double>(i[0],i[1]) = q;

                    }else{adj.at<double>(i[0],i[1]) = 0;}

                }

            }

        }


    std::pair<std::vector<cv::DMatch>, std::vector<cv::DMatch>> match_keypoints(const struct keypoints &kp1,const struct keypoints &kp2){

        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

        std::vector<std::vector<cv::DMatch>> knn_matches;
        std::vector<cv::DMatch> filter_matches;

        matcher->knnMatch( kp1.descriptor, kp2.descriptor, knn_matches, 2 );

        const float ratio_thresh = 0.8f;
        std::pair<std::vector<cv::DMatch>, std::vector<cv::DMatch>> good_matches;

        for (size_t i = 0; i < knn_matches.size(); i++){

            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance){

                good_matches.first.push_back(knn_matches[i][0]);
                cv::DMatch rev;
                rev.queryIdx = knn_matches[i][0].trainIdx;
                rev.trainIdx = knn_matches[i][0].queryIdx;
                rev.imgIdx = knn_matches[i][0].imgIdx;
                rev.distance = knn_matches[i][0].distance;
                good_matches.second.push_back(rev);
            }
        }

        return good_matches;

    }



void keypoints_in_overlap_centered(const std::vector<cv::Point2f>& matches,
                                                 const std::vector<cv::KeyPoint>& keypoints,
                                                 struct Homography& H,
                                                 const cv::Size& image_size1,
                                                 const cv::Size& image_size2) {
    // Define the centered rectangle of image1
    std::vector<cv::Point2f> image1_rect_centered(4);
    float w1 = image_size1.width;
    float h1 = image_size1.height;
    image1_rect_centered[0] = cv::Point2f(-w1/2, -h1/2);
    image1_rect_centered[1] = cv::Point2f(w1/2, -h1/2);
    image1_rect_centered[2] = cv::Point2f(w1/2, h1/2);
    image1_rect_centered[3] = cv::Point2f(-w1/2, h1/2);

    // Define the centered rectangle of image2
    std::vector<cv::Point2f> image2_rect_centered(4);
    float w2 = image_size2.width;
    float h2 = image_size2.height;
    image2_rect_centered[0] = cv::Point2f(-w2/2, -h2/2);
    image2_rect_centered[1] = cv::Point2f(w2/2, -h2/2);
    image2_rect_centered[2] = cv::Point2f(w2/2, h2/2);
    image2_rect_centered[3] = cv::Point2f(-w2/2, h2/2);

    cv::Matx33f H_centered = H.H;
    std::vector<cv::Point2f> image2_in_image1_centered;
    cv::perspectiveTransform(image2_rect_centered, image2_in_image1_centered, H_centered);

    std::vector<cv::Point2f> overlap_polygon;
    double area = cv::intersectConvexConvex(image1_rect_centered, image2_in_image1_centered, overlap_polygon);

    if (overlap_polygon.size() < 3) {
        return;
    }

    int keypoints_in_overlap_count = 0;
    for (const auto& kp : keypoints) {
        if (cv::pointPolygonTest(overlap_polygon, kp.pt, false) >= 0) {
            keypoints_in_overlap_count++;
        }
    }

    int matches_in_overlap_count = 0;
    for (const auto& match_pt : matches) {
        if (cv::pointPolygonTest(overlap_polygon, match_pt, false) >= 0) {
            matches_in_overlap_count++;
        }
    }

    H.area_kp = keypoints_in_overlap_count;
    H.area_match = matches_in_overlap_count;
    H.overlap_perc = (float)area/(float)(image_size1.width*image_size1.height);

}


    float adj_calculator::match_quality(const struct keypoints &kp1,const cv::Mat img1,const struct keypoints &kp2,const cv::Mat img2,int row,int col){

            std::vector<cv::DMatch> ransac_matchf;
            float rows = img1.rows;
            float cols = img1.cols;

            std::vector<cv::DMatch> match12 = match_mat_raw[row][col];

            if (match12.size() < 30){

                return 0;
            }

            std::vector<cv::Point2f> obj;
            std::vector<cv::Point2f> scene;

            struct Homography H12 = find_homography(kp1,kp2,match12,conf_local.RANSAC_iterations,4,img1,img2,conf_local.x_margin);

            std::vector<cv::Point2f> points1, points2;
            for (int i = 0; i < match12.size(); i++)
                {
                        //-- Get the keypoints from the good matches
                    points1.push_back(kp1.keypoint[match12[i].queryIdx].pt);
                    points2.push_back(kp2.keypoint[match12[i].trainIdx].pt);
                }

            H12.H = H12.H / H12.H(2,2);
            //std::cout <<"homography: "<<H12.H<<"\n";
/*

            struct Homography H12;
            std::vector<cv::Point2f> points1, points2;
            for (int i = 0; i < match12.size(); i++)
                {
                        //-- Get the keypoints from the good matches
                    points1.push_back(kp1.keypoint[match12[i].queryIdx].pt);
                    points2.push_back(kp2.keypoint[match12[i].trainIdx].pt);
                }

            try{
                H12.H = findHomography(cv::Mat(points2), cv::Mat(points1), cv::RANSAC);
            }catch(...){

                return 0;

            }
*/
            //std::cout<<"homog: "<<H12.H;

            if(hom_sanity(H12.H,img1,img2) == false){

                return 0;

            }

            hom_mat[row][col] = H12.H;
            hom_mat[col][row] = H12.H.inv();

            struct Homography H21 = H12;
            H21.H = hom_mat[col][row];

            int out = N_outliers(kp1,kp2,match12,H12.H,conf_local.x_margin);
            int n_in = match12.size() - out;

            keypoints_in_overlap_centered(points1,kp1.keypoint,H12,img1.size(),img2.size());
/*
            std::cout <<"\n"<<"kp in area "<<H12.area_kp<<"\n";
            std::cout <<"kp total "<<kp1.keypoint.size()<<"\n";
            std::cout <<"match in area "<<H12.area_match<<"\n";
            std::cout <<"n_in/match "<<((double)n_in)/((double)H12.area_match)<<"\n";
            std::cout <<"overlap area "<<H12.overlap_perc<<"\n";
            std::cout <<"tot_match "<<match12.size()<<"\n";
            std::cout <<"n_in "<<n_in<<"\n";
*/
            double overlap_inl_match_1 = ((double)n_in)/((double)H12.area_match);
            double overlap_inl_keyp_1 = ((double)n_in)/((double)H12.area_kp);
            float overlap_test = conf_local.min_overlap;
            float overlap_inl_match = conf_local.overlap_inl_match;
            float overlap_inl_keyp = conf_local.overlap_inl_keyp;
            float conf = conf_local.conf;

            //float overlap_inl_match = .05;
            //float overlap_inl_keyp = .005;
            //float conf = .025;

            //inlier match in area test------------img1
            //more imlier than matches in area
            if((overlap_inl_match_1 > 1.00)){

                return 0;

            }

            //overlpa area
            if((overlap_test > H12.overlap_perc)){

                return 0;

            }

            //inlier/ points in area
            if(overlap_inl_match > overlap_inl_match_1){

                return 0;
            }

            //inlier total match test
            if (overlap_inl_keyp_1 < overlap_inl_keyp ){
                return 0;
            }

            //image 2 --------------------
            keypoints_in_overlap_centered(points2,kp2.keypoint,H21,img2.size(),img1.size());
            double overlap_inl_match_2 = ((double)n_in)/((double)H21.area_match);
            double overlap_inl_keyp_2 = ((double)n_in)/((double)H21.area_kp);

            if((overlap_inl_match_2 > 1.00)){

                return 0;
            }

            if((overlap_test > H21.overlap_perc)){

                return 0;
            }

            if(overlap_inl_match > overlap_inl_match_2){

                return 0;
            }

            if (overlap_inl_keyp_2 < overlap_inl_keyp ){

                return 0;
            }

            //confidence check----------
            if ((overlap_inl_keyp_2 + overlap_inl_keyp_2)/2 < conf){
                return 0;
            }


            //std::cout <<"accepted "<<"\n";
            //std::cout <<" "<<"\n";

            ransac_matchf = clean_matches(kp1,kp2,match12,H12.H,conf_local.x_margin);

            match_mat[row][col].resize(ransac_matchf.size());
            match_mat[row][col] = ransac_matchf;

            for(int m = 0;m<match_mat[row][col].size();m++){
                cv::DMatch push;
                push.distance = match_mat[row][col][m].distance;
                push.trainIdx = match_mat[row][col][m].queryIdx;
                push.queryIdx = match_mat[row][col][m].trainIdx;
                push.imgIdx = match_mat[row][col][m].imgIdx;
                match_mat[col][row].push_back(push);
            }

            //float orat = 1-((float)out/((float)match12.size()));

            return H12.overlap_perc;

        }

        //change this
    void adj_calculator::get_match_number_matrix(int T){

            for(const std::vector<int> & i : TR[T]){

                if(not (f_adress == NULL)){
                    f_adress->fetch_add(add/2, std::memory_order_relaxed);
                } //progress bar counbter

                if (i[0] == i[1]){

                    adj.at<double>(i[0],i[1]) = 0;

                }

                else{
                    std::pair<std::vector<cv::DMatch>, std::vector<cv::DMatch>> match;

                    if(c_adress and c_adress->load()){

                        continue;

                    }else{

                        match = match_keypoints(kpmat[i[0]],kpmat[i[1]]);

                        adj.at<double>(i[0],i[1]) = match.first.size();
                        match_mat_raw[i[0]][i[1]] = match.first;

                    }

                }

            }

        }


    cv::Mat convert_to_rootsift(const cv::Mat& descriptors) {
        cv::Mat rootsift_descriptors;

        if (descriptors.empty()) {
            return rootsift_descriptors;
        }

        cv::Mat float_descriptors;
        descriptors.convertTo(float_descriptors, CV_32F);

        for (int i = 0; i < float_descriptors.rows; i++) {
            cv::Mat descriptor = float_descriptors.row(i);
            double norm = cv::norm(descriptor, cv::NORM_L1);

            // Avoid division by zero
            if (norm > 0) {
                descriptor /= norm;
            }

            cv::sqrt(descriptor, descriptor);
        }

        rootsift_descriptors = float_descriptors;
        return rootsift_descriptors;
    }


    struct keypoints extract_keypoints(const cv::Mat &img,int nfeatures,int nOctaveLayers ,double contrastThreshold ,double edgeThreshold ,double sigma ){

        cv::Mat greyMat;
        keypoints kp;

        cv::cvtColor(img, greyMat, cv::COLOR_BGR2GRAY);

        auto f_detector = cv::SIFT::create(nfeatures,
                                        nOctaveLayers,
                                        contrastThreshold ,
                                        edgeThreshold ,
                                        sigma );

        f_detector->detectAndCompute(greyMat,cv::noArray(),kp.keypoint,kp.descriptor);
        kp.descriptor = convert_to_rootsift(kp.descriptor);

        for(int i = 0;i < kp.keypoint.size();i++){

            kp.keypoint[i].pt.x = kp.keypoint[i].pt.x - (img.cols / 2);
            kp.keypoint[i].pt.y = kp.keypoint[i].pt.y - (img.rows / 2);

        }

        return kp;
    }


    keypoints extract_keypoints_detGFTT_descSIFT(const cv::Mat &img,
                                        int maxCorners,
                                        double qualityLevel,
                                        double minDistance,
                                        int blockSize,
                                        bool useHarrisDetector,
                                        double k,
                                        int nFeatures,
                                        int nOctaveLayers,
                                        double contrastThreshold,
                                        double edgeThreshold,
                                        double sigma) {
        cv::Mat greyMat;
        keypoints kp;

        cv::cvtColor(img, greyMat, cv::COLOR_BGR2GRAY);

        // Detect keypoints using GFTT
        std::vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack(greyMat, corners, maxCorners, qualityLevel, minDistance,
                            cv::noArray(), blockSize, useHarrisDetector, k);

        // Convert Point2f to KeyPoint
        cv::KeyPoint::convert(corners, kp.keypoint);

        // Compute descriptors using SIFT
        auto descriptor_extractor = cv::SIFT::create(nFeatures, nOctaveLayers,
                                                contrastThreshold, edgeThreshold, sigma);

        descriptor_extractor->compute(greyMat, kp.keypoint, kp.descriptor);

        // Adjust keypoint coordinates to be relative to image center
        for (auto &kp_point : kp.keypoint) {
            kp_point.pt.x -= (img.cols / 2);
            kp_point.pt.y -= (img.rows / 2);
        }

        return kp;
    }


    std::vector<keypoints> extrace_kp_vector(const std::vector<cv::Mat> & imgs,std::vector<int> idx,match_conf* conf){


        std::vector<keypoints> kp;
        for(const int& s : idx) {
            if(conf == NULL){
                kp.push_back(extract_keypoints(imgs[s]));
                //kp.push_back(extract_keypoints_detGFTT_descSIFT(imgs[s]));
            }else{
                kp.push_back(extract_keypoints(imgs[s],conf->nfeatures,conf->nOctaveLayers,conf->contrastThreshold,conf->edgeThreshold,conf->sigma_sift));
                //kp.push_back(extract_keypoints_detGFTT_descSIFT(imgs[s]));
            }

        }

        return kp;
    }


    void adj_calculator::get_threads(int n){

            int size = adj.rows;
            std::vector<std::vector<int>> calcs;
            double elem_counter = 0;

            for (int i = 0;i < size;i++){

                for (int j = i;j < size;j++){

                    calcs.push_back({i,j});
                    elem_counter++;

                }
            }

            add = (1.0/6.0) * (1.0/elem_counter);

            TR = splitVector(calcs, n);

        }


    void adj_calculator::heuristic_match_filter(int n){

            int size = adj.rows;
            if (n <= 0 or size == 0) {
                throw std::runtime_error("Wrong parameter or empty matches");
            }

            n = (size < n) ? size : n;

            cv::Mat temp = cv::Mat::zeros(adj.size(),adj.type());

            for(int i = 0; i < size; i++) {

                std::vector<std::pair<double, int>> candidates;
                for (int j = i + 1; j < size; j++) {
                    double value = adj.at<double>(i, j);
                    candidates.push_back(std::make_pair(value, j));
                }

                if (candidates.size() > static_cast<size_t>(n)) {
                    std::partial_sort(
                        candidates.begin(),
                        candidates.begin() + n,
                        candidates.end(),
                        [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
                            return a.first > b.first;
                        }
                    );
                    candidates.resize(n);
                }

                for (const auto& cand : candidates) {
                    int j = cand.second;
                    temp.at<double>(i, j) = cand.first;

                }

            }

            adj = temp;

    }


    std::vector<cv::DMatch> adj_calculator::clean_matches(const struct keypoints &kp1,const struct keypoints &kp2,std::vector<cv::DMatch> match,const  cv::Matx33f &H,double margin){

        std::vector<cv::DMatch> clean_match;
        std::vector<cv::Vec2f> true_;
        std::vector<cv::Vec2f> pred_;

        for (int i = 0;i < match.size();i++){

            true_.push_back(kp1.keypoint[match[i].queryIdx ].pt);
            pred_.push_back(kp2.keypoint[match[i].trainIdx ].pt);
        }

        cv::perspectiveTransform(pred_, pred_, H);
        std::vector<double> dist;

        for(int i = 0;i < match.size();i++){

            if (margin >= cv::norm(true_[i]-pred_[i])){

                clean_match.push_back(match[i]);
                dist.push_back(cv::norm(true_[i]-pred_[i]));

            }

        }

        std::vector<cv::DMatch> rank_match;
        std::vector<size_t> rank = find_n_smallest_indices(dist, conf_local.max_keypoints);

        for(const int &r : rank){

            rank_match.push_back(clean_match[r]);

        }

        return rank_match;
    }


}

