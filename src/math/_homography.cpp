
#include "_homography.h"


namespace util {


    struct Homography find_homography(const struct keypoints &kp1,const struct keypoints &kp2,const std::vector<cv::DMatch> &match,int max_iter,int sample,const cv::Mat &img1,const cv::Mat &img2){

        struct Homography Hom;
        cv::Matx33f H(1, 0, 0,
                      0, 1, 0,
                      0, 0, 1);
        Hom.H = H;

        if (match.size() < 8){
               std::invalid_argument("non matching images");
        }

        double loss = homography_loss(kp1,kp2,match ,Hom.H );

        for (int i = 0;i < max_iter;i++){

            std::vector<cv::Vec3f> img;
            std::vector<cv::Vec3f> train;

            std::vector<int> samples = randomN(sample, match.size());

            for(const int& s : samples) {
                //std::cout << s <<"-";
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
            double temp_loss = homography_loss(kp1,kp2,match ,H_temp);

            if (temp_loss < loss){
                //std::cout << loss <<"\n";
                loss = temp_loss;

                Hom.H = H_temp;

            }

            /*
            if(hom_sanity(H_temp,img1,img2)){
            //if(1){

                double temp_loss = homography_loss(kp1,kp2,match ,H_temp);

                if (temp_loss < loss){
                    //std::cout << loss <<"\n";
                    loss = temp_loss;

                    Hom.H = H_temp;

                }
            }
*/
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


    float N_outliers(const struct keypoints &kp1,const struct keypoints &kp2,std::vector<cv::DMatch> &match,const  cv::Matx33f &H,std::vector<float> &T){

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

        for (const cv::Vec2f& v : val){

            if (( v[0] > T[0] )|| ( v[1] > T[1] )){

                out++;

            }

        }

        return (float)out;
    }


    double homography_loss(const struct keypoints &kp1,const struct keypoints &kp2,const std::vector<cv::DMatch> &match , const cv::Matx33f &H ){

        std::vector<cv::Vec2f> true_;
        std::vector<cv::Vec2f> pred_;

        for (int i = 0;i < match.size();i++){

            true_.push_back(kp1.keypoint[match[i].queryIdx ].pt);
            pred_.push_back(kp2.keypoint[match[i].trainIdx ].pt);
        }

        double loss = 0;

        cv::perspectiveTransform(pred_, pred_, H);
        for (int i = 0;i < match.size();i++){
            //loss = loss + cv::norm(true_[i]-pred_[i]);
            loss = loss + cv::norm(true_[i]-pred_[i]);
            //std::cout << true_[i] - pred_[i] <<"-";
        }
        return loss/match.size();
    }


    bool hom_sanity(cv::Matx33f hom,const cv::Mat &img1,const cv::Mat &img2){

        //validate
        if(hom(2,2) < 1e-6 ){

            return false;

        }

        hom = hom / hom(2,2);

        cv::Matx22f upper_square;
        upper_square(0,0) = hom(0,0);
        upper_square(1,0) = hom(1,0);
        upper_square(1,1) = hom(1,1);
        upper_square(0,1) = hom(0,1);

        double upper_det = cv::determinant(upper_square);

        //orientation
        if(upper_det <= 0){
            //std::cout <<"\n"<< "orientation: ";
            return false;

        }

        //area scaling
        if((abs(upper_det) < 0.1 ) or ( abs(upper_det) > 10)){
            //std::cout <<"\n"<< "area scaling: ";
            return false;

        }


        //skew factor
        if((abs(hom(2,0)) > 0.002 ) or ( abs(hom(2,1)) > 0.002)){
            //std::cout <<"\n"<< "skew factor: ";
            return false;

        }


        return true;
    }


    std::pair<double, double> computeOverlapPercentages(
    const cv::Mat& imgA,
    const cv::Mat& imgB,
    Eigen::MatrixXd& H){

        Eigen::Matrix3d H_eigen;
        H_eigen << H(0,0), H(0,1), H(0,2),
                H(1,0), H(1,1), H(1,2),
                H(2,0), H(2,1), H(2,2);

        Eigen::Matrix3d H_inv = H_eigen.inverse();

        // Helper function to transform points
        auto transformPoints = [](const std::vector<cv::Point2f>& points,
                                const Eigen::Matrix3d& homography)
        {
            std::vector<cv::Point2f> transformed;
            for (const auto& p : points) {
                Eigen::Vector3d hp(p.x, p.y, 1.0);
                Eigen::Vector3d result = homography * hp;
                transformed.emplace_back(
                    result.x() / result.z(),
                    result.y() / result.z()
                );
            }
            return transformed;
        };


        auto createImagePolygon = [](float width, float height)
        {
            return std::vector<cv::Point2f>{
                {0, 0},
                {width, 0},
                {width, height},
                {0, height}
            };
        };


        auto calculateOverlap = [](const std::vector<cv::Point2f>& poly1,
                                const std::vector<cv::Point2f>& poly2,
                                double imageArea)
        {
            std::vector<cv::Point2f> intersection;
            float area = cv::intersectConvexConvex(poly1, poly2, intersection, true);
            return (area / imageArea) * 100.0;
        };

        // Image A's perspective (transform B to A's space)
        std::vector<cv::Point2f> cornersB = createImagePolygon(imgB.cols, imgB.rows);
        auto transformedB = transformPoints(cornersB, H_inv);
        double overlapA = calculateOverlap(
            transformedB,
            createImagePolygon(imgA.cols, imgA.rows),
            imgA.cols * imgA.rows
        );

        // Image B's perspective (transform A to B's space)
        std::vector<cv::Point2f> cornersA = createImagePolygon(imgA.cols, imgA.rows);
        auto transformedA = transformPoints(cornersA, H_eigen);
        double overlapB = calculateOverlap(
            transformedA,
            createImagePolygon(imgB.cols, imgB.rows),
            imgB.cols * imgB.rows
        );

        return {overlapA, overlapB};
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

        const float ratio_thresh = 0.7f;
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


    float adj_calculator::match_quality(const struct keypoints &kp1,const cv::Mat img1,const struct keypoints &kp2,const cv::Mat img2,int row,int col){

            std::vector<cv::DMatch> ransac_matchf;
            std::vector<cv::DMatch> ransac_matchs;
            float rows = img1.rows;
            float cols = img1.cols;

            std::vector<float> T12 = {(float)conf_local.x_margin,(float)conf_local.y_margin};
            std::vector<cv::DMatch> match12 = match_mat_raw[row][col];

            if (match12.size() < 25){

                return 0;
            }

            std::vector<cv::Point2f> obj;
            std::vector<cv::Point2f> scene;

/*
            struct Homography H12 = find_homography(kp1,kp2,match12,conf_local.RANSAC_iterations,4,img1,img2);

            if(not hom_sanity(H_temp,img1,img2)){

                struct Homography Hom;
                cv::Matx33f H(1, 0, 0,
                            0, 1, 0,
                            0, 0, 1);
                H12.H = H;

            }

            H12.H = H12.H / H12.H(2,2);
            //std::cout <<"homography: "<<H12.H<<"\n";
*/
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

            //std::cout<<"homog: "<<H12.H;

            hom_mat[row][col] = H12.H;
            hom_mat[col][row] = H12.H.inv();

            int out = N_outliers(kp1,kp2,match12,H12.H,T12);
            int n_in = match12.size() - out;


            if(n_in > ( 8 + .3 * match12.size())){

                ransac_matchf = clean_matches(kp1,kp2,match12,H12.H,T12);

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


                float orat = 1-out/((float)match12.size());


                cv::Mat h_d = cv::Mat(H12.H,true);

                Eigen::MatrixXd H;

                cv::cv2eigen(h_d,H);
                std::pair<double, double> overlap = computeOverlapPercentages(
                img1,
                img2,
                H);

                return (float)(overlap.second/100);
                //return orat;

            }else{

                return 0;

            }

            return 0;
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
        for(int i = 0;i < kp.keypoint.size();i++){

            kp.keypoint[i].pt.x = kp.keypoint[i].pt.x - (img.cols / 2);
            kp.keypoint[i].pt.y = kp.keypoint[i].pt.y - (img.rows / 2);

        }

        return kp;
    }


    std::vector<keypoints> extrace_kp_vector(const std::vector<cv::Mat> & imgs,std::vector<int> idx,match_conf* conf){


        std::vector<keypoints> kp;
        for(const int& s : idx) {
            if(conf == NULL){
                kp.push_back(extract_keypoints(imgs[s]));
            }else{
                kp.push_back(extract_keypoints(imgs[s],conf->nfeatures,conf->nOctaveLayers,conf->contrastThreshold,conf->edgeThreshold,conf->sigma_sift));
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


    std::vector<cv::DMatch> adj_calculator::clean_matches(const struct keypoints &kp1,const struct keypoints &kp2,std::vector<cv::DMatch> match,const  cv::Matx33f &H,const std::vector<float> &T){

        std::vector<cv::DMatch> clean_match;
        std::vector<cv::Vec2f> true_;
        std::vector<cv::Vec2f> pred_;

        for (int i = 0;i < match.size();i++){

            true_.push_back(kp1.keypoint[match[i].queryIdx ].pt);
            pred_.push_back(kp2.keypoint[match[i].trainIdx ].pt);
        }

        cv::perspectiveTransform(pred_, pred_, H);

        std::vector<cv::Vec2f> val;
        std::vector<double> dist;


        cv::absdiff(true_,pred_,val);


        for (int i = 0; i < val.size();i++){

            if (( val[i][0] < T[0] ) and ( val[i][1] < T[1] )){

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

