#include "_maths.h"

namespace maths {

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

        return kp;
    }


    std::vector<cv::DMatch> match_keypoints(const struct keypoints &kp1,const struct keypoints &kp2){

        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        std::vector< std::vector<cv::DMatch> > knn_matches;
        std::vector<cv::DMatch> filter_matches;

        matcher->knnMatch( kp1.descriptor, kp2.descriptor, knn_matches, 2 );

        const float ratio_thresh = 0.7f;
        std::vector<cv::DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        return good_matches;

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


cv::Vec3f eucl2hom_point_2D(const cv::Vec2f& p){
        cv::Vec3f v;
        v[0]=p[0];
        v[1]=p[1];
        v[2]=1;

        return v;
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


cv::Matx33f decondition_homography2D(const cv::Matx33f &T_base, const cv::Matx33f &T_attach, const cv::Matx33f &H){
        cv::Matx33f DC;
        DC = T_base.inv()*H;
        DC = DC*T_attach;

        return DC;
}


std::vector<int> randomN(int n, int m){

        std::random_device rd;
        std::mt19937 g(rd());

        std::vector<int> numbers(m) ;
        std::iota (std::begin(numbers), std::end(numbers), 0);

        std::vector<int> result;
        if (n < m) {
            std::sample(numbers.begin(), numbers.end(), std::back_inserter(result), n, g);
        } else {
            throw std::invalid_argument("n should be less than m");
        }

        return result;
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


cv::Matx33f find_homography(const struct keypoints &kp1,const struct keypoints &kp2,const std::vector<cv::DMatch> &match,int max_iter,int sample){


        cv::Matx33f H(1, 0, 0,
                      0, 1, 0,
                      0, 0, 1);


        if (match.size() < 8){
               std::invalid_argument("non matching images");
        }

        double loss = homography_loss(kp1,kp2,match ,H );

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
            cv::Matx33f T_a = Normalize2D(train);

            img = applyH_2D(img,T_b,GEOM_TYPE_POINT);
            train = applyH_2D(train,T_a,GEOM_TYPE_POINT);

            cv::Mat_<float> DES = getDesignMatrix_homography2D(img, train);

            cv::Matx33f H_temp = solve_homography2D(DES);

            H_temp = decondition_homography2D(T_b, T_a, H_temp);

            double temp_loss = homography_loss(kp1,kp2,match ,H_temp);

            if (temp_loss < loss){
                //std::cout << loss <<"\n";
                loss = temp_loss;

                H = H_temp;

            }

        }
        return H;
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

float match_quality(const struct keypoints &kp1,const cv::Mat img1,const struct keypoints &kp2,const cv::Mat img2){

        std::vector<float> T12 = {(float)img1.cols/400,(float)img1.rows/400};

        std::vector<cv::DMatch> match12 = match_keypoints(kp1,kp2);

        if (match12.size() < 16){

            return 0;
        }

        std::vector<cv::Point2f> obj;
        std::vector<cv::Point2f> scene;

        cv::Matx33f H12 = find_homography(kp1,kp2,match12,100,4);

        int out = N_outliers(kp1,kp2,match12,H12,T12);

        return 1-out/((float)match12.size());

}



}
