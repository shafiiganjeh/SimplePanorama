#include "_maths.h"

namespace maths {


#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>



Eigen::MatrixXd approximate(
    Eigen::MatrixXd& H,
    Eigen::MatrixXd& K,
    std::vector<cv::KeyPoint>& keypoints1,
    std::vector<cv::KeyPoint>& keypoints2,
    std::vector<cv::DMatch> &matches,
    Eigen::MatrixXd R_i)
{
    // Validate inputs
    assert(H.rows() == 3 && H.cols() == 3);
    assert(K.rows() == 3 && K.cols() == 3);
    assert(!matches.empty());
    assert(keypoints1.size() >= matches.size());
    assert(keypoints2.size() >= matches.size());

    // 1. Compute initial rotation via SVD decomposition
    Eigen::MatrixXd M = K.inverse() * H * K;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd Vt = svd.matrixV().transpose();

    // Ensure proper rotation matrix
    if ((U * Vt).determinant() < 0)
        U.col(2) *= -1;

    Eigen::MatrixXd R_initial = (U * Vt).transpose() * R_i;

    // 2. Generate rotation hypotheses
    std::vector<Eigen::MatrixXd> candidates = {
        R_initial,                                  // Original estimate
        R_initial * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix(),  // X-180
        R_initial * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix(),  // Y-180
        R_initial * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix()   // Z-180
    };

    // 3. Compute reprojection errors
    Eigen::MatrixXd best_R = R_initial;
    double min_error = std::numeric_limits<double>::max();

    // Precompute normalized coordinates
    std::vector<Eigen::Vector3d> norm_points1, norm_points2;
    for (cv::DMatch& m : matches) {
        cv::Point2f& p1 = keypoints1[m.queryIdx].pt;
        cv::Point2f& p2 = keypoints2[m.trainIdx].pt;
        norm_points1.emplace_back(K.inverse() * Eigen::Vector3d(p1.x, p1.y, 1.0));
        norm_points2.emplace_back(K.inverse() * Eigen::Vector3d(p2.x, p2.y, 1.0));
    }

    // Evaluate each candidate
    for (Eigen::MatrixXd& R_candidate : candidates) {
        double total_error = 0.0;
        size_t valid_count = 0;

        for (size_t i = 0; i < matches.size(); ++i) {
            // Transform point from camera 1 to camera 2's coordinates
            Eigen::Vector3d p2_est = R_candidate * norm_points1[i];

            // Check cheirality constraint (positive depth)
            if (p2_est.z() <= 0 || norm_points2[i].z() <= 0)
                continue;

            // Calculate angular error
            Eigen::Vector3d dir_est = p2_est.normalized();
            Eigen::Vector3d dir_actual = norm_points2[i].normalized();
            double err = 1.0 - dir_est.dot(dir_actual);

            total_error += err;
            valid_count++;
        }

        // Use average error if valid points exist
        if (valid_count > 0) {
            const double avg_error = total_error / valid_count;
            if (avg_error < min_error) {
                min_error = avg_error;
                best_R = R_candidate;
            }
        }
    }

    return best_R;
}


std::pair<double, double> computeOverlapPercentages(
    const cv::Mat& imgA,
    const cv::Mat& imgB,
    Eigen::MatrixXd& H)
{

    // Convert H to Eigen::Matrix3d
    Eigen::Matrix3d H_eigen;
    H_eigen << H(0,0), H(0,1), H(0,2),
               H(1,0), H(1,1), H(1,2),
               H(2,0), H(2,1), H(2,2);

    // Get inverse homography for B->A transformation
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

    // Define image boundaries as polygons
    auto createImagePolygon = [](float width, float height)
    {
        return std::vector<cv::Point2f>{
            {0, 0},
            {width, 0},
            {width, height},
            {0, height}
        };
    };

    // Calculate overlap percentage for one direction
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

            kp.keypoint[i].pt.x = kp.keypoint[i].pt.x - (img.rows / 2);
            kp.keypoint[i].pt.y = kp.keypoint[i].pt.y - (img.cols / 2);

        }

        return kp;
    }


std::pair<std::vector<cv::DMatch>, std::vector<cv::DMatch>> match_keypoints(const struct keypoints &kp1,const struct keypoints &kp2){

        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

        std::vector<std::vector<cv::DMatch>> knn_matches;
        std::vector< std::vector<cv::DMatch> > knn_matches2;
        std::vector<cv::DMatch> filter_matches;

        matcher->knnMatch( kp1.descriptor, kp2.descriptor, knn_matches, 2 );

        const float ratio_thresh = 0.7f;
        std::pair<std::vector<cv::DMatch>, std::vector<cv::DMatch>> good_matches;

        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
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


cv::Vec2f hom2eucl_point_2D(const cv::Vec3f& p){
        if(p[3] == 0){
            throw std::invalid_argument("cannot convert.");
        }

        cv::Vec2f v;
        v[0]=p[0]/p[3];
        v[1]=p[1]/p[3];

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


bool isTransformationConsistent(const cv::Matx33f& H, int wide_source, int height_source,
                                int wide_target, int height_target, float margin_ratio = 0.5f){

    const std::vector<cv::Point2f> target_corners = {
        cv::Point2f(0, 0),                      // Top-left
        cv::Point2f(wide_target - 1, 0),        // Top-right
        cv::Point2f(wide_target - 1, height_target - 1), // Bottom-right
        cv::Point2f(0, height_target - 1)       // Bottom-left
    };

    // Transform all corners using homography
    std::vector<cv::Point2f> source_corners;
    cv::perspectiveTransform(target_corners, source_corners, H);

    // Calculate margin boundaries
    const float margin_x = margin_ratio * wide_source;
    const float margin_y = margin_ratio * height_source;

    // Check validity of all transformed points
    for (const auto& p : source_corners) {
        // Check for numerical validity
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            return false;
        }

        // Check bounds with margin
        const bool x_ok = (p.x >= -margin_x) && (p.x <= wide_source + margin_x);
        const bool y_ok = (p.y >= -margin_y) && (p.y <= height_source + margin_y);

        if (!x_ok || !y_ok) {
            return false;
        }
    }

    return true;
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

            if(hom_sanity(H_temp,img1,img2)){
            //if(1){

                double temp_loss = homography_loss(kp1,kp2,match ,H_temp);

                if (temp_loss < loss){
                    //std::cout << loss <<"\n";
                    loss = temp_loss;

                    Hom.H = H_temp;

                }
            }

        }
        return Hom;
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



std::vector<maths::keypoints> extrace_kp_vector(const std::vector<cv::Mat> & imgs,std::vector<int> idx){

    std::vector<maths::keypoints> kp;
    for(const int& s : idx) {

        kp.push_back(maths::extract_keypoints(imgs[s]));

    }

    return kp;
}



struct translation get_translation(const cv::Mat &base, const cv::Mat &attach,const cv::Matx33f &H){

            std::vector<cv::Vec2f> cor;
            translation Tr;

            cor.push_back(cv::Vec2f(0,0));
            cor.push_back(cv::Vec2f(0,attach.rows));
            cor.push_back(cv::Vec2f(attach.cols,0));
            cor.push_back(cv::Vec2f(attach.cols,attach.rows));

            cv::perspectiveTransform(cor, cor, H);

            Tr.xstart = std::min( std::min( cor[0][0], cor[1][0]), (float)0);
            Tr.xend   = std::max( std::max( cor[2][0], cor[3][0]), (float)base.cols);
            Tr.ystart = std::min( std::min( cor[0][1], cor[2][1]), (float)0);
            Tr.yend   = std::max( std::max( cor[1][1], cor[3][1]), (float)base.rows);

            // create translation matrix
            cv::Matx33f T = cv::Matx33f::zeros();
            T(0, 0) = 1;
            T(1, 1) = 1;
            T(2, 2) = 1;
            T(0, 2) = -Tr.xstart;
            T(1, 2) = -Tr.ystart;

            Tr.T = T;


            return Tr;
}

std::map<int, std::pair<int,double>> path_table(const cv::Mat& adj,const std::vector<std::pair<int, std::vector<int>>> &nodes,int start){
    int f_node = start;

    cv::Mat path_lenght = cv::Mat::ones(adj.rows, adj.cols, CV_64F);
    path_lenght = path_lenght - path_lenght.mul(adj);

    std::map<int, std::pair<int,double>> table;
    std::unordered_set<int> visited;
    std::unordered_set<int> to_visit;

    std::vector<int> search;

    for (std::pair<int, std::vector<int>> n : nodes){

        table[n.first] = std::pair<int,double>(-1,10000);
        search.push_back(n.first);

    }

    table[f_node] = std::pair<int,double>(-1,0);
    //visited.insert(f_node);

    while (visited.size() < nodes.size()){
        visited.insert(f_node);

        auto it = find(search.begin(), search.end(), f_node);
        int index = it - search.begin();
        //std::cout << index <<"\n";

        for (const int &n : nodes[index].second){

            if (0 == visited.count(n)){

                double path_val = adj.at<double>(n,nodes[index].first) + table[f_node].second;
                if(path_val < table[n].second){table[n] = std::pair<int,double>(nodes[index].first,path_val);}
                to_visit.insert(n);

            }

        }

        double sm = 10000;
        int next;
        for (const int &n : to_visit){
            double path_val = table[n].second;

            if(path_val < sm){

                sm = path_val;
                next = n;

            }

        }

        f_node = next;
        if(to_visit.size() > 0){
            to_visit.erase(to_visit.find(next));
        }

    }

    return table;
}



std::vector<std::pair<int, std::vector<int>>> bfs_ordered_with_neighbors(const cv::Mat& adj, int i) {

    std::vector<std::pair<int, std::vector<int>>> result;
    int n = adj.cols;
    if (n == 0 || i < 0 || i >= n) {
        return result;
    }

    std::vector<bool> visited(n, false);
    std::queue<int> q;

    visited[i] = true;
    q.push(i);
    std::vector<int> traversal_order;

    // BFS
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        traversal_order.push_back(u);

        for (int v = 0; v < n; ++v) {
            if (adj.at<double>(u,v) > 0 && !visited[v]) {
                visited[v] = true;
                q.push(v);
            }
        }
    }

    // Collect neighbors for each node in BFS
    for (int u : traversal_order) {
        std::vector<int> neighbors;
        for (int v = 0; v < n; ++v) {
            if (adj.at<double>(u,v) > 0) {
                neighbors.push_back(v);
            }
        }
        result.emplace_back(u, neighbors);
    }

    return result;
}


std::vector<int> dfs(cv::Mat& graph, int source) {
    int m = graph.rows;
    std::vector<bool> visited(m, false);
    std::stack<int> s;
    s.push(source);
    std::vector<int> connected_vertices;

    while (!s.empty()) {
        int vertex = s.top();
        s.pop();

        if (!visited[vertex]) {
            visited[vertex] = true;
            connected_vertices.push_back(vertex);

            for (int i = m - 1; i >= 0; --i) {
                if (graph.at<double>(vertex,i) && !visited[i]) {
                    s.push(i);
                }
            }
        }
    }

    return connected_vertices;
}



float focal_from_hom(const std::vector<std::vector< cv::Matx33f >> & H_mat,const cv::Mat &adj){

    float d1, d2;
    float v1, v2;

    float f1,f0;
    std::vector<float> all_focals;

    bool f1_ok,f0_ok;


    for (int i = 0;i < H_mat.size();i++){

        for (int j = i;j < H_mat.size();j++){

            if ((0 < adj.at<double>(i,j)) and (i != j) ){
                const cv::Matx33f& H = H_mat[i][j];

                f1_ok = true;
                d1 = H(2,0) * H(2,1);
                d2 = (H(2,1) - H(2,0)) * (H(2,1) + H(2,0));

                v1 = -(H(0,0) * H(0,1) + H(1,0) * H(1,1)) / d1;
                v2 = (H(0,0) * H(0,0) + H(1,0) * H(1,0) - H(0,1) * H(0,1) - H(1,1) * H(1,1)) / d2;

                if (v1 < v2) std::swap(v1, v2);
                if (v1 > 0 && v2 > 0) f1 = std::sqrt(std::abs(d1) > std::abs(d2) ? v1 : v2);
                else if (v1 > 0) f1 = std::sqrt(v1);
                else f1_ok = false;


                f0_ok = true;
                d1 = H(0,0) * H(1,0) + H(0,1) * H(1,1);
                d2 = H(0,0) * H(0,0) + H(1,0) * H(1,0) - H(0,1) * H(0,1) - H(1,1) * H(1,1);

                v1 = -H(0,2) * H(1,2) / d1;
                v2 = (H(1,2) * H(1,2) - H(0,2) * H(0,2)) / d2;

                if (v1 < v2) std::swap(v1, v2);
                if (v1 > 0 && v2 > 0) f0 = std::sqrt(std::abs(d1) > std::abs(d2) ? v1 : v2);
                else if (v1 > 0) f0 = std::sqrt(v1);
                else f0_ok = false;

                if (f0_ok && f1_ok){
                    all_focals.push_back(std::sqrt(f0 * f1));
                std::cout<< "focal : "<<std::sqrt(f0 * f1)<<"\n";}
                else{
                    std::cout << "something went wrong on image-par: "<<i<<" and "<<j<<"\n";
                }
            }
        }
    }

    float sum = std::accumulate(all_focals.begin(), all_focals.end(), 0.0);
    float mean = sum / all_focals.size();
    if (mean != mean){
        return 8000;
    }

    return mean;
}


    void adj_calculator::get_threads(int n){

            int size = adj.rows;
            std::vector<std::vector<int>> calcs;

            for (int i = 0;i < size;i++){

                for (int j = i;j < size;j++){

                    calcs.push_back({i,j});

                }
            }

            TR = maths::splitVector(calcs, n);

        }


    adj_calculator::adj_calculator(const std::vector<cv::Mat> & imgs,const std::vector<maths::keypoints> &key_p){

            adj.create(imgs.size(), imgs.size(), CV_64F);
            adj = cv::Mat::zeros(imgs.size(), imgs.size(), CV_64F);

            kpmat.resize(key_p.size());
            kpmat = key_p;

            hom_mat.resize(imgs.size(), std::vector<cv::Matx33f>(imgs.size()));

            match_mat.resize(imgs.size(), std::vector<std::vector<cv::DMatch>>(imgs.size()));

        }


    void adj_calculator::cal_adj(const std::vector<cv::Mat> & imgs,int T){

            for(const std::vector<int> & i : TR[T]){

                if (i[0] == i[1]){

                    adj.at<double>(i[0],i[1]) = 0;

                }

                else{
                    double q = match_quality(kpmat[i[0]],imgs[i[0]],kpmat[i[1]],imgs[i[1]],i[0],i[1]);

                    if(q > 0){

                        adj.at<double>(i[0],i[1]) = q;

                    }else{adj.at<double>(i[0],i[1]) = 0;}

                }

            }

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
        std::vector<double> dist;


        cv::absdiff(true_,pred_,val);


        for (int i = 0; i < val.size();i++){

            if (( val[i][0] < T[0] ) and ( val[i][1] < T[1] )){

                clean_match.push_back(match[i]);
                dist.push_back(cv::norm(true_[i]-pred_[i]));

            }

        }

        std::vector<cv::DMatch> rank_match;
        std::vector<size_t> rank = find_n_smallest_indices(dist, 100);

        for(const int &r : rank){

            rank_match.push_back(clean_match[r]);

        }

        return rank_match;
    }



    float adj_calculator::match_quality(const struct maths::keypoints &kp1,const cv::Mat img1,const struct maths::keypoints &kp2,const cv::Mat img2,int row,int col){

            std::vector<cv::DMatch> ransac_matchf;
            std::vector<cv::DMatch> ransac_matchs;
            float rows = img1.rows;
            float cols = img1.cols;

            std::vector<float> T12 = {5,5};

            std::pair<std::vector<cv::DMatch>, std::vector<cv::DMatch>> match12 = maths::match_keypoints(kp1,kp2);

            if (match12.first.size() < 15){

                return 0;
            }

            std::vector<cv::Point2f> obj;
            std::vector<cv::Point2f> scene;

/*

            struct maths::Homography H12 = maths::find_homography(kp1,kp2,match12.first,2000,4,img1,img2);
            H12.H = H12.H / H12.H(2,2);
            //std::cout <<"homography: "<<H12.H<<"\n";
*/
            struct maths::Homography H12;
            std::vector<cv::Point2f> points1, points2;
            for (int i = 0; i < match12.first.size(); i++)
                {
                        //-- Get the keypoints from the good matches
                    points1.push_back(kp1.keypoint[match12.first[i].queryIdx].pt);
                    points2.push_back(kp2.keypoint[match12.first[i].trainIdx].pt);
                }

            try{
                H12.H = findHomography(cv::Mat(points2), cv::Mat(points1), cv::RANSAC);
            }catch(...){

                return 0;

            }

            //std::cout<<"homog: "<<H12.H;

            hom_mat[row][col] = H12.H;
            hom_mat[col][row] = H12.H.inv();

            int out = maths::N_outliers(kp1,kp2,match12.first,H12.H,T12);
            int n_in = match12.first.size() - out;


            if(n_in > ( 8 + .3 * match12.first.size())){

                ransac_matchf = clean_matches(kp1,kp2,match12.first,H12.H,T12);
                //ransac_matchs = clean_matches(kp2,kp1,match12.second,H12.H.inv(),T12);

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


                float orat = 1-out/((float)match12.first.size());


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


        bool compareDescending(const struct adj_str& a, const struct adj_str& b) {
            return a.nodes > b.nodes;
        }

        std::vector<struct adj_str> extract_adj(const cv::Mat &source_adj){

            cv::Mat adj;
            source_adj.copyTo(adj);
            adj = adj + adj.t();
            cv::Mat zeromat = cv::Mat::zeros(1,adj.cols,adj.type());

            std::vector<struct adj_str> adj_mats;

            for(int i = 0; i < adj.rows; i++){

                if(cv::countNonZero(adj.row(i))){
                    //std::cout <<"\n"<<"row : "<<i<<"\n";

                    std::vector<int> graph = dfs(adj, i);
                    cv::Mat sub_graph = cv::Mat::zeros(adj.size(),adj.type());

                    for(int nodes : graph){

                        adj.row(nodes).copyTo(sub_graph.row(nodes));
                        zeromat.row(0).copyTo(adj.row(nodes));
                    }

                    struct adj_str temp;
                    temp.adj = sub_graph;
                    temp.nodes = graph.size();
                    adj_mats.push_back(temp);

                }

            }

            std::sort(adj_mats.begin(), adj_mats.end(), compareDescending);

            return adj_mats;

        }



}
