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


    std::pair<std::vector<cv::DMatch>, std::vector<cv::DMatch>> match_keypoints(const struct keypoints &kp1,const struct keypoints &kp2){

        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        std::vector< std::vector<cv::DMatch> > knn_matches;
        std::vector<cv::DMatch> filter_matches;

        matcher->knnMatch( kp1.descriptor, kp2.descriptor, knn_matches, 2 );

        const float ratio_thresh = 0.7f;
        std::pair<std::vector<cv::DMatch>, std::vector<cv::DMatch>> good_matches;

        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.first.push_back(knn_matches[i][0]);
                good_matches.second.push_back(knn_matches[i][1]);
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


    bool trcheck = isTransformationConsistent(hom, img1.cols, img1.rows,img2.cols, img2.rows, .8);


    return trcheck;
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



template <typename T>
std::vector<std::vector<T>> splitVector(const std::vector<T>& vec, int n) {
        std::vector<std::vector<T>> result;

        // Return an empty set of vectors if n is non-positive or vector is empty
        if (n <= 0 || vec.empty()) {
            return result;
        }

        if (n > vec.size()) {

            throw std::invalid_argument("n should be less than vector size.");
        }

        // Calculate the size of each part
        int basic_part_size = vec.size() / n;
        int remainder = vec.size() % n;

        // Start index for slicing
        int start_index = 0;

        for (int i = 0; i < n; ++i) {
            int current_part_size = basic_part_size + (i < remainder ? 1 : 0);  // Add 1 if i is less than remainder
            std::vector<T> part(vec.begin() + start_index, vec.begin() + start_index + current_part_size);
            result.push_back(part);
            start_index += current_part_size;
        }

        return result;
}
void _(){
    int n = 1;
    std::vector<int> idx;
    std::vector<std::vector<int>> split_id = splitVector(idx, n);
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

            if ((.5 <= adj.at<double>(i,j)) and (i != j) ){
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

                    if(q >= .5){

                        adj.at<double>(i[0],i[1]) = q;

                    }else{adj.at<double>(i[0],i[1]) = 0;}

                }

            }

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
        cv::Mat dist_sq_mat = residual * inv_cov * residual.t();
        float dist = dist_sq_mat.at<float>(0, 0);

        dist = std::sqrt(dist);

        if (dist <= chi2_threshold) {
            clean_matches.push_back(m);
        }

    }

    return clean_matches;
}


    float adj_calculator::match_quality(const struct maths::keypoints &kp1,const cv::Mat img1,const struct maths::keypoints &kp2,const cv::Mat img2,int row,int col){

            std::vector<cv::DMatch> ransac_matchf;
            std::vector<cv::DMatch> ransac_matchs;
            float rows = img1.rows;
            float cols = img1.cols;

            std::vector<float> T12 = {cols / 300,rows / 300};

            std::pair<std::vector<cv::DMatch>, std::vector<cv::DMatch>> match12 = maths::match_keypoints(kp1,kp2);

            if (match12.first.size() < 25){

                return 0;
            }

            std::vector<cv::Point2f> obj;
            std::vector<cv::Point2f> scene;

            struct maths::Homography H12 = maths::find_homography(kp1,kp2,match12.first,3000,4,img1,img2);
            H12.H = H12.H / H12.H(2,2);
            //std::cout <<"homography: "<<H12.H<<"\n";
/*

            struct maths::Homography H12;
            std::vector<cv::Point2f> points1, points2;
            for (int i = 0; i < match12.first.size(); i++)
                {
                        //-- Get the keypoints from the good matches

                    points1.push_back(kp1.keypoint[match12.first[i].queryIdx].pt);

                    points2.push_back(kp2.keypoint[match12.first[i].trainIdx].pt);
                }

            H12.H = findHomography(cv::Mat(points2), cv::Mat(points1), cv::RANSAC);
            std::cout<<"homog: "<<H12.H;
*/
            hom_mat[row][col] = H12.H;
            hom_mat[col][row] = H12.H.inv();

            int out = maths::N_outliers(kp1,kp2,match12.first,H12.H,T12);
            int n_in = match12.first.size() - out;

            if(n_in > ( 8 + .3 * match12.first.size())){


                ransac_matchf = clean_matches(kp1,kp2,match12.first,H12.H,T12);
                ransac_matchs = clean_matches(kp2,kp1,match12.second,H12.H.inv(),T12);

                match_mat[row][col].resize(ransac_matchf.size());
                match_mat[row][col] = ransac_matchf;
                match_mat[col][row].resize(ransac_matchs.size());
                match_mat[col][row] = ransac_matchs;
                std::cout<<"\n"<<"-------------in: "<<ransac_matchf.size()<<"\n";

                std::cout<<"\n"<<"out: "<<out<<"\n";
                float orat = 1-out/((float)match12.first.size());
                if((.4 < orat) and ( orat < .5)){
                    orat = .50;
                }

                return orat;


            }else{
                std::cout<<"\n"<<"in: "<<n_in<<"\n";
                std::cout<<"\n"<<"------------out: "<<out<<"\n";
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

                    //std::cout <<"\n"<<"newadj : "<<adj<<"\n";
                    //std::cout <<"\n"<<"adjextracted : "<<sub_graph<<"\n";
                    struct adj_str temp;
                    temp.adj = sub_graph;
                    temp.nodes = graph.size();
                    adj_mats.push_back(temp);

                    //std::cout <<"grap "<<sub_graph<<"\n";
                    //std::cout <<"nodes "<<graph.size()<<"\n";

                }

            }

            std::sort(adj_mats.begin(), adj_mats.end(), compareDescending);

            return adj_mats;

        }



}
