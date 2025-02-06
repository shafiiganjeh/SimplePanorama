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

float graph_thread::match_quality(const struct keypoints &kp1,const cv::Mat img1,const struct keypoints &kp2,const cv::Mat img2,int row,int col){

        std::vector<float> T12 = {(float)img1.cols/350,(float)img1.rows/350};

        std::vector<cv::DMatch> match12 = match_keypoints(kp1,kp2);

        if (match12.size() < 16){

            return 0;
        }

        std::vector<cv::Point2f> obj;
        std::vector<cv::Point2f> scene;

        cv::Matx33f H12 = find_homography(kp1,kp2,match12,1000,4);
        hom_mat[row][col] = H12;
        hom_mat[col][row] = H12.inv();

        int out = N_outliers(kp1,kp2,match12,H12,T12);

        return 1-out/((float)match12.size());

}


std::vector<maths::keypoints> extrace_kp_vector(const std::vector<cv::Mat> & imgs,std::vector<int> idx){

    std::vector<maths::keypoints> kp;
    for(const int& s : idx) {

        kp.push_back(maths::extract_keypoints(imgs[s]));

    }

    return kp;
}


void graph_thread::set_mat(const std::vector<cv::Mat> & imgs,std::vector<maths::keypoints> key_p){

        adj = cv::Mat::zeros(imgs.size(), imgs.size(), CV_64F);
        kpmat = key_p;
        hom_mat.resize(imgs.size(), std::vector<cv::Matx33f>(imgs.size()));

}

cv::Mat graph_thread::return_adj_mat(){

            return adj + adj.t();

}

std::vector<std::vector< cv::Matx33f >> graph_thread::return_Hom_mat(){

    return hom_mat;

}


void graph_thread::cal_adj(const std::vector<cv::Mat> & imgs,const std::vector<std::vector<int>> idx){

    for(const std::vector<int> & i : idx){

        if (i[0] == i[1]){

            adj.at<double>(i[0],i[1]) = 0;

        }else{
            double q = match_quality(kpmat[i[0]],imgs[i[0]],kpmat[i[1]],imgs[i[1]],i[0],i[1]);
            if(q >= .5){

                adj.at<double>(i[0],i[1]) = q;

            }else{adj.at<double>(i[0],i[1]) = 0;}

        }

    }

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
    int n;
    std::vector<int> idx;
    std::vector<std::vector<int>> split_id = splitVector(idx, n);
}

thread graph_thread::get_threads(int n){

        int size = adj.rows;
        std::vector<std::vector<int>> calcs;

        for (int i = 0;i < size;i++){

            for (int j = i;j < size;j++){

                calcs.push_back({i,j});

            }
        }

        thread TR = splitVector(calcs, n);
        return TR;

}


cv::Matx33f get_translation(const cv::Mat &base, const cv::Mat &attach,const cv::Matx33f &H){

            std::vector<cv::Vec2f> cor;

            cor.push_back(cv::Vec2f(0,0));
            cor.push_back(cv::Vec2f(0,attach.rows));
            cor.push_back(cv::Vec2f(attach.cols,0));
            cor.push_back(cv::Vec2f(attach.cols,attach.rows));

            cv::perspectiveTransform(cor, cor, H);

            float xstart = std::min( std::min( cor[0][0], cor[1][0]), (float)0);
            float xend   = std::max( std::max( cor[2][0], cor[3][0]), (float)base.cols);
            float ystart = std::min( std::min( cor[0][1], cor[2][1]), (float)0);
            float yend   = std::max( std::max( cor[1][1], cor[3][1]), (float)base.rows);

            // create translation matrix
            cv::Matx33f T = cv::Matx33f::zeros();
            T(0, 0) = 1;
            T(1, 1) = 1;
            T(2, 2) = 1;
            T(0, 2) = -xstart;
            T(1, 2) = -ystart;

            return T;
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


}
