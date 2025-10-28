
#include "_util.h"


namespace util {


struct ComponentResult analyzeComponentsWithCircles(const cv::Mat& image, float minArea) {

    cv::Mat mask;
    if(image.type() == CV_8UC1){

        mask = cv::Mat::ones(image.size(), image.type()) * 255 - image;

    }else{

        cv::inRange(image, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0), mask);

    }


    if (cv::countNonZero(mask) == 0) {
        throw std::runtime_error("No connected components found: mask is entirely zero");
    }

    cv::Point2f imageCenter(image.cols / 2.0f, image.rows / 2.0f);

    cv::Mat labels, stats, centroids;
    int numComponents = cv::connectedComponentsWithStats(mask, labels, stats, centroids);

    if (numComponents <= 1) {
        throw std::runtime_error("No connected components found");
    }

    ComponentResult result;
    result.imageCenter = imageCenter;

    for (int i = 1; i < numComponents; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);

        if (area >= minArea) {

            cv::Mat componentMask = cv::Mat::zeros(mask.size(), CV_8UC1);
            componentMask.setTo(255, labels == i);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(componentMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (!contours.empty()) {
                CircleInfo circleInfo;
                circleInfo.componentArea = area;
                circleInfo.boundingBox = cv::Rect(
                    stats.at<int>(i, cv::CC_STAT_LEFT),
                    stats.at<int>(i, cv::CC_STAT_TOP),
                    stats.at<int>(i, cv::CC_STAT_WIDTH),
                    stats.at<int>(i, cv::CC_STAT_HEIGHT)
                );

                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(contours[0], center, radius);
                circleInfo.center = center;
                circleInfo.radius = radius;

                float dx = center.x - imageCenter.x;
                float dy = center.y - imageCenter.y;
                circleInfo.distanceFromCenter = std::sqrt(dx * dx + dy * dy);

                circleInfo.size = CV_PI * radius * radius;

                result.circles.push_back(circleInfo);
            }
        }
    }

    if (result.circles.empty()) {
        throw std::runtime_error("No components found with area >= " + std::to_string(minArea));
    }

    return result;
}


std::string doubleToString(double value, int precision) {

    std::ostringstream oss;

    oss.imbue(std::locale::classic());
    oss << std::fixed << std::setprecision(precision) << value;

    if (oss.fail()) {
        throw std::runtime_error("Double to string conversion failed");
    }

    return oss.str();
}


std::string floatToString(float value, int precision) {

    std::ostringstream oss;

    oss.imbue(std::locale::classic());
    oss << std::fixed << std::setprecision(precision) << value;

    if (oss.fail()) {
        throw std::runtime_error("Float to string conversion failed");
    }

    return oss.str();
}


double stringToDouble(const std::string& str) {
    std::istringstream totalSString(str);
    double valueAsDouble;
    // maybe use some manipulators
    totalSString >> valueAsDouble;
    if(!totalSString)
        throw std::runtime_error("Error converting to double");
    return valueAsDouble;
}


float stringToFloat(const std::string& str) {
    std::istringstream totalSString(str);
    float valueAsFloat;
    // maybe use some manipulators
    totalSString >> valueAsFloat;
    if(!totalSString)
        throw std::runtime_error("Error converting to float");
    return valueAsFloat;
}


int stringToInt(const std::string& str) {
    std::istringstream totalSString(str);
    int valueAsInt;
    // maybe use some manipulators
    totalSString >> valueAsInt;
    if(!totalSString)
        throw std::runtime_error("Error converting to int");
    return valueAsInt;
}


val processValue(double value, int max_val) {
    int rounded_up = static_cast<int>(std::ceil(value));
    int capped_int = (rounded_up > max_val) ? max_val : rounded_up;

    double capped_double = (value > max_val) ? max_val : value;

    return {capped_int, capped_double};
}


cv::Rect scaleRect(const cv::Rect& r, double xs, double sy) {

    int x = (int)std::round(r.x * xs);
    int y = (int)std::round(r.y * sy);

    int width = (int)std::round(r.width * xs);
    int height = (int)std::round(r.height * sy);

    return cv::Rect(x, y, width, height);
}


std::pair<float,float> get_rot_dif(std::vector<Eigen::MatrixXd> &Crret){

    float max_w = 0;
    float min_w = 0;
    float max_h = 0;
    float min_h = 0;

    for(int i = 0;i<Crret.size();i++){


        float x = std::atan2(Crret[i](2,1),Crret[i](2,2)); //h
        float y = std::atan2(-Crret[i](2,0),sqrt(Crret[i](2,1)*Crret[i](2,1) + Crret[i](2,2)*Crret[i](2,2))); //w

        if(min_w > y){
           min_w = y;
        }

        if(max_w < y){
           max_w = y;
        }

        if(min_h > x){
           min_h = x;
        }

        if(max_h < x){
           max_h = x;
        }

    }

    return {abs(max_h) + abs(min_h),abs(max_w) + abs(min_w)};
}


struct size_data get_pan_dimension(const std::vector<cv::Point>& top_lefts,const std::vector<cv::Mat>& images){

    struct size_data p_size;
    int min_x = INT_MAX, min_y = INT_MAX;
    int max_x = INT_MIN, max_y = INT_MIN;

    for (size_t i = 0; i < images.size(); i++) {
        cv::Point tl = top_lefts[i];
        cv::Point br(tl.x + images[i].cols, tl.y + images[i].rows);

        min_x = std::min(min_x, tl.x);
        min_y = std::min(min_y, tl.y);
        max_x = std::max(max_x, br.x);
        max_y = std::max(max_y, br.y);
    }

    int width = max_x - min_x;
    int height = max_y - min_y;

    p_size.dims.width = max_x - min_x;
    p_size.dims.height = max_y - min_y;
    p_size.min_x = min_x;
    p_size.min_y = min_y;
    p_size.max_x = max_x;
    p_size.max_y = max_y;

    return p_size;
}


std::vector<double> computeRowSumDividedByZeroCount(const cv::Mat& mat) {

    const int rows = mat.rows;
    const int cols = mat.cols;
    std::vector<double> results;
    results.reserve(rows);

    for (int i = 0; i < rows; ++i) {
        const cv::Mat row = mat.row(i);
        const double row_sum = cv::sum(row)[0];
        const int zero_count = cols - cv::countNonZero(row);
        results.push_back(row_sum / zero_count);
    }

    return results;
}

//https://stackoverflow.com/questions/28287138/c-randomly-sample-k-numbers-from-range-0n-1-n-k-without-replacement
std::vector<int> randomN(int n, int m){

        std::random_device rd;
        std::mt19937 gen(rd());

        std::unordered_set<int> elems;
        for (int r = m - n; r < m; ++r) {
            int v = std::uniform_int_distribution<>(0, r)(gen);

            if (!elems.insert(v).second) {
                elems.insert(r);
            }
        }

        std::vector<int> result(elems.begin(), elems.end());

        return result;
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




struct translation get_translation(const cv::Size &base, const cv::Mat &attach,const cv::Matx33f &H){

            std::vector<cv::Vec2f> cor;
            translation Tr;

            cor.push_back(cv::Vec2f(0,0));
            cor.push_back(cv::Vec2f(0,attach.rows));
            cor.push_back(cv::Vec2f(attach.cols,0));
            cor.push_back(cv::Vec2f(attach.cols,attach.rows));

            cv::perspectiveTransform(cor, cor, H);

            Tr.xstart = std::min( std::min( cor[0][0], cor[1][0]), (float)0);
            Tr.xend   = std::max( std::max( cor[2][0], cor[3][0]), (float)base.width);
            Tr.ystart = std::min( std::min( cor[0][1], cor[2][1]), (float)0);
            Tr.yend   = std::max( std::max( cor[1][1], cor[3][1]), (float)base.height);

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
                if (v1 > 0 and v2 > 0) f1 = std::sqrt(std::abs(d1) > std::abs(d2) ? v1 : v2);
                else if (v1 > 0) f1 = std::sqrt(v1);
                else f1_ok = false;


                f0_ok = true;
                d1 = H(0,0) * H(1,0) + H(0,1) * H(1,1);
                d2 = H(0,0) * H(0,0) + H(1,0) * H(1,0) - H(0,1) * H(0,1) - H(1,1) * H(1,1);

                v1 = -H(0,2) * H(1,2) / d1;
                v2 = (H(1,2) * H(1,2) - H(0,2) * H(0,2)) / d2;

                if (v1 < v2) std::swap(v1, v2);
                if (v1 > 0 and v2 > 0) f0 = std::sqrt(std::abs(d1) > std::abs(d2) ? v1 : v2);
                else if (v1 > 0) f0 = std::sqrt(v1);
                else f0_ok = false;

                if (f0_ok and f1_ok){
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
    if ((mean != mean) or (mean < 300)){
        return -1;
    }

    return mean;
}



    bool compareDescending(const struct adj_str& a, const struct adj_str& b) {
            return a.nodes > b.nodes;
        }

    std::vector<struct adj_str> extract_adj(const cv::Mat &source_adj){


        if (source_adj.empty()) {
            throw std::invalid_argument("Input matrix is empty");
        }

        if (source_adj.rows != source_adj.cols) {
            throw std::invalid_argument("Input matrix is not square");
        }

        cv::Mat adj;
        source_adj.copyTo(adj);
        adj = adj + adj.t();
        cv::Mat zeromat = cv::Mat::zeros(1, adj.cols, adj.type());

        std::vector<adj_str> adj_mats;

        for (int i = 0; i < adj.rows; i++) {
            if (cv::countNonZero(adj.row(i))) {
                std::vector<int> graph = dfs(adj, i);
                cv::Mat sub_graph = cv::Mat::zeros(adj.size(), adj.type());

                for (int node : graph) {
                    adj.row(node).copyTo(sub_graph.row(node));
                    zeromat.row(0).copyTo(adj.row(node));
                }

                adj_str temp;
                temp.connectivity = computeRowSumDividedByZeroCount(sub_graph);

                for (int j = 1; j < sub_graph.rows; j++) {
                    sub_graph.row(j).colRange(0, j).setTo(0);
                }

                temp.adj = sub_graph;
                temp.nodes = graph.size();
                adj_mats.push_back(temp);
            }
        }


        if (adj_mats.empty()) {
            throw std::runtime_error("No connected subgraphs found");
        }

        std::sort(adj_mats.begin(), adj_mats.end(), compareDescending);

        return adj_mats;

}


void RadialNormalizer::computeParameters(const cv::Point& initialPoint, const std::vector<cv::Point>& points) {
    center = cv::Point2f(static_cast<float>(initialPoint.x), static_cast<float>(initialPoint.y));

    if (points.empty()) {
        scale = 1.0f;
        return;
    }

    // Find the maximum Euclidean distance from the initial point
    float maxDistance = 0.0f;

    for (const auto& point : points) {
        float dx = static_cast<float>(point.x) - center.x;
        float dy = static_cast<float>(point.y) - center.y;
        float distance = std::sqrt(dx * dx + dy * dy);

        if (distance > maxDistance) {
            maxDistance = distance;
        }
    }

    // If all points are at the center, use scale 1 to avoid division by zero
    scale = (maxDistance == 0.0f) ? 1.0f : 1.0f / maxDistance;
}

std::vector<cv::Point2f> RadialNormalizer::normalize(const std::vector<cv::Point>& points) {
    std::vector<cv::Point2f> normalizedPoints;
    normalizedPoints.reserve(points.size());

    for (const auto& p : points) {
        normalizedPoints.push_back(normalizePoint(p));
    }

    return normalizedPoints;
}

std::vector<cv::Point> RadialNormalizer::denormalize(const std::vector<cv::Point2f>& normalizedPoints) {
    std::vector<cv::Point> originalPoints;
    originalPoints.reserve(normalizedPoints.size());

    for (const auto& p : normalizedPoints) {
        originalPoints.push_back(denormalizePoint(p));
    }

    return originalPoints;
}

}
