
#include "_util.h"


namespace util {


cv::Rect scaleRect(const cv::Rect& r, double xs, double sy) {

    int x = (int)std::round(r.x * xs);
    int y = (int)std::round(r.y * sy);

    int width = (int)std::round(r.width * xs);
    int height = (int)std::round(r.height * sy);

    return cv::Rect(x, y, width, height);
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

                temp.connectivity = computeRowSumDividedByZeroCount(sub_graph);

                for(int i = 1; i < sub_graph.rows; i++) {
                    sub_graph.row(i).colRange(0, i).setTo(0);
                }

                temp.adj = sub_graph;
                temp.nodes = graph.size();
                adj_mats.push_back(temp);

            }

        }

        std::sort(adj_mats.begin(), adj_mats.end(), compareDescending);

        return adj_mats;

    }



}
