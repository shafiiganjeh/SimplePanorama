
#include "_graph_cut.h"

namespace gcut {


    //https://faculty.cc.gatech.edu/~turk/my_papers/graph_cuts.pdf
std::vector<cv::Mat> graph_cut(const std::vector<cv::Mat>& images,const std::vector<cv::Mat>& masks,const std::vector<cv::Point>& top_lefts,std::vector<int>& seq,std::atomic<double>* f_adress){

    struct util::size_data dim = util::get_pan_dimension(top_lefts,images);

    cv::Mat panorama(
        dim.dims.height,
        dim.dims.width,
        CV_8UC3,
        cv::Scalar(0, 0, 0)
    );

    cv::Mat panorama_test(
        dim.dims.height,
        dim.dims.width,
        CV_8UC3,
        cv::Scalar(0, 0, 0)
    );

    cv::Mat scene(
        dim.dims.height,
        dim.dims.width,
        CV_8U,
        cv::Scalar(0)
    );

    std::vector<cv::Rect> image_roi(images.size());
    for (int i = 0; i < images.size(); i++) {
        cv::Rect roi(
            top_lefts[seq[i]].x - dim.min_x,  // x offset
            top_lefts[seq[i]].y - dim.min_y,  // y offset
            images[seq[i]].size().width,
            images[seq[i]].size().height
        );
        image_roi[seq[i]] = roi;
    }

    cv::Mat adj = computeOverlapMatrix(image_roi);
    std::cout<<adj;

    std::vector<cv::Mat> masks_copy;
    for (const auto& mat : masks) {
        masks_copy.push_back(mat.clone()); // clone() creates a deep copy
    }

    images[seq[0]].copyTo(panorama(image_roi[seq[0]]),masks_copy[seq[0]]);

    masks[seq[0]].copyTo(scene(image_roi[seq[0]]),masks[seq[0]]);
    std::vector<int> added;

    double add = (1.0/3.2) * (1.0/(double)seq.size());

    for(int s = 1;s < seq.size();s++){

        cv::Mat gray_image1,gray_image2;
        cv::cvtColor(panorama(image_roi[seq[s]]), gray_image1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(images[seq[s]], gray_image2, cv::COLOR_BGR2GRAY);


        cv::Mat cmat = gcut::computeCut(gray_image1,
                                    gray_image2,
                                    scene(image_roi[seq[s]]),
                                    masks[seq[s]]);


        cmat.copyTo(scene(image_roi[seq[s]]),cmat);
        images[seq[s]].copyTo(panorama(image_roi[seq[s]]),cmat);

        masks_copy[seq[s]] = cmat;
        if(not (f_adress == NULL)){*f_adress = *f_adress + add;}
    }


    for(int s = 0;s < seq.size();s++){

        for(int &a : added){

            cv::Rect overlap = image_roi[a] bitand image_roi[seq[s]];

            if(overlap.empty()) continue;

            cv::Rect local_input(
                overlap.x - image_roi[a].x,
                overlap.y - image_roi[a].y,
                overlap.width,
                overlap.height
            );

            cv::Rect local_remove(
                overlap.x - image_roi[seq[s]].x,
                overlap.y - image_roi[seq[s]].y,
                overlap.width,
                overlap.height
            );

            cv::Mat input_roi = masks_copy[a](local_input);
            cv::Mat remove_roi = masks_copy[seq[s]](local_remove);

            input_roi.setTo(0, remove_roi);

        }

        added.push_back(seq[s]);

    }

    return masks_copy;
}


cv::Mat computeOverlapMatrix(const std::vector<cv::Rect>& rois) {
    int n = rois.size();
    cv::Mat overlap(n, n, CV_32SC1); // Matrix to store integer overlap areas

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            cv::Rect intersection = rois[i] & rois[j];
            int area = intersection.area();
            if(i == j){
                overlap.at<int>(i, j) = 0;
            }else{
                overlap.at<int>(i, j) = area;
            }

        }
    }
    return overlap;
}


    cv::Mat combineMasks(
        cv::Point p1, const cv::Mat& m1,
        cv::Point p2, const cv::Mat& m2,
        const cv::Mat& m3,
        bool invert) {

        int x1 = p1.x, y1 = p1.y;
        int x2 = p2.x, y2 = p2.y;

        int x_union = std::min(x1, x2);
        int y_union = std::min(y1, y2);
        int x_max = std::max(x1 + m1.cols, x2 + m2.cols);
        int y_max = std::max(y1 + m1.rows, y2 + m2.rows);
        int w_union = x_max - x_union;
        int h_union = y_max - y_union;

        int x_overlap = std::max(x1, x2);
        int y_overlap = std::max(y1, y2);
        int x_overlap_end = std::min(x1 + m1.cols, x2 + m2.cols);
        int y_overlap_end = std::min(y1 + m1.rows, y2 + m2.rows);
        int w_overlap = x_overlap_end - x_overlap;
        int h_overlap = y_overlap_end - y_overlap;
        cv::Mat m2_mod = m2.clone();

        if (w_overlap > 0 && h_overlap > 0 && !m3.empty()) {

            cv::Rect roi2_in_img2(x_overlap - x2, y_overlap - y2, w_overlap, h_overlap);

                cv::Mat m3_used = m3;
                if (m3_used.size() != cv::Size(w_overlap, h_overlap)) {
                    resize(m3, m3_used, cv::Size(w_overlap, h_overlap), 0, 0, cv::INTER_NEAREST);
                }

                cv::bitwise_not(m2_mod, m2_mod);

                add(m2_mod(roi2_in_img2), m3_used, m2_mod(roi2_in_img2));
        }

        cv::bitwise_not(m2_mod, m2_mod);

        return m2_mod;
    }


    uchar simple_gradient::read_Mat(int i,const cv::Mat* mat) const{

        int total = mat->rows * mat->cols;
        if (i < 0 || i >= total) {
            throw std::out_of_range("Index out of matrix bounds");
        }

        if (mat->isContinuous()) {
            return mat->ptr<uchar>(0)[i];
        }else {
            int row = i / mat->cols;
            int col = i % mat->cols;
            return mat->at<uchar>(row, col);
        }

    }

    edge_value simple_gradient::read(int i, int j) const{

        int MA = read_Mat(i,&(this->M1));
        int MB = read_Mat(j,&(this->M2));
        edge_value ret;

        if( not ((MA > 0) or (MB > 0)) ){

            ret.h = mask_constant;
            ret.v = mask_constant;

            return ret;

        }else{

            uchar A = read_Mat(i,&(this->a_dif));
            uchar B = read_Mat(j,&(this->a_dif));

            ret.h = ((float)A + (float)B) / (2*255);
            ret.v = ((float)A + (float)B) / (2*255);

            return ret;

        }

    }


    float scharr_gradient::read_Mat(int i,const cv::Mat* mat) const{

        int total = mat->rows * mat->cols;
        if (i < 0 || i >= total) {
            throw std::out_of_range("Index out of matrix bounds");
        }

        if (mat->isContinuous()) {
            return mat->ptr<float>(0)[i];
        }else {
            int row = i / mat->cols;
            int col = i % mat->cols;
            return mat->at<float>(row, col);
        }

    }


    edge_value scharr_gradient::read(int i, int j) const{

        float MA = read_Mat(i,&(this->M1));
        float MB = read_Mat(j,&(this->M2));
        edge_value ret;

        if( not ((MA > 0) or (MB > 0)) ){

            ret.h = mask_constant;
            ret.v = mask_constant;

            return ret;

        }else{

            float A = read_Mat(i,&(this->a_dif));
            float B = read_Mat(j,&(this->a_dif));

            float g1x_i = read_Mat(i,&(this->grad1_x));
            float g1y_i = read_Mat(i,&(this->grad1_y));
            float g1x_j = read_Mat(j,&(this->grad1_x));
            float g1y_j = read_Mat(j,&(this->grad1_y));

            float g2x_i = read_Mat(i,&(this->grad2_x));
            float g2y_i = read_Mat(i,&(this->grad2_y));
            float g2x_j = read_Mat(j,&(this->grad2_x));
            float g2y_j = read_Mat(j,&(this->grad2_y));

            ret.h = (A + B) / ( abs(g1y_i) + abs(g1y_j) + abs(g2y_i) + abs(g2y_j) + eps);
            ret.v = (A + B) / ( abs(g1x_i) + abs(g1x_j) + abs(g2x_i) + abs(g2x_j) + eps);
            return ret;

        }

    }


    cv::Mat computeCut(
        const cv::Mat& img1,
        const cv::Mat& img2,
        const cv::Mat& mask1,
        const cv::Mat& mask2) {

        cv::Mat img1_res;
        cv::Mat img2_res;
        cv::Mat mask1_res;
        cv::Mat mask2_res;

        cv::Mat result(img1.size(),img1.type());
        class scharr_gradient grad(img1,img2,mask1,mask2);
        class EdgeWeight reader(grad);

        class gcut::graph_object obj(mask1,mask2);

        //Tweight.rescale(res_factor.first,res_factor.second);

        int n = img1_res.rows;
        int m = img1_res.cols;

        edge_value ret;
        for(int i = 0;i < img1.cols;i++){
            for(int j = 0;j < img1.rows;j++){

                int idx = i+ j*img1.cols;
                ret = grad.read(idx,idx);
                result.at<uchar>(j,i) = (uchar)ret.h;

            }
        }


        std::vector<int> labels = define_graph_full(obj,reader);

/*
        cv::Mat result2(img1_res.size(),img1_res.type());
        std::vector<int> labels = define_graph(n, m,reader,Tweight);

        for(int i = 0;i < img1_res.cols;i++){
            for(int j = 0;j < img1_res.rows;j++){

                int idx = i+ j*img1_res.cols;

                if(labels[idx] > 0){
                    result2.at<uchar>(j,i) = 255;
                }else{
                    result2.at<uchar>(j,i) = 0;
                }

            }
        }

*/
        return obj.write_cut(labels);
    }


    std::vector<int> define_graph_full(class gcut::graph_object &obj,class EdgeWeight &reader){

        std::vector<int> labels;
        typedef Graph<float,float,float> GraphType;

        GraphType *g = new GraphType(/*estimated # of nodes*/ obj.num_of_nodes, /*estimated # of edges*/ obj.num_of_edges);

        for(int i = 0;i < obj.num_of_nodes;i++){

            g -> add_node();

        }


        int edge_index = 0;
        for(int i = 0;i < obj.num_of_nodes;i++){

            struct property p = obj.read_graph(i);

            if(p.h_edge){

                int start = obj.index[i];
                int target = obj.index[p.h_edge_connection];

                edge_value ret = reader.getWeight(start,target);
                g -> add_edge( i, p.h_edge_connection,    /* capacities */  ret.h, ret.h );

            }


            if(p.v_edge){

                int start = obj.index[i];
                int target = obj.index[p.v_edge_connection];

                edge_value ret = reader.getWeight(start,target);
                g -> add_edge( i, p.v_edge_connection,    /* capacities */  ret.v, ret.v );

            }

            if(p.sink or p.source){

                g -> add_tweights(i,   /* capacities */  p.sink*5000, p.source*5000);

            }

        }

        float flow = g -> maxflow();
        std::cout<<"\n"<<"flow: "<<flow<<"\n";

        for(int i = 0;i < obj.num_of_nodes;i++){

            labels.push_back(g->what_segment(i) == GraphType::SOURCE);

        }

        obj.write_cut(labels);

        return labels;

    }



}
