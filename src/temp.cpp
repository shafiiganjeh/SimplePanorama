#ifndef GRAPH_H
#define GRAPH_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "CutPlanar.h"
#include <iostream>
#include "_blending.h"

namespace gcut {

    struct overlap_pair{

        cv::Mat img1;
        cv::Mat img2;
        cv::Mat mask1;
        cv::Mat mask2;

    };


    class gradient {
        public:
            virtual ~gradient() = default;
            virtual double read(int i, int j) const = 0;
    };


    class simple_gradient : public gradient {

        private:

            int mask_constatnt = 255*2;
            cv::Mat a_dif;
            const cv::Mat& M1;
            const cv::Mat& M2;
            uchar read_Mat(int i,const cv::Mat* mat) const;

        public:

            simple_gradient(const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& mask1, const cv::Mat& mask2):  M1(mask1),M2(mask2){

                CV_Assert(img1.size() == img2.size() && img1.type() == CV_8UC1 && img2.type() == CV_8UC1);
                CV_Assert(mask1.size() == img1.size() && mask1.type() == CV_8U);
                CV_Assert(mask2.size() == img1.size() && mask2.type() == CV_8U);

                cv::absdiff(img1, img2, a_dif);

            }

            double read(int i, int j) const override;

    };


    class EdgeWeight {
        private:
            const gradient& weightFunc;

        public:
            EdgeWeight(const gradient& wf) : weightFunc(wf){}

            double getWeight(int i, int j) const {
                return weightFunc.read(i, j);
            }
    };


    void define_graph(int n, int m,EdgeWeight & reader);

    struct overlap_pair get_overlap(
        const cv::Mat& img1,
        const cv::Mat& img2,
        const cv::Point& corners1,
        const cv::Point& corners2);

    cv::Mat computeDissimilarity(
        const cv::Mat& img1,
        const cv::Mat& img2,
        const cv::Mat& mask1,
        const cv::Mat& mask2,
        float replace_value = 0.0f,
        float eps = 1e-6f);

}

#endif

/*
    void define_graph(int n, int m){

        int n_face = (n-1);
        int n_vert = 1;
        int n_edge = n;

        PlanarVertex vertex[n*m + n_vert + n_vert];
        PlanarEdge edge[2*n*m-m-n + n_edge + n_edge];
        PlanarFace face[n*m-n-m + 2 + n_face + n_face];

        int total_horizontal_edges = n * (m - 1);
        // split eges in horizonatal and vertical.



        int edge_index = n_edge;

        // Horizontal edges
        for (int a = 0; a < n; a++) {
            for (int b = 0; b < m-1; b++) {
                int start = a * m + b + n_vert;
                int target = a * m + b + 1 + n_vert;
                int left_face, right_face;

                // Face (left):
                if (a == 0) left_face = 0;
                else left_face = 1 + (a-1)*(m-1) + b + n_face;

                // Face (right):
                if (a == n-1) right_face = 0;
                else right_face = 1 + a*(m-1) + b + n_face;

                edge[edge_index].setEdge(vertex + start,vertex + target,face + left_face,face + right_face, 1, 1);

                std::cout<< "edge_index: "<<edge_index<<"\n"<<"start "<< start<<" target "<<target<<" left_face "<<left_face<<" right_face "<<right_face<<"\n";

                edge_index++;
            }
        }

        // Vertical edges
        for (int a = 0; a < n-1; ++a) {
            for (int b = 0; b < m; ++b) {
                int start = a * m + b + n_vert;
                int target = (a+1) * m + b + n_vert;
                int left_face, right_face;

                // Face (left)
                if (b == 0) left_face = a + 1;
                else left_face = 1 + a*(m-1) + (b-1) + n_face;

                // Face (right)
                if (b == m-1) right_face = a + n_face + n*m-n-m + 2;
                else right_face = 1 + a*(m-1) + b + n_face;

                edge[edge_index].setEdge(vertex + start,vertex + target,face + right_face,face + left_face, 1, 1);

                std::cout<< "edge_index: "<<edge_index<<"\n"<<"start "<< start<<" target "<<target<<" left_face "<<left_face<<" right_face "<<right_face<<"\n";

                edge_index++;
            }
        }


        int edge_index_left = 0;
        for (int a = 0; a < n; ++a) {
            int start = 0;
            int target = a*m+1;
            int left_face, right_face;


            if(a == 0) left_face = 0;
            else left_face = a;
            if(a == n - 1) right_face = 0;
            else right_face = a + 1;

            std::cout<< "edge_index: "<<edge_index_left<<"\n"<<"start "<< start<<" target "<<target<<" left_face "<<left_face<<" right_face "<<right_face<<"\n";

            edge[edge_index_left].setEdge(vertex + start,vertex + target,face + left_face,face + right_face, 1, 1);

            edge_index_left ++;
        }

        for (int a = 0; a < n; a++) {
            int target = n*m + n_vert;
            int start = (a + 1)*m;
            int left_face, right_face;


            if(a == 0) left_face = 0;
            else left_face = a + n_face + n*m-n-m + 1;
            if(a == n - 1) right_face = 0;
            else right_face = a + n_face + n*m-n-m + 2;

            std::cout<< "edge_index: "<<edge_index_left + edge_index<<"\n"<<"start "<< start<<" target "<<target<<" left_face "<<left_face<<" right_face "<<right_face<<"\n";

            edge[edge_index].setEdge(vertex + start,vertex + target,face + left_face,face + right_face, 1, 1);

            edge_index++;
        }

        int max_edges;
        n > 4 ? max_edges = n: max_edges = 4;
        PlanarEdge *edges_CCW[max_edges];

        for (int a = 0; a < n; ++a) {
            for (int b = 0; b < m; ++b) {
                int vertex_index = a * m + b + n_vert ;
                int count = 0;

                std::cout<< "vertex_index: "<<vertex_index<<"\n";

                // West edge (horizontal edge from (a, b-1) to (a, b))
                if (b > 0) {
                    int edge_id = a * (m - 1) + (b - 1) + n_edge;
                    edges_CCW[count] = edge + edge_id;
                    std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                if (b == 0) {
                    int edge_id = a;
                    edges_CCW[count] = edge + edge_id;
                    std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                // South edge (vertical edge from (a, b) to (a+1, b))
                if (a < n - 1) {
                    int edge_id = total_horizontal_edges + a * m + b + n_edge;
                    edges_CCW[count] = edge + edge_id;
                    std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                // East edge (horizontal edge from (a, b) to (a, b+1))
                if (b < m - 1) {
                    int edge_id = a * (m - 1) + b + n_edge;
                    edges_CCW[count] = edge + edge_id;
                    std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }
                if (b == m - 1) {
                    int edge_id = a + 2*n*m-m-n + n_edge;
                    edges_CCW[count] = edge + edge_id;
                    std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                // North edge (vertical edge from (a-1, b) to (a, b))
                if (a > 0) {
                    int edge_id = total_horizontal_edges + (a - 1) * m + b + n_edge;
                    edges_CCW[count] = edge + edge_id;
                    std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                vertex[vertex_index].setEdgesCCW(edges_CCW, count);
            }
        }
        std::cout<< "vertex_index: "<<0<<"\n";
        edges_CCW[0] = edge+ 0;
        std::cout<< "edges_CCW[count]: "<<0<<" edge_id "<<0<<"\n";
        int count = 1;
        for (int a = 1; a < n; a++) {

            edges_CCW[a] = edge + n-a;
            std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<n-a<<"\n";
            count ++;

        }

        vertex[0].setEdgesCCW(edges_CCW, count);

        std::cout<< "vertex_index: "<<n*m + n_vert <<"\n";
        count = 0;
        for (int a = 0; a < n; a++) {

            edges_CCW[a] = edge + 2*n*m-m-n + n_edge + a;
            std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<2*n*m-m-n + n_edge + a<<"\n";
            count ++;

        }
        vertex[n*m + n_vert].setEdgesCCW(edges_CCW, count);


        CutPlanar planar_cut;
        planar_cut.initialize(n*m + n_vert + n_vert,vertex, 2*n*m-m-n + n_edge + n_edge,edge, n*m-n-m + 2 + n_face + n_face,face);
        planar_cut.setSource(0);
        planar_cut.setSink  (n*m + n_vert);

        double flow;
        flow = planar_cut.getMaxFlow();
        std::cout << "Maxmal Flow: " << flow << std::endl;

    }

    */


    void define_graph(int n, int m,EdgeWeight & reader){

        int n_face = (n-1);
        int n_vert = 1;
        int n_edge = n;

        PlanarVertex vertex[n*m + n_vert + n_vert];

        PlanarEdge edge[2*n*m-m-n + n_edge + n_edge];
        PlanarFace face[n*m-n-m + 2 + n_face + n_face];

        int total_horizontal_edges = n * (m - 1);
        // split eges in horizonatal and vertical.



        int edge_index = n_edge;

        // Horizontal edges
        for (int a = 0; a < n; a++) {
            for (int b = 0; b < m-1; b++) {
                int start = a * m + b + n_vert;
                int target = a * m + b + 1 + n_vert;
                int left_face, right_face;

                // Face (left):
                if (a == 0) left_face = 0;
                else left_face = 1 + (a-1)*(m-1) + b + n_face;

                // Face (right):
                if (a == n-1) right_face = 0;
                else right_face = 1 + a*(m-1) + b + n_face;

                edge[edge_index].setEdge(vertex + start,vertex + target,face + left_face,face + right_face, 1, 1);

                //std::cout<< "edge_index: "<<edge_index<<"\n"<<"start "<< start<<" target "<<target<<" left_face "<<left_face<<" right_face "<<right_face<<"\n";

                edge_index++;
            }
        }

        // Vertical edges
        for (int a = 0; a < n-1; ++a) {
            for (int b = 0; b < m; ++b) {
                int start = a * m + b + n_vert;
                int target = (a+1) * m + b + n_vert;
                int left_face, right_face;

                // Face (left)
                if (b == 0) left_face = a + 1;
                else left_face = 1 + a*(m-1) + (b-1) + n_face;

                // Face (right)
                if (b == m-1) right_face = a + n_face + n*m-n-m + 2;
                else right_face = 1 + a*(m-1) + b + n_face;

                edge[edge_index].setEdge(vertex + start,vertex + target,face + right_face,face + left_face, 1, 1);

                //std::cout<< "edge_index: "<<edge_index<<"\n"<<"start "<< start<<" target "<<target<<" left_face "<<left_face<<" right_face "<<right_face<<"\n";

                edge_index++;
            }
        }


        int edge_index_left = 0;
        for (int a = 0; a < n; ++a) {
            int start = 0;
            int target = a*m+1;
            int left_face, right_face;


            if(a == 0) left_face = 0;
            else left_face = a;
            if(a == n - 1) right_face = 0;
            else right_face = a + 1;

            //std::cout<< "edge_index: "<<edge_index_left<<"\n"<<"start "<< start<<" target "<<target<<" left_face "<<left_face<<" right_face "<<right_face<<"\n";

            edge[edge_index_left].setEdge(vertex + start,vertex + target,face + left_face,face + right_face, 1, 1);

            edge_index_left ++;
        }

        for (int a = 0; a < n; a++) {
            int target = n*m + n_vert;
            int start = (a + 1)*m;
            int left_face, right_face;


            if(a == 0) left_face = 0;
            else left_face = a + n_face + n*m-n-m + 1;
            if(a == n - 1) right_face = 0;
            else right_face = a + n_face + n*m-n-m + 2;

            //std::cout<< "edge_index: "<<edge_index_left + edge_index<<"\n"<<"start "<< start<<" target "<<target<<" left_face "<<left_face<<" right_face "<<right_face<<"\n";

            edge[edge_index].setEdge(vertex + start,vertex + target,face + left_face,face + right_face, 1, 1);

            edge_index++;
        }

        int max_edges;
        n > 4 ? max_edges = n: max_edges = 4;
        PlanarEdge *edges_CCW[max_edges];

        for (int a = 0; a < n; ++a) {
            for (int b = 0; b < m; ++b) {
                int vertex_index = a * m + b + n_vert ;
                int count = 0;

                //std::cout<< "vertex_index: "<<vertex_index<<"\n";

                // West edge (horizontal edge from (a, b-1) to (a, b))
                if (b > 0) {
                    int edge_id = a * (m - 1) + (b - 1) + n_edge;
                    edges_CCW[count] = edge + edge_id;
                    //std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                if (b == 0) {
                    int edge_id = a;
                    edges_CCW[count] = edge + edge_id;
                    //std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                // South edge (vertical edge from (a, b) to (a+1, b))
                if (a < n - 1) {
                    int edge_id = total_horizontal_edges + a * m + b + n_edge;
                    edges_CCW[count] = edge + edge_id;
                    //std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                // East edge (horizontal edge from (a, b) to (a, b+1))
                if (b < m - 1) {
                    int edge_id = a * (m - 1) + b + n_edge;
                    edges_CCW[count] = edge + edge_id;
                    //std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }
                if (b == m - 1) {
                    int edge_id = a + 2*n*m-m-n + n_edge;
                    edges_CCW[count] = edge + edge_id;
                    //std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                // North edge (vertical edge from (a-1, b) to (a, b))
                if (a > 0) {
                    int edge_id = total_horizontal_edges + (a - 1) * m + b + n_edge;
                    edges_CCW[count] = edge + edge_id;
                    //std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                vertex[vertex_index].setEdgesCCW(edges_CCW, count);
            }
        }
        //std::cout<< "vertex_index: "<<0<<"\n";
        edges_CCW[0] = edge+ 0;
        //std::cout<< "edges_CCW[count]: "<<0<<" edge_id "<<0<<"\n";
        int count = 1;
        for (int a = 1; a < n; a++) {

            edges_CCW[a] = edge + n-a;
            //std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<n-a<<"\n";
            count ++;

        }

        vertex[0].setEdgesCCW(edges_CCW, count);

        //std::cout<< "vertex_index: "<<n*m + n_vert <<"\n";
        count = 0;
        for (int a = 0; a < n; a++) {

            edges_CCW[a] = edge + 2*n*m-m-n + n_edge + a;
            //std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<2*n*m-m-n + n_edge + a<<"\n";
            count ++;

        }
        vertex[n*m + n_vert].setEdgesCCW(edges_CCW, count);

        CutPlanar planar_cut;
        planar_cut.initialize(n*m + n_vert + n_vert,vertex, 2*n*m-m-n + n_edge + n_edge,edge, n*m-n-m + 2 + n_face + n_face,face);

        std::cout<<"\n"<< "2*n*m-m-n + n_edge + n_edge: "<<2*n*m-m-n + n_edge + n_edge<<"\n";
std::cout<<"\n"<< "2*n*m-m-n + n_edge + n_edge: "<<2*n*m-m-n + n_edge + n_edge<<"\n";

        planar_cut.setSource(0);
        planar_cut.setSink  (n*m + n_vert);

        double flow;
        flow = planar_cut.getMaxFlow();
        std::cout << "Maxmal Flow: " << flow << std::endl;

    }






#include "_graph_cut.h"
#include "_max_flow.h"

namespace gcut {


    void resizeToMaxArea(const cv::Mat& image, int n, cv::Mat& result) {
        if (n <= 0) {
            result = cv::Mat();
            return;
        }

        int w0 = image.cols;
        int h0 = image.rows;

        if (w0 <= 0 || h0 <= 0) {
            result = image.clone();
            return;
        }

        double area0 = static_cast<double>(w0) * h0;
        double s0 = std::sqrt(static_cast<double>(n) / area0);

        int w_floor = static_cast<int>(std::floor(w0 * s0));
        int w_ceil = static_cast<int>(std::ceil(w0 * s0));
        int h_floor = static_cast<int>(std::floor(h0 * s0));
        int h_ceil = static_cast<int>(std::ceil(h0 * s0));

        std::vector<std::pair<int, int>> candidates = {
            {w_floor, h_floor},
            {w_floor, h_ceil},
            {w_ceil, h_floor},
            {w_ceil, h_ceil}
        };

        long long best_area = -1;
        int best_w = -1, best_h = -1;
        double orig_aspect = static_cast<double>(w0) / h0;
        double best_aspect_diff = DBL_MAX;

        for (const auto& cand : candidates) {
            int w = cand.first;
            int h = cand.second;

            if (w <= 0 || h <= 0)
                continue;

            long long area = static_cast<long long>(w) * h;
            if (area > n)
                continue;

            double aspect_ratio = static_cast<double>(w) / h;
            double aspect_diff = std::fabs(aspect_ratio - orig_aspect);

            if (area > best_area) {
                best_area = area;
                best_w = w;
                best_h = h;
                best_aspect_diff = aspect_diff;
            } else if (area == best_area) {
                if (aspect_diff < best_aspect_diff) {
                    best_w = w;
                    best_h = h;
                    best_aspect_diff = aspect_diff;
                }
            }
        }

        if (best_w <= 0 || best_h <= 0) {
            best_w = 1;
            best_h = 1;
        }

        cv::resize(image, result, cv::Size(best_w, best_h));
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

    double simple_gradient::read(int i, int j) const{

        int MA = read_Mat(i,&(this->M1));
        int MB = read_Mat(j,&(this->M2));
        int conti = read_Mat(i,&(this->cont));
        int contj = read_Mat(j,&(this->cont));

        if( ((i%M1.cols == (int)(M1.cols / 2)) or (j%M2.cols == (int)(M2.cols / 2))) and ((MA > 0) or (MB > 0))){

            return 0;

        }else if((conti > 0) or (contj > 0) ){

            return 0;

        }else if( (MA > 0) or (MB > 0) ){

            return 255;

        }else{

            uchar A = read_Mat(i,&(this->a_dif));
            uchar B = read_Mat(j,&(this->a_dif));

            //return ((double)A + (double)B) / (2*255);
            return 255;
        }

    }

    cv::Mat computeDissimilarity(
        const cv::Mat& img1,
        const cv::Mat& img2,
        const cv::Mat& mask1,
        const cv::Mat& mask2,
        float replace_value,
        float eps) {

        cv::Mat img1_res;
        cv::Mat img2_res;
        cv::Mat mask1_res;
        cv::Mat mask2_res;

        int size = 240*240;

        resizeToMaxArea(img1, size, img1_res);
        resizeToMaxArea(img2, size, img2_res);
        resizeToMaxArea(mask1, size, mask1_res);
        resizeToMaxArea(mask2, size, mask2_res);

        int tett = 77;

        //mask1_res.col(0).setTo(tett);

        cv::Mat result(img1_res.size(),img1_res.type());

        class simple_gradient grad(img1_res,img2_res,mask1_res,mask2_res);
        class EdgeWeight reader(grad);

        int n = img1_res.rows;
        int m = img1_res.cols;


        for(int i = 0;i < img1_res.cols;i++){
            for(int j = 0;j < img1_res.rows;j++){

                int idx = i+ j*img1_res.cols;

                result.at<uchar>(j,i) = reader.getWeight(idx,idx);

            }
        }

        cv::imshow("Image Display", result);
        cv::waitKey(0);

         std::vector<int> labels = define_graph(img1_res.rows, img1_res.cols,reader);

        for(int i :labels){
            int row = i / m;
            int col = i % m;
            result.at<uchar>(row, col) = 255;

        }

        int source = ((int)((n-1)/2)) * m + m - 1;
        std::cout<<"source: "<<source<<"\n";
        int row = source / m;
        int col = source % m;
        result.at<uchar>(row, col) = 254;

        cv::imshow("Image Display", result);
        cv::waitKey(0);

        /*
        CV_Assert(img1.size() == img2.size() && img1.type() == img2.type());
        CV_Assert(mask1.size() == img1.size() && mask1.type() == CV_8U);
        CV_Assert(mask2.size() == img1.size() && mask2.type() == CV_8U);
        cv::Mat f_img1, f_img2;
        img1.convertTo(f_img1, CV_32F);
        img2.convertTo(f_img2, CV_32F);

        //absolute difference
        cv::Mat a_dif;
        cv::absdiff(f_img1, f_img2, a_dif);

        //Scharr gradients magnitude
        cv::Mat grad1_x, grad1_y, grad2_x, grad2_y;
        cv::Scharr(f_img1, grad1_x, CV_32F, 1, 0);
        cv::Scharr(f_img1, grad1_y, CV_32F, 0, 1);

        cv::Scharr(f_img2, grad2_x, CV_32F, 1, 0);
        cv::Scharr(f_img2, grad2_y, CV_32F, 0, 1);

        cv::Mat mag1, mag2;
        cv::magnitude(grad1_x, grad1_y, mag1);
        cv::magnitude(grad2_x, grad2_y, mag2);

        // Scaled the difference
        cv::Mat denom;
        cv::add(mag1, mag2, denom);
        denom += eps;
        cv::Mat result;
        cv::divide(a_dif, denom, result);

        //mask result
        cv::Mat mask_union;
        cv::bitwise_or(mask1, mask2, mask_union);

        result.setTo(replace_value, mask_union);

        */
        return result;
    }



    std::vector<int> define_graph(int n, int m,class EdgeWeight reader){

        PlanarVertex vertex[n*m];
        PlanarEdge edge[2*n*m-m-n];
        PlanarFace face[n*m-n-m + 2];
        int edge_index = 0;
        int total_horizontal_edges = n * (m - 1);
        // split eges in horizonatal and vertical.

        // Horizontal edges
        for (int a = 0; a < n; a++) {
            for (int b = 0; b < m-1; b++) {
                int start = a * m + b;
                int target = a * m + b + 1;
                int left_face, right_face;

                // Face (left):
                if (a == 0) left_face = 0;
                else left_face = 1 + (a-1)*(m-1) + b;

                // Face (right):
                if (a == n-1) right_face = 0;
                else right_face = 1 + a*(m-1) + b;


                double w = reader.getWeight(start,target);
                std::cout<<w;
                edge[edge_index].setEdge(vertex + start,vertex + target,face + left_face,face + right_face, w, w);

                //std::cout<< "edge_index: "<<edge_index<<"\n"<<"start "<< start<<" target "<<target<<" left_face "<<left_face<<" right_face "<<right_face<<"\n";

                edge_index++;
            }
        }

        // Vertical edges (top to bottom)
        for (int a = 0; a < n-1; ++a) {
            for (int b = 0; b < m; ++b) {
                int start = a * m + b;         //(a, b)
                int target = (a+1) * m + b;     //(a+1, b)
                int left_face, right_face;

                // Face (left)
                if (b == 0) left_face = 0;
                else left_face = 1 + a*(m-1) + (b-1);

                // Face (right)
                if (b == m-1) right_face = 0;
                else right_face = 1 + a*(m-1) + b;

                double w = reader.getWeight(start,target);
                std::cout<<w;
                edge[edge_index].setEdge(vertex + start,vertex + target,face + right_face,face + left_face, w, w);

                //std::cout<< "edge_index: "<<edge_index<<"\n"<<"start "<< start<<" target "<<target<<" left_face "<<left_face<<" right_face "<<right_face<<"\n";

                edge_index++;
            }
        }

        PlanarEdge *edges_CCW[4];

        for (int a = 0; a < n; ++a) {
            for (int b = 0; b < m; ++b) {
                int vertex_index = a * m + b;
                int count = 0;

                //std::cout<< "vertex_index: "<<vertex_index<<"\n";

                // West edge (horizontal edge from (a, b-1) to (a, b))
                if (b > 0) {
                    int edge_id = a * (m - 1) + (b - 1);
                    edges_CCW[count] = edge + edge_id;
                    //std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                // South edge (vertical edge from (a, b) to (a+1, b))
                if (a < n - 1) {
                    int edge_id = total_horizontal_edges + a * m + b;
                    edges_CCW[count] = edge + edge_id;
                    //std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                // East edge (horizontal edge from (a, b) to (a, b+1))
                if (b < m - 1) {
                    int edge_id = a * (m - 1) + b;
                    edges_CCW[count] = edge + edge_id;
                    //std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                // North edge (vertical edge from (a-1, b) to (a, b))
                if (a > 0) {
                    int edge_id = total_horizontal_edges + (a - 1) * m + b;
                    edges_CCW[count] = edge + edge_id;
                    //std::cout<< "edges_CCW[count]: "<<count<<" edge_id "<<edge_id<<"\n";
                    count++;
                }

                vertex[vertex_index].setEdgesCCW(edges_CCW, count);
            }
        }

        int source = ((int)((n-1)/2)) * m;
        int sink = source + m - 1;

        CutPlanar planar_cut;
        planar_cut.initialize(n*m,vertex, 2*n*m-m-n,edge, n*m-n-m + 2,face);
        planar_cut.setSource(source);
        planar_cut.setSink(sink);
        double flow;

        flow = planar_cut.getMaxFlow();
        std::cout << "Maxmal Flow: " << flow << std::endl;
        std::cout<<"\n"<<"-----------------done"<<"\n";
        std::vector<int> labels;
        labels = planar_cut.getLabels(CutPlanar::LABEL_SINK);

        return labels;
    }



    struct overlap_pair get_overlap(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const cv::Point& corners1,
        const cv::Point& corners2){

        struct overlap_pair res;
        cv::Mat gray_image1;
        cv::Mat gray_image2;

        cv::cvtColor(img1, gray_image1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(img2, gray_image2, cv::COLOR_BGR2GRAY);

        cv::Mat mask1 = blnd::createSurroundingMask(img1,false, 1);
        cv::Mat mask2 = blnd::createSurroundingMask(img2,false, 1);

        cv::Rect rect_1(corners1.x, corners1.y, img1.cols, img1.rows);
        cv::Rect rect_2(corners2.x, corners2.y, img2.cols, img2.rows);

        cv::Rect overlap_rect = rect_1 & rect_2;
                if (overlap_rect.width <= 0 || overlap_rect.height <= 0) {
                    throw("ERROR: Non overlapping image segment");
                }

        cv::Rect roi_1(overlap_rect.x - corners1.x, overlap_rect.y - corners1.y, overlap_rect.width, overlap_rect.height);
        cv::Rect roi_2(overlap_rect.x - corners2.x, overlap_rect.y - corners2.y, overlap_rect.width, overlap_rect.height);

        cv::Mat mask_1_roi = mask1(roi_1);
        cv::Mat mask_2_roi = mask2(roi_2);

        cv::Mat gray_1_roi = gray_image1(roi_1);
        cv::Mat gray_2_roi = gray_image2(roi_2);

        res.img1 = gray_1_roi;
        res.img2 = gray_2_roi;
        res.mask1 = mask_1_roi;
        res.mask2 = mask_2_roi;

        return res;

    }


}
