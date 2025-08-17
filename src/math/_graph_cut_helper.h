
#ifndef GRAPH_HELP_H
#define GRAPH_HELP_H

#include <opencv2/opencv.hpp>

namespace gcut {


    struct property{

        bool sink;
        bool source;
        bool obj;

        bool v_edge = false;
        int v_edge_connection = -1;
        bool h_edge = false;
        int h_edge_connection = -1;

    };


    class graph_object{

    public:

        graph_object(const cv::Mat &scene,const cv::Mat &element,int thickness = 1):element_address(element){

        CV_Assert(scene.size() == element.size());
        CV_Assert((scene.type() == CV_8U) and (element.type() == CV_8U));
        graph = cv::Mat::zeros(scene.size(), CV_8UC3);

        cv::Mat object = extract_object(scene,element);
        index_inverse = std::vector<int>(object.rows*object.cols,-1);

        int n = 0;
        for(int row = 0;row < object.rows;row++){

            for(int col = 0;col < object.cols;col++){

                if(object.at<uchar>(row,col)){

                    index.push_back(col+row*object.cols);
                    index_inverse[col+row*object.cols] = n;
                    n++;

                }

            }

        }

        divisor = object.cols;
        obj_row = object.rows;
        obj_col = object.cols;

        num_of_nodes = index.size();

        cv::Mat cont_scene = find_contour(scene,thickness);
        cv::Mat cont_element = find_contour(element,thickness);
        cv::Mat cont_object = find_contour(object,thickness);

        cont_scene = extract_object(cont_object,cont_scene);
        cont_element = extract_object(cont_object,cont_element);
        cont_object.release();

        std::vector<cv::Mat> ch;

        ch.push_back(object);
        object.release();
        ch.push_back(cont_scene);
        cont_scene.release();
        ch.push_back(cont_element);
        cont_element.release();

        cv::merge(ch, graph);
        get_edge_number();

    }

        int num_of_nodes;
        int num_of_edges;

        std::vector<int> index;
        std::vector<int> index_inverse;

        struct property read_graph(int n);
        cv::Mat write_cut(std::vector<int> label);

        void show();
        cv::Mat return_graph();

    private:

        const cv::Mat &element_address;

        int divisor;
        int obj_row;
        int obj_col;

        cv::Mat graph;
        struct property read_mat(int row,int col);
        void write_to_obj(int n,uchar val,std::vector<int> &channel);

        cv::Mat extract_object(const cv::Mat &input,const cv::Mat &extract);
        cv::Mat find_contour(const cv::Mat& inputMask,int thickness);
        void get_edge_number();

    };


}

#endif
