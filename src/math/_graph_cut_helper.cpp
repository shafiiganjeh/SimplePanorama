
#include "_graph_cut_helper.h"

namespace gcut {


    cv::Mat graph_object::extract_object(const cv::Mat &input,const cv::Mat &extract){

        cv::Mat result = cv::Mat::zeros(input.size(), input.type());
        if (input.type() != CV_8UC1 or extract.type() != CV_8UC1) {
            throw std::runtime_error("Error: Masks must be single-channel 8-bit (CV_8UC1)\n");
        }

        if (input.size() != extract.size()) {
            throw std::runtime_error("Error: Mask dimensions must match\n");
        }

        input.copyTo(result, extract);
        return result;

    }


    struct property graph_object::read_mat(int row,int col){

        struct property ret;

        uchar* pixelPtr = graph.ptr<uchar>(row) + col * 3;

        ret.obj  = pixelPtr[0] > 0;
        ret.sink = pixelPtr[1] > 0;
        ret.source  = pixelPtr[2] > 0;

        return ret;

    }


    void graph_object::write_to_obj(int n,uchar val,std::vector<int> &channel){

        auto dv = std::div(index[n], divisor);
        int row = dv.quot;
        int col = dv.rem;

        uchar* pixelPtr = graph.ptr<uchar>(row) + col * 3;
        pixelPtr[channel[0]] = val;
        pixelPtr[channel[1]] = val;
        pixelPtr[channel[2]] = val;

    }


    struct property graph_object::read_graph(int n){

        auto dv = std::div(index[n], divisor);
        int row = dv.quot;
        int col = dv.rem;
        struct property ret = read_mat(row,col);

        if(not (row == obj_row - 1)) {

            struct property v_connected = read_mat(row + 1,col);
            ret.v_edge = v_connected.obj;
            ret.v_edge_connection = index_inverse[col+(row + 1)*obj_col];

        }

        if(not (col == obj_col - 1)) {

            struct property h_connected = read_mat(row,col + 1);
            ret.h_edge = h_connected.obj;
            ret.h_edge_connection = n + 1;

        }

        return ret;

    }


    cv::Mat graph_object::write_cut(std::vector<int> label){

        if(label.size() != num_of_nodes){

            throw std::runtime_error("Label size and node size don't match!");

        }

        cv::Mat cut = element_address.clone();

        for(int i = 0;i <label.size();i++){

            auto dv = std::div(index[i], divisor);
            int row = dv.quot;
            int col = dv.rem;

            cut.at<uchar>(row,col) = 255 * label[i];


        }

        return cut;

    }


    void graph_object::get_edge_number(){

        int edges = 0;
        for(int i = 0;i < num_of_nodes;i++){

            property p = read_graph(i);
            edges = edges + p.h_edge;
            edges = edges + p.v_edge;

        }

        num_of_edges = edges;
    }


    void graph_object::show(){

            cv::imshow("Image Display", graph);
            cv::waitKey(0);

    }


    cv::Mat graph_object::return_graph(){

        cv::Mat mask;
        cv::cvtColor(graph, mask, cv::COLOR_BGR2GRAY);
        return mask;

    }


    void add_mask(cv::Mat &input,cv::Mat &mask,cv::Rect &ROI){

        if (mask.type() != CV_8UC1 or input.type() != CV_8UC1) {
            throw std::runtime_error("Error: Masks must be single-channel 8-bit (CV_8UC1)\n");
        }

        mask.copyTo(input(ROI),mask);

    }


    std::vector<cv::Mat> extract_components(const cv::Mat& inputMask) {

        CV_Assert(!inputMask.empty() && inputMask.type() == CV_8U);
        cv::Mat labels;

        int num_objects = cv::connectedComponents(inputMask, labels, 8, CV_32S);
        std::vector<cv::Mat> components;

        for (int label = 1; label < num_objects; ++label) {
            cv::Mat component = cv::Mat::zeros(inputMask.size(), CV_8U);
            //Set pixels matching the current label to 255
            component.setTo(255, labels == label);
            components.push_back(component);
        }

        return components;
    }


    cv::Mat graph_object::find_contour(const cv::Mat& inputMask,int thickness) {

        CV_Assert(!inputMask.empty() && inputMask.type() == CV_8U);

        cv::Mat padded_inputMask;
        const int padding = thickness + 1;
        cv::copyMakeBorder(inputMask, padded_inputMask,
                        padding, padding, padding, padding,
                        cv::BORDER_CONSTANT, cv::Scalar(0));
        cv::Mat eroded;
        cv::erode(padded_inputMask, eroded, cv::Mat(), cv::Point(-1, -1), thickness);

        cv::Mat cont = padded_inputMask - eroded;

        return cont(cv::Rect(padding, padding,
                    inputMask.cols, inputMask.rows));

    }


}
