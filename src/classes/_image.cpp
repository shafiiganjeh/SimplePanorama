

#include "_image.h"


namespace img {

    images::images(std::vector<std::string> files):f_list(files){};


    void images::load_images(){


            for(auto& elem : f_list) {

                img_data.push_back( imgm::file_to_cv(elem) );

            }
    }

    void images::clear_images(){

        std::vector<cv::Mat> empty;

        img_data = empty;

    }

    std::vector<cv::Mat> images::get_images(){return img_data;}

}

