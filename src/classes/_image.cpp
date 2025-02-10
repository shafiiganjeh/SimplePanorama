
#include "_image.h"



namespace img {


    images::images(std::vector<std::string> files){
        f_list = files;
    };


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

    std::vector<std::string> images::get_f_list(){return f_list;}

    std::vector<maths::keypoints> images::get_keypoints(){return keypnts;}

    void images::calculate_keypoints(int threads){

        std::vector<maths::keypoints> kpmat;
        kpmat.resize( img_data.size() );
        std::vector<int> idx( img_data.size() );
        std::iota( idx.begin(), idx.end(), 0 );
        std::vector<std::vector<maths::keypoints>> split_kp;

        std::vector<std::vector<int>> split_id = maths::splitVector(idx, threads);

        std::vector<std::future<std::vector<maths::keypoints>>> kp;
        kp.resize(threads);

        for (int i = 0;i<threads;i++){
            kp[i] = std::async(maths::extrace_kp_vector,img_data,split_id[i]);
        }

        for (int i = 0;i<threads;i++){
            kp[i].wait();
        }

        for (int i = 0;i < split_id.size();i++){
            split_kp.push_back(kp[i].get());

        }

        for (int i = 0;i < split_id.size();i++){

            for (int j = 0;j < split_id[i].size();j++){

                kpmat[split_id[i][j]] = split_kp[i][j];

            }

        }

        keypnts = kpmat;
    }

void images::images_to_cylinder(float f){

    for(int i = 0;i<img_data.size();i++){

        img_data[i] = imgm::project(img_data[i],400.0,533.0,f);

    }

}


}


