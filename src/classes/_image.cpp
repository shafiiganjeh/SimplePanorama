
#include "_image.h"



namespace img {


    images::images(std::vector<std::string> files){
        f_list = files;
    };


    std::vector<cv::Mat> images::load_connected_images(const std::vector<bool> &load){

            std::vector<cv::Mat> images;
            for(int i = 0;i < f_list.size();i++) {
                if(load[i]){images.push_back( imgm::file_to_cv(f_list[i]) );}
            }

            return images;

    }

    void images::clear_images(){

        std::vector<cv::Mat> empty;

        img_data = empty;

    }

    std::vector<cv::Mat> images::get_images(){return img_data;}

    std::vector<std::string> images::get_f_list(){return f_list;}

    std::vector<util::keypoints> images::get_keypoints(){return keypnts;}

    void images::calculate_keypoints(int threads){


        std::vector<util::keypoints> kpmat;
        kpmat.resize( img_data.size() );
        std::vector<int> idx( img_data.size() );
        std::iota( idx.begin(), idx.end(), 0 );
        std::vector<std::vector<util::keypoints>> split_kp;

        std::vector<std::vector<int>> split_id = util::splitVector(idx, threads);

        std::vector<std::future<std::vector<util::keypoints>>> kp;
        kp.resize(threads);

        for (int i = 0;i<threads;i++){
            kp[i] = std::async(util::extrace_kp_vector,std::ref(img_data),split_id[i]);
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

        float h_size = img_data[i].cols / 2;
        float v_size = img_data[i].rows / 2;
        img_data[i] = imgm::project(img_data[i],h_size,v_size,f);

    }

}


void images::images_to_cylinder(std::vector<double> f , std::vector<cv::Vec2f> im_center){

    for(int i = 0;i<img_data.size();i++){
        std::cout <<" im c " <<im_center[i][0]<<"\n";
        float h_size = img_data[i].cols / 2 + im_center[i][0];
        float v_size = img_data[i].rows / 2;
        img_data[i] = imgm::project(img_data[i],h_size,v_size,(float)f[i]);

    }


}

}
