
#include "_image.h"



namespace img {


    images::images(std::vector<std::string> files){
        f_list = files;
    };


    void images::add_images(std::vector<std::string>& files){

        std::unordered_set<std::string> files_set(f_list.begin(), f_list.end());

        for(std::string & i : files){

            if(files_set.count(i) == 0){
                f_list.push_back(i);
            }

        }

    }


    void images::load_resized(int max_size){

        std::unordered_set<std::string> setloaded(loaded.begin(), loaded.end());
        std::unordered_set<std::string> settoload(f_list.begin(), f_list.end());

        for (const auto& element : setloaded) {
            settoload.erase(element);
        }

        std::vector<std::string> temp_list = std::vector<std::string>(settoload.begin(), settoload.end());

        for(auto& elem : temp_list) {
            std::cout<<"\n"<< elem<<"\n";
            loaded.push_back(elem);
            cv::Mat temp_img = imgm::file_to_cv(elem);

            if((temp_img.rows < 300) or (temp_img.cols < 300)or (max_size < 300)){

                throw std::runtime_error("Error: Image size too small (img.width < 300 or img.height < 300)");

            }

            std::cout << temp_img.size();
            cv::Mat temp_res = temp_img;

            if(std::max(temp_img.rows,temp_img.cols) > max_size){

                if(temp_img.cols >= temp_img.rows){

                    temp_res = imgm::resizeKeepAspectRatio(temp_img, max_size);

                }else{

                    float resize = (max_size * temp_img.cols) / temp_img.rows;
                    temp_res = imgm::resizeKeepAspectRatio(temp_img, resize);

                }

            }

            img_data.push_back( temp_res );

        }

    }


    std::vector<cv::Mat> images::load_connected_images(const std::vector<bool> &load){

            std::vector<cv::Mat> images;
            for(int i = 0;i < loaded.size();i++) {
                if(load[i]){
                    images.push_back( imgm::file_to_cv(loaded[i]) );

                }else{
                    images.push_back(cv::Mat());

                }
            }

            return images;

    }

    void images::clear_images(){

        std::vector<cv::Mat>().swap(img_data);

    }

    std::vector<cv::Mat>* images::get_image_addresses(){return &img_data;}

    std::vector<std::string> images::get_f_list(){return f_list;}

    std::vector<util::keypoints> images::get_keypoints(){return keypnts;}

    void images::calculate_keypoints(int threads,util::match_conf* conf,std::atomic<double> * frct,std::atomic<bool> * cancel){


        std::vector<util::keypoints> kpmat;
        kpmat.resize( img_data.size() );
        std::vector<int> idx( img_data.size() );
        std::iota( idx.begin(), idx.end(), 0 );
        std::vector<std::vector<util::keypoints>> split_kp;

        std::vector<std::vector<int>> split_id = util::splitVector(idx, threads);

        std::vector<std::future<std::vector<util::keypoints>>> kp;
        kp.resize(threads);
/*
        for (int i = 0;i<threads;i++){

            kp[i] = std::async(util::extrace_kp_vector,std::ref(img_data),split_id[i]);

        }
*/
        for (int i = 0; i < threads; i++) {
            kp[i] = std::async(std::launch::async, [this,split_id,i,frct,threads,cancel,conf]() {

                std::vector<util::keypoints> result;

                if(cancel and cancel->load()){
                    return result;
                }

                result = util::extrace_kp_vector(std::ref(img_data), split_id[i],conf);

                if(not (frct == NULL)){

                    double add =  1/(6 * (double)threads);
                    frct->fetch_add(add, std::memory_order_relaxed);

                }

                return result;
            });
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
        //std::cout <<" im c " <<im_center[i][0]<<"\n";
        float h_size = img_data[i].cols / 2 + im_center[i][0];
        float v_size = img_data[i].rows / 2;
        img_data[i] = imgm::project(img_data[i],h_size,v_size,(float)f[i]);

    }

}

}
