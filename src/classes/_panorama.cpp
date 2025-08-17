
#ifndef PAN_H
#define PAN_H

#include "_panorama.h"

namespace pan{


    void stitch_parameters::set_config(struct config& conf){


        if(conf.gain_compensation){

            gain = gain::gain_compensation(owned_res.imgs,owned_res.adj,owned_res.corners);

        }

        if(conf.cut){

            mask_cut = gcut::graph_cut(owned_res.imgs,owned_res.masks,owned_res.corners,owned_res.ord);

        }


    }


    cv::Mat stitch_parameters::get_preview(struct config& conf){

        struct blend_data temp;

        for(int i = 0;i < conf.use.size(); i++){


            if(conf.gain_compensation){

                temp.imgs.push_back(owned_res.imgs[i] / gain[i]);

            }else{

                temp.imgs.push_back(owned_res.imgs[i]);

            }

            temp.corners.push_back(owned_res.corners[i]);
            temp.msks.push_back(owned_res.masks[i]);
            if(conf.cut){temp.msks_cut.push_back(mask_cut[i]);}
            temp.ord.push_back(owned_res.ord[i]);

        }

        cv::Mat preview = blend(temp,conf);
        return preview;

    }


    cv::Mat stitch_parameters::blend(struct blend_data& temp,struct config& conf){

        cv::Mat blend;
        switch(conf.blend){

            case NO_BLEND:

                if(conf.cut){
                    blend = blnd::no_blend(temp.imgs,temp.msks_cut,temp.corners);
                }else{
                    blend = blnd::no_blend(temp.imgs,temp.msks,temp.corners);
                }

                return blend;

            case SIMPLE_BLEND:

                blend = blnd::simple_blend(temp.imgs,temp.msks,temp.corners);
                return blend;

            case MULTI_BLEND:

                blend = blnd::multi_blend(temp.imgs,temp.msks_cut,temp.msks,temp.corners,3,5);
                return blend;

        }

        cv::Mat blank;
        return blank;

    }


    cv::Mat stitch_parameters::return_full(struct config& conf){

        std::vector<Eigen::MatrixXd> k_scaled = owned_res.K;
        struct stch::stitch_data ret;

        {
        std::vector<cv::Mat> images = pan_address->load_connected_images(owned_res.connectivity);
        for(int i = 0;i < owned_res.K.size();i++){

                double ratio = (double)images[i].size[0]/(double)owned_res.imgs[i].size[0];

                Eigen::Matrix3d scale;
                scale << ratio,0,0,0,ratio,0,0,0,1;

                k_scaled[i] = scale * owned_res.K[i];

            }

        ret = stch::get_proj_parameters(images,owned_res.rot,k_scaled,owned_res.maxLoc,owned_res.connectivity);
        }

        std::vector<cv::Mat> masks_full = std::vector<cv::Mat>(owned_res.masks.size());
        std::vector<cv::Mat> masks_full_cut = std::vector<cv::Mat>(mask_cut.size());

        for(int i = 0;i < owned_res.masks.size();i++){

            cv::resize(owned_res.masks[i], masks_full[i],ret.imgs[i].size());

        }

        for(int i = 0;i < mask_cut.size();i++){

            cv::resize(mask_cut[i], masks_full_cut[i],ret.imgs[i].size());

        }


/*

        if(cut){

            std::vector<cv::Mat> tmp = gcut::graph_cut(owned_res.imgs,owned_res.masks,owned_res.corners,owned_res.ord);

            for(int i = 0;i < owned_res.K.size();i++){

                cv::resize(tmp[owned_res.ind[i]], mask_resized[owned_res.ord[i]],ret.imgs[owned_res.ind[i]].size());
                cv::resize(owned_res.masks[owned_res.ord[i]], mask_f[owned_res.ord[i]],ret.imgs[owned_res.ind[i]].size());

            }

        }else{

            for(int i = 0;i < owned_res.K.size();i++){

                cv::resize(owned_res.masks[owned_res.ord[i]], mask_resized[owned_res.ord[i]],ret.imgs[owned_res.ind[i]].size());

            }

        }

        mask_full = mask_f;
        gain = gain::gain_compensation(owned_res.imgs,owned_res.adj,owned_res.corners);

        ret.msks = mask_resized;
*/
        cv::Mat returna;
        return returna;

    }


    void panorama::load_resized(int max_size){

        for(auto& elem : f_list) {

            cv::Mat temp_img = imgm::file_to_cv(elem);

            CV_Assert(temp_img.rows >= 300);
            CV_Assert(temp_img.cols >= 300);
            CV_Assert(max_size >= 300);
            float ratio = 1;

            std::cout << temp_img.size();

            if(std::max(temp_img.rows,temp_img.cols) > max_size){

                if(temp_img.cols >= temp_img.rows){

                    temp_img = imgm::resizeKeepAspectRatio(temp_img, max_size);
                    ratio =  temp_img.cols / max_size;

                }else{

                    ratio = temp_img.rows / max_size;
                    float resize = (max_size * temp_img.cols) / temp_img.rows;
                    temp_img = imgm::resizeKeepAspectRatio(temp_img, resize);

                }


            }

            img_data.push_back( temp_img );

        }

    }

    cv::Mat panorama::get_adj(){

        return adj;

    }


    std::vector<std::vector<std::vector<cv::DMatch>>> panorama::get_match_mat(){

        return match_mat;

    }


    std::vector<std::vector< cv::Matx33f >> panorama::get_hom_mat(){

        return hom_mat;

    }


    void panorama::get_adj_par(int threads){


            class util::adj_calculator ctest(img_data,keypnts);
            ctest.get_threads(threads);

            std::vector<std::thread> thread_vector;
            std::vector<std::thread> thread_vector_match_mat;


            for(int i = 0;i < threads;i++){

                std::thread tobj(&util::adj_calculator::get_match_number_matrix,&ctest,i);
                thread_vector_match_mat.push_back(std::move(tobj));

            }

            for(int i = 0;i < threads;i++){

                thread_vector_match_mat[i].join();

            }

            ctest.heuristic_match_filter(conf_local.max_images_per_match);

            for(int i = 0;i < threads;i++){

                std::thread tobj(&util::adj_calculator::cal_adj,&ctest,img_data,i);
                thread_vector.push_back(std::move(tobj));

            }


            for(int i = 0;i < threads;i++){

                thread_vector[i].join();

            }

            adj = ctest.adj;
            hom_mat = ctest.hom_mat;
            match_mat = ctest.match_mat;

        }



    void panorama::stitch_panorama(int threads,struct config& conf){

        conf_local = conf;
        load_resized(conf.init_size);

        int threadsimg;
        if(threads > img_data.size()){
            threadsimg = img_data.size();
        }else{
            threadsimg = threads;
        }

        calculate_keypoints(threadsimg);
        get_adj_par(threadsimg);

        std::vector<struct util::adj_str> adj_string;
        adj_string = util::extract_adj(adj);
        imgm::pan_img_transform Tr(&adj_string[0],&img_data);

        Tr.focal = util::focal_from_hom(hom_mat,adj_string[0].adj);
        if(Tr.focal == -1) {Tr.focal = conf.focal;}

        imgm::calc_stitch_from_adj(Tr,hom_mat,match_mat,keypnts);


        stitched = stitch_parameters(stch::bundleadjust_stitching(Tr,hom_mat,keypnts,match_mat,threads),this);

        for(int i = 0;i<adj_string[0].connectivity.size();i++){

            if(adj_string[0].connectivity[i] > 0){
                conf_local.use.push_back(true);
            }

        }

        if (stitched.has_value()) {
            stitched->set_config(conf_local);
        }else{ throw std::invalid_argument("something went wrong"); }

        cv::Mat prev = stitched->get_preview(conf_local);

        cv::imshow("Image Display", prev);
        cv::waitKey(0);


/*




        load_images();


        struct stch::stitch_data outp = s_par.calc_projection(img_data_HD,img_data,true);

        s_par.normalize_gain(outp.imgs);

        blnd::multi_blend(outp.imgs,outp.msks,s_par.mask_full,outp.corners,3,7);
*/
    }

}
#endif
