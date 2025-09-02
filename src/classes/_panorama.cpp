
#ifndef PAN_H
#define PAN_H

#include "_panorama.h"

namespace pan{


    void stitch_parameters::set_config(struct config& conf){

        std::atomic<double>* f_adress = NULL;
        if(not (progress == NULL)){progress->bar_text("Blending Images...");

            f_adress = &(progress->fraction);

        }

        if(conf.gain_compensation){

            gain = gain::gain_compensation(owned_res.imgs,owned_res.adj,owned_res.corners);

        }

        if(conf.cut){

            mask_cut = gcut::graph_cut(owned_res.imgs,owned_res.masks,owned_res.corners,owned_res.ord,f_adress);

        }


    }


    cv::Mat stitch_parameters::get_preview(struct config& conf){

        struct blend_data temp;
        temp.imgs.reserve(conf.use.size());
        temp.corners.reserve(conf.use.size());
        temp.msks.reserve(conf.use.size());
        temp.msks_cut.reserve(conf.use.size());
        temp.ord.reserve(conf.use.size());

        for(int i = 0;i < conf.use.size(); i++){


            if(conf.gain_compensation){

                cv::Mat compensated = owned_res.imgs[i] / gain[i];
                temp.imgs.push_back(std::move(compensated));

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


    cv::Size stitch_parameters::get_preview_size(struct config& conf){

        struct util::size_data p_size = util::get_pan_dimension(owned_res.corners,owned_res.imgs);

        return p_size.dims;
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
                blend = blend * 255;
                blend.convertTo(blend, CV_8UC3);

                return blend;

        }

        cv::Mat blank;
        return blank;

    }


    cv::Mat stitch_parameters::return_full(struct config& conf){

        std::vector<Eigen::MatrixXd> k_scaled = owned_res.K;
        struct blend_data blend_dat;
        blend_dat.ord = owned_res.ord;

        {
        struct stch::stitch_data ret;
            {

            std::vector<cv::Mat> images = pan_address->load_connected_images(owned_res.connectivity);
            std::vector<cv::Mat>* img_addresses = pan_address->get_image_addresses();

            for(int i = 0; i < owned_res.connectivity.size();i++){

                if(owned_res.connectivity[i] > 0){

                    double ratio = (double)images[i].size[0]/(double)(*img_addresses)[i].size[0];

                    Eigen::Matrix3d scale;
                    scale << ratio,0,0,0,ratio,0,0,0,1;
                    k_scaled[i] = scale * owned_res.K[i];

                }

            }

                ret = stch::get_proj_parameters(images,owned_res.rot,k_scaled,owned_res.maxLoc,owned_res.connectivity);
                blend_dat.imgs = ret.imgs;
                blend_dat.corners = ret.corners;

             }


        }

        if(conf.gain_compensation){
            for(int i = 0;i < conf.use.size(); i++){

                    blend_dat.imgs[i] = blend_dat.imgs[i] / gain[i];

            }
        }

        blend_dat.msks = std::vector<cv::Mat>(owned_res.masks.size());
        blend_dat.msks_cut = std::vector<cv::Mat>(mask_cut.size());

        for(int i = 0;i < owned_res.masks.size();i++){

            cv::resize(owned_res.masks[i], blend_dat.msks[i],blend_dat.imgs[i].size(),cv::INTER_CUBIC );

        }

        for(int i = 0;i < mask_cut.size();i++){

            cv::resize(mask_cut[i], blend_dat.msks_cut[i],blend_dat.imgs[i].size(),cv::INTER_CUBIC );

        }


        cv::Mat full = blend(blend_dat,conf);
        return full;

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

            std::atomic<double>* f_adress = NULL;
            if(not (progress == NULL)){

                f_adress = &(progress->fraction);
            }


            class util::adj_calculator ctest(img_data,keypnts,f_adress);
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

//(kp match) adjust blend

    bool panorama::stitch_panorama(struct config& conf){

        conf_local = conf;
        load_resized(conf.init_size);
        int threads = conf_local.threads;
        int threadsimg;
        if(threads > img_data.size()){
            threadsimg = img_data.size();
        }else{
            threadsimg = threads;
        }

        if(not (progress == NULL)){

            progress->bar_text("Finding Keypoints...");
            calculate_keypoints(threadsimg,&(progress->fraction));

        }else{

            calculate_keypoints(threadsimg);

        }

        if(not (progress == NULL)){
            progress->bar_text("Matching Images...");
        }
        get_adj_par(threadsimg);

        std::vector<struct util::adj_str> adj_string;
        std::cout <<adj;

        adj_string = util::extract_adj(adj);
        imgm::pan_img_transform Tr(&adj_string[0],&img_data);

        Tr.focal = util::focal_from_hom(hom_mat,adj_string[0].adj);
        if(Tr.focal == -1) {Tr.focal = conf.focal;}

        imgm::calc_stitch_from_adj(Tr,hom_mat,match_mat,keypnts);

        if(not (progress == NULL)){
            progress->bar_text("Adjusting Images...");
            stitched = stitch_parameters(stch::bundleadjust_stitching(Tr,hom_mat,keypnts,match_mat,threads,&(progress->fraction)),this,progress);
        }else{

            stitched = stitch_parameters(stch::bundleadjust_stitching(Tr,hom_mat,keypnts,match_mat,threads),this);

        }



        for(int i = 0;i<adj_string[0].connectivity.size();i++){

            if(adj_string[0].connectivity[i] > 0){
                conf_local.use.push_back(true);
            }

        }

        if (stitched.has_value()) {
            stitched->set_config(conf_local);
        }else{ throw std::invalid_argument("something went wrong"); return false;}

        return true;
    }


    cv::Mat panorama::get_preview(){

        return stitched->get_preview(conf_local);

    }


    cv::Mat panorama::get_panorama(cv::Rect ROI){

        if(!pan_exist){

            pan_exist = true;
            panorama_full = stitched->return_full(conf_local);

        }

        if(ROI.area() > 0){

            cv::Size p_size_small = stitched->get_preview_size(conf_local);
            double ratio = ((double)panorama_full.cols/(double)p_size_small.width + (double)panorama_full.rows/(double)p_size_small.height)/2;
            ROI = util::scaleRect(ROI,ratio,ratio);

            ROI.x = std::max(0,ROI.x);
            ROI.y = std::max(0,ROI.y);
            ROI.width = std::min(panorama_full.cols,ROI.width);
            ROI.height = std::min(panorama_full.rows,ROI.height);

            return panorama_full(ROI);
        }

        return panorama_full;

    }

}
#endif
