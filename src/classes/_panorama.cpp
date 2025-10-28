
#include "_panorama.h"

namespace pan{

    const char* BlendingToString(int value) {
    switch (value) {
        #define X(name, value) case value: return #name;
        BLENDING_ENUM
        #undef X
        default: return "SIMPLE_BLEND";
        }

    }

    int StringToBlending(const std::string& str) {
        #define X(name, value) if (str == #name) return value;
        BLENDING_ENUM
        #undef X

        throw std::invalid_argument("Invalid Blending string: " + str);
    }

    const char* ProjectionToString(int value) {
    switch (value) {
        #define X(name, value) case value: return #name;
        PROJECTION_ENUM
        #undef X
        default: return "SPHERICAL";
        }

    }

    int StringToProjection(const std::string& str) {
        #define X(name, value) if (str == #name) return value;
        PROJECTION_ENUM
        #undef X

        throw std::invalid_argument("Invalid Projection string: " + str);
    }


    const char* StretchToString(int value) {
    switch (value) {
        #define X(name, value) case value: return #name;
        STRETCHING_ENUM
        #undef X
        default: return "QUADRATIC";
        }

    }

    int StringToStretch(const std::string& str) {
        #define X(name, value) if (str == #name) return value;
        STRETCHING_ENUM
        #undef X

        throw std::invalid_argument("Invalid Stretch string: " + str);
    }


    void correct_img(std::vector<cv::Mat>& mask_cut,std::vector<cv::Mat>& imgs){

        for(int i = 0;i < mask_cut.size();i++){

            cv::Mat cop;
            imgs[i].copyTo(cop,mask_cut[i]);
            imgs[i] = cop;

        }

    }


    void stitch_parameters::set_config(struct config& conf,std::atomic<bool> *cancel_var){

        if(conf.straighten){

            owned_res.rot = strg::straightenPanorama(owned_res.rot);

        }

        const double focal = owned_res.K[owned_res.maxLoc](0, 0);  // Focal length from first camera

        switch(conf.proj) {
            case SPHERICAL:{
                proj::spherical_proj projector_sp(focal);
                owned_res.proj = std::make_shared<proj::spherical_proj>(projector_sp);
            }break;
            case STEREOGRAPHIC:{
                proj::sten_proj projector_st(focal);
                owned_res.proj = std::make_shared<proj::sten_proj>(projector_st);
            }break;
            case CYLINDRICAL:{
                proj::cylindrical_proj projector_st(focal);
                owned_res.proj = std::make_shared<proj::cylindrical_proj>(projector_st);
            }break;

        }

        struct proj::proj_data PRD = proj::get_proj_parameters(owned_res.imgs,owned_res.rot,owned_res.K,owned_res.connectivity,owned_res.proj);

        if(conf.fix_center and (conf.proj == STEREOGRAPHIC)){

            std::shared_ptr<proj::sten_proj> recast = std::static_pointer_cast<proj::sten_proj>(owned_res.proj);

            std::pair<cv::Point,float> prev_est = recast->estimate_circle(PRD);
            owned_res.ansatz_prev = prev_est.first;
            owned_res.radius_prev = prev_est.second;

            bool quad;
            if(conf.stretching == QUADRATIC_SCALING){
                quad = true;
            }else{
                quad = false;
            }

            if(recast->has_circle == true){
                owned_res.has_circle = true;
                recast->disk_reproj(PRD,quad);

            }

        }

        owned_res.imgs = PRD.imgs;
        owned_res.masks = PRD.msks;
        owned_res.corners = PRD.corners;
        owned_res.prev_size = util::get_pan_dimension(owned_res.corners,owned_res.imgs);

        if(conf.blend_intensity){
            owned_res.intensity_correction = test::equalizeIntensities(owned_res.imgs,owned_res.masks,owned_res.corners);
        }

        std::atomic<double>* f_adress = NULL;
        if(not (progress == NULL)){progress->bar_text("Blending Images...");

            f_adress = &(progress->fraction);

        }

        if(conf.gain_compensation){

            gain = gain::gain_compensation(owned_res.imgs,owned_res.adj,owned_res.corners);

        }

        if(conf.cut){

            mask_cut = gcut::graph_cut(owned_res.imgs,owned_res.masks,owned_res.corners,owned_res.ord,f_adress,cancel_var);

        }else if((conf.blend == MULTI_BLEND) or (conf.cut_seams == TRUE)){

            mask_cut = dcut::dist_cut(owned_res.masks,owned_res.corners);

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

            //correct_img(temp.msks,temp.imgs);

            temp.corners.push_back(owned_res.corners[i]);
            temp.msks.push_back(owned_res.masks[i]);
            if(conf.cut or conf.cut_seams){temp.msks_cut.push_back(mask_cut[i]);}
            temp.ord.push_back(owned_res.ord[i]);

        }

        if(conf.blend_intensity){
            test::adjust_intensity(temp.imgs,owned_res.intensity_correction);
        }

        cv::Mat preview = blend(temp,conf);

        if((conf.proj == STEREOGRAPHIC) and (conf.fix_center == true) and (owned_res.has_circle == true)){

            std::shared_ptr<proj::sten_proj> recast = std::static_pointer_cast<proj::sten_proj>(owned_res.proj);
            recast->inpaint(preview,owned_res.ansatz_prev,owned_res.radius_prev);

        }

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

                if(conf.cut or conf.cut_seams){

                    blend = blnd::no_blend(temp.imgs,temp.msks_cut,temp.corners);

                }else{
                    blend = blnd::no_blend(temp.imgs,temp.msks,temp.corners);
                }

                return blend;

            case SIMPLE_BLEND:

                blend = blnd::simple_blend(temp.imgs,temp.msks,temp.corners);
                return blend;

            case MULTI_BLEND:

                blend = blnd::multi_blend(temp.imgs,temp.msks_cut,temp.msks,temp.corners,conf.bands,conf.sigma_blend);
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
        struct proj::proj_data ret;
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

                const double focal = k_scaled[owned_res.maxLoc](0, 0);  // Focal length from first camera
                owned_res.proj->change_focal(focal);
                ret = proj::get_proj_parameters(images,owned_res.rot,k_scaled,owned_res.connectivity,owned_res.proj);

                if(conf.fix_center and (conf.proj == STEREOGRAPHIC)){

                    std::shared_ptr<proj::sten_proj> recast = std::static_pointer_cast<proj::sten_proj>(owned_res.proj);
                    std::pair<cv::Point,float> prev_est = recast->estimate_circle(ret);
                    owned_res.ansatz = prev_est.first;
                    owned_res.radius = prev_est.second;
                    bool quad;
                    if(conf.stretching == QUADRATIC_SCALING){
                        quad = true;
                    }else{
                        quad = false;
                    }

                    if(recast->has_circle == true){

                        recast->disk_reproj(ret,quad);

                    }

                }

                blend_dat.imgs = ret.imgs;
                blend_dat.corners = ret.corners;
                blend_dat.msks = ret.msks;

             }

        }

        if(conf.gain_compensation){
            for(int i = 0;i < conf.use.size(); i++){

                    blend_dat.imgs[i] = blend_dat.imgs[i] / gain[i];

            }
        }

        blend_dat.msks_cut = std::vector<cv::Mat>(mask_cut.size());

        for(int i = 0;i < mask_cut.size();i++){

            cv::resize(mask_cut[i], blend_dat.msks_cut[i],blend_dat.imgs[i].size(),cv::INTER_CUBIC );

        }

        if(conf.blend_intensity){
            test::adjust_intensity(blend_dat.imgs,owned_res.intensity_correction);
        }
        //correct_img(blend_dat.msks,blend_dat.imgs);

        cv::Mat full = blend(blend_dat,conf);

        if((conf.proj == STEREOGRAPHIC) and (conf.fix_center == true) and (owned_res.has_circle == true)){

            std::shared_ptr<proj::sten_proj> recast = std::static_pointer_cast<proj::sten_proj>(owned_res.proj);

            recast->inpaint(full,owned_res.ansatz,owned_res.radius);

        }

        return full;

    }


    void panorama::cancel(){

        cancel_var = true;

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

            class util::adj_calculator ctest(img_data,keypnts,&conf_m,f_adress,&cancel_var);
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

            if(cancel_var){return;}

            ctest.heuristic_match_filter(conf_local.max_images_per_match);

            for(int i = 0;i < threads;i++){

                std::thread tobj(&util::adj_calculator::cal_adj,&ctest,img_data,i);
                thread_vector.push_back(std::move(tobj));

            }


            for(int i = 0;i < threads;i++){

                thread_vector[i].join();

            }

            if(cancel_var){return;}
            adj = ctest.adj;
            hom_mat = ctest.hom_mat;
            match_mat = ctest.match_mat;

        }

//(kp match) adjust blend

    bool panorama::stitch_panorama(struct config* conf){

        conf_m.contrastThreshold = conf_local.contrastThreshold;
        conf_m.edgeThreshold = conf_local.edgeThreshold;
        conf_m.max_images_per_match = conf_local.max_images_per_match;
        conf_m.max_keypoints = conf_local.max_keypoints;
        conf_m.nfeatures = conf_local.nfeatures;
        conf_m.nOctaveLayers = conf_local.nOctaveLayers;
        conf_m.sigma_sift = conf_local.sigma_sift;
        conf_m.RANSAC_iterations = conf_local.RANSAC_iterations;
        conf_m.x_margin = conf_local.x_margin;
        conf_m.min_overlap = conf_local.min_overlap;
        conf_m.overlap_inl_match = conf_local.overlap_inl_match;
        conf_m.overlap_inl_keyp = conf_local.overlap_inl_keyp;
        conf_m.conf = conf_local.conf;


        conf_local = *conf;
        load_resized(conf_local.init_size);
        int threads = conf_local.threads;
        int threadsimg;
        if(threads > img_data.size()){
            threadsimg = img_data.size();
        }else{
            threadsimg = threads;
        }

        if(not (progress == NULL)){

            progress->bar_text("Finding Keypoints...");
            calculate_keypoints(threadsimg,&conf_m,&(progress->fraction),&cancel_var);

        }else{

            calculate_keypoints(threadsimg,&conf_m);

        }

        if(cancel_var){return false;}

        if(not (progress == NULL)){
            progress->bar_text("Matching Images...");
        }
        get_adj_par(threadsimg);

        if(cancel_var){return false;}

        std::vector<struct util::adj_str> adj_string;
        std::cout <<adj;

        adj_string = util::extract_adj(adj);
        imgm::pan_img_transform Tr(&adj_string[0],&img_data);

        Tr.focal = util::focal_from_hom(hom_mat,adj_string[0].adj);
        if(Tr.focal == -1) {Tr.focal = conf_local.focal;}


        conf_local.result = {adj_string[0].nodes,adj_string[0].adj.cols};
        for(int f = 0;f < adj_string[0].connectivity.size();f++){

            if(adj_string[0].connectivity[f] > 0){
                conf_local.img_list.push_back(f_list[f]);
            }

        }


        imgm::calc_stitch_from_adj(Tr,hom_mat,match_mat,keypnts);
        Tr.fast = conf_local.fast;

        if(not (progress == NULL)){
            progress->bar_text("Adjusting Images...");
            stitched = stitch_parameters(stch::bundleadjust_stitching(Tr,hom_mat,keypnts,match_mat,conf_local.lambda,threads,&(progress->fraction),&cancel_var),this,progress);
        }else{

            stitched = stitch_parameters(stch::bundleadjust_stitching(Tr,hom_mat,keypnts,match_mat,conf_local.lambda,threads),this);

        }


        if(cancel_var){return false;}

        for(int i = 0;i<adj_string[0].connectivity.size();i++){

            if(adj_string[0].connectivity[i] > 0){
                conf_local.use.push_back(true);
            }

        }

        if(cancel_var){return false;}

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
            cv::Rect full = cv::Rect(0,0,panorama_full.cols,panorama_full.rows);
            ROI = full bitand ROI;

            return panorama_full(ROI);
        }

        return panorama_full;

    }


    void panorama::test(struct config* conf){

        conf_m.contrastThreshold = conf_local.contrastThreshold;
        conf_m.edgeThreshold = conf_local.edgeThreshold;
        conf_m.max_images_per_match = conf_local.max_images_per_match;
        conf_m.max_keypoints = conf_local.max_keypoints;
        conf_m.nfeatures = conf_local.nfeatures;
        conf_m.nOctaveLayers = conf_local.nOctaveLayers;
        conf_m.sigma_sift = conf_local.sigma_sift;
        conf_m.RANSAC_iterations = conf_local.RANSAC_iterations;
        conf_m.x_margin = conf_local.x_margin;


        conf_local = *conf;
        load_resized(conf_local.init_size);
        int threads = 8;
        int threadsimg;
        if(threads > img_data.size()){
            threadsimg = img_data.size();
        }else{
            threadsimg = threads;
        }

        calculate_keypoints(threadsimg,&conf_m);


        class util::adj_calculator ctest(img_data,keypnts,&conf_m);
        ctest.get_threads(1);


        ctest.get_match_number_matrix(0);

        ctest.heuristic_match_filter(conf_local.max_images_per_match);

        ctest.cal_adj(img_data,0);


    }


}
