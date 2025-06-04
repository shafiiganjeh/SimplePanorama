
#include "_stitch.h"

namespace stch {

/*algorithm for stitching:
 * set n = 3
 * find n best matches and adjust for n simultaneously.
 *
 * for every new image calculate the best match to the already adjusted pairs.
 * if the new image matches badly
 * if all options are bad remove the image.
 *
*/

std::vector<double> computeRowSumDividedByZeroCount(const cv::Mat& mat) {

    const int rows = mat.rows;
    const int cols = mat.cols;
    std::vector<double> results;
    results.reserve(rows);

    for (int i = 0; i < rows; ++i) {
        const cv::Mat row = mat.row(i);
        const double row_sum = cv::sum(row)[0];
        const int zero_count = cols - cv::countNonZero(row);
        results.push_back(row_sum / zero_count);
    }

    return results;
}


std::vector<int> getTopNonZeroIndices(const cv::Mat& M, int r, int n) {
    std::vector<std::pair<double, int>> elements;

    for (int c = 0; c < M.cols; ++c) {
        const double val = M.at<double>(r, c);
        if (val > 0) {
            elements.emplace_back(val, c);
        }
    }

    std::vector<int> idx;

    // Return all non-zero indices if <= n exist
    if (elements.size() <= static_cast<size_t>(n)) {
        idx.reserve(elements.size());
        for (const auto& elem : elements) {
            idx.push_back(elem.second);
        }
    }

    else {

        std::sort(elements.begin(), elements.end(),
                  [](const auto& a, const auto& b) { return a.first > b.first; });

        idx.reserve(n);
        for (int i = 0; i < n; ++i) {
            idx.push_back(elements[i].second);
        }
    }

    return idx;
}


Eigen::MatrixXd approximate(const cv::Matx33f &hom,Eigen::MatrixXd K){

    Eigen::MatrixXd H;
    cv::Matx33d homd = static_cast<cv::Matx33d>(hom);
    cv::cv2eigen(homd,H);
    H = H/H(2,2);

    Eigen::MatrixXd KRK = K.inverse() * H * K;
    Eigen::MatrixXd R_approx = ( KRK * KRK.transpose() ).pow(0.5) * (KRK.transpose()).inverse();

    return R_approx;
}



adjust_par prep_opt(const class imgm::pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >>& Hom_mat,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,const std::vector<maths::keypoints> &kp,const std::unordered_set<int> &exists,std::vector<int> indx,int ref,int attach){

    cv::Mat adj_new(indx.size(), indx.size(), T.adj->type());
    std::sort(indx.begin(), indx.end());
    std::vector<maths::keypoints> kp_new;
    std::vector<std::vector<std::vector<cv::DMatch>>> new_match_mat(indx.size(),std::vector<std::vector<cv::DMatch>>(indx.size()));

    for (int i = 0; i < indx.size(); ++i) {
        for (int j = 0; j < indx.size(); ++j) {

            adj_new.at<double>(i,j) = T.adj->at<double>(indx[i],indx[j]);
            new_match_mat[i][j] = match_mat[indx[i]][indx[j]];

        }
    }

    adjust_par ret(&adj_new,T.img_address);
    ret.adj = adj_new;
    ret.T.adj = &ret.adj;
    ret.ref = ref;
    ret.attatch = attach;
    ret.T.focal = T.K[ref](0,0);
    std::map<int, int> ind2numb;
    std::map<int, int> numb2ind;

    for (int i = 0; i < indx.size(); ++i) {

        kp_new.push_back(kp[indx[i]]);
        ind2numb[i] = indx[i];
        numb2ind[indx[i]] = i;
        if(exists.count(indx[i]) > 0){

            ret.T.rot.push_back(Eigen::MatrixXd::Identity(3, 3));
            ret.T.K.push_back(T.K[ref]);

        }else{

            if(T.rot[attach].isIdentity(1e-6)){
                Eigen::MatrixXd r_approx = approximate(Hom_mat[attach][indx[i]],T.K[attach]);
                ret.T.rot.push_back(r_approx);

            }else{
                ret.T.rot.push_back(T.rot[attach]);

            }
            ret.T.K.push_back(T.K[attach]);


        }

    }
    ret.ind2numb = ind2numb;
    ret.numb2ind = numb2ind;

    ret.kp = kp_new;
    ret.match = new_match_mat;

    return ret;
}


std::vector<float> angels(const Eigen::MatrixXd &R){

    float x = atan2(R(2,1),R(2,2)) ;
    float y = atan2(-R(2,0),sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2))) ;
    float z = atan2(R(1,0),R(0,0)) ;

    std::vector<float> ang = {x,y,z};
    return ang;
}


void bundleadjust_stitching(class imgm::pan_img_transform &T,class imgm::pan_img_transform &Tnew,const std::vector<std::vector< cv::Matx33f >> &Hom_mat,const std::vector<maths::keypoints> &kpold,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,int threads){

    cv::Mat path_mat = (*T.adj) + (*T.adj).t();
    std::vector<std::vector< cv::Matx33f >> Hom_mat_new(Hom_mat);
    std::vector<maths::keypoints> kp = kpold;
    std::vector<double> connectivity = computeRowSumDividedByZeroCount(path_mat);
    //find best node
    int maxLoc = std::distance(connectivity.begin(),std::max_element(connectivity.begin(), connectivity.end()));
    //all neighbours
    std::vector<std::pair<int, std::vector<int>>> tree = maths::bfs_ordered_with_neighbors(path_mat, maxLoc);

    // find n best matches sort the rest by stitching order

    std::map<int, int> tofrom;
    std::map<int, struct bundle_par> tf_par;

        for(int i = 0;i < T.pair_stitch.size();i++){

            for(int j = 0 ; j < T.pair_stitch[i].size();j=j + 2){

                if(tofrom.count(T.pair_stitch[i][j+1]) < 1){
                   std::cout <<"\n"<<"from: "<<T.pair_stitch[i][j]<<" to: "<<T.pair_stitch[i][j+1]<<"\n";
                   tofrom[T.pair_stitch[i][j+1]] = T.pair_stitch[i][j];
                }
            }
        }

        std::cout <<"\n"<<"sorted: "<<"\n";
        int size = tofrom.size() + 1;
        std::unordered_set<int> calcs;
        calcs.insert(maxLoc);

        std::cout <<"\n"<<"adj: "<<"\n"<<(*T.adj)<<"\n";
        std::vector<int> kvec;
        std::vector<int> vvec;

        //find n best neighbours of maxLoc
        int n = 2;
        std::vector<int> ind = getTopNonZeroIndices(path_mat, maxLoc, n);



        ind.push_back(maxLoc); //<-best matching image
        std::map<int,int> testtt;

        std::unordered_set<int> exists;
        exists.insert(maxLoc);
        std::unordered_set<int> nearest;
        std::map<int, int>::iterator it;

        //std::sort(ind.begin(), ind.end());
        for(int i = 0;i < ind.size();i++){
            nearest.insert(ind[i]);
            std::cout <<"\n"<<"ind[i]: "<<"\n"<<ind[i]<<"\n";
        }

        while(calcs.size() < size){

            for (const auto& elem: calcs) {

                if(tofrom.count(elem) > 0){

                    it = tofrom.find(elem);
                    tofrom.erase(it);

                }

            }
            int c = 0;
            for ( const auto &[key, value]: tofrom ) {

                if(calcs.count(value) > 0){
                    //bool approx = false;

                    calcs.insert(key);

                    if(nearest.count(key) == 0){
                        kvec.push_back(key);
                        vvec.push_back(value);
                        std::cout <<"\n"<<"from: "<<value<<" to: "<<key<<"\n";
                    }

                }

            }

        }



        double focal = 1000;
        Eigen::MatrixXd K = Eigen::MatrixXd::Identity(3, 3);
        K(0,0) = focal;
        K(1,1) = focal;
        K(0,2) = (int)((*Tnew.img_address)[maxLoc].cols / 2);
        K(1,2) = (int)((*Tnew.img_address)[maxLoc].rows / 2);

        for(int i = 0;i < T.K.size();i++){
            T.K[i] = K;
            T.rot[i] = Eigen::MatrixXd::Identity(3, 3);
        }

        adjust_par par = prep_opt(T,Hom_mat_new,match_mat,kp ,exists,ind,maxLoc,maxLoc);

        class bundm::adjuster testad(par.kp,par.match,.0001,par.T,false,threads);
        struct bundm::inter_par teees = testad.iterate();
        double error_value = testad.error_value;

        std::vector<Eigen::MatrixXd> rret = testad.ret_rot();
        std::vector<Eigen::MatrixXd> Kret = testad.ret_K();
        Eigen::MatrixXd ref_K = Kret[par.numb2ind[maxLoc]];
        T.K[maxLoc] = Kret[par.numb2ind[maxLoc]];
        T.rot[maxLoc] = rret[par.numb2ind[maxLoc]];

        cv::Mat panorama = cv::Mat::zeros(1500,5067,CV_32FC3);
        cv::Size si = panorama.size();


            float tx=0 ;
            float ty=0 ;


        for(int i = 0;i < n+1;i++){

            std::cout <<"\n"<<"focal "<<par.ind2numb[i]<<" " <<Kret[i](0,0)<<"\n";
            std::vector<float> ang = angels(rret[i]);

            Eigen::MatrixXd rot(3,3);
            double deg = ang[1];
            double cs = cos(deg);
            double sii = sin(deg);

            rot << cs, 0, sii,
                0, 1, 0,
                -sii, 0, cs;

            cv::Matx33f TR = cv::Matx33f::eye();
            TR(0,2) = - Kret[i](0,0)*ang[1];
            TR(1,2) = 0;

            rret[i] = rot.transpose()*rret[i];

            class imgm::cylproj transformer2(rret[i],ref_K,Kret[i],(*Tnew.img_address)[par.ind2numb[i]].size(),0,+200);

            struct bundle_par push;
            push.rot_key = rret[i];
            push.k_key = Kret[i];
            push.k_val = ref_K;
            push.prev_node = maxLoc;
            push.translation = TR;
            push.current_node = par.ind2numb[i];

            tf_par[par.ind2numb[i]] = push;

            TR(0,2) = TR(0,2)-1000;
            class imgm::cylhom translate(teees.focal[i],Kret[i](0,2),Kret[i](1,2),TR,0,0);

            cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[par.ind2numb[i]], transformer2,si);

            img_tr = imgm::applyGeometricTransform(img_tr, translate,si);

            img_tr.copyTo(panorama, img_tr);

            exists.insert(par.ind2numb[i]);
            cv::imshow("Image Display",panorama);
            cv::waitKey(0);

        }


        for(int i = 0;i < kvec.size();i++){

            std::vector<int> ind;
            ind.push_back(kvec[i]);
            ind.push_back(vvec[i]);

            std::unordered_set<int> exists;
            exists.insert(vvec[i]);
            std::vector<int> ins = getTopNonZeroIndices(path_mat, maxLoc, 10);



            adjust_par par = prep_opt(T,Hom_mat_new,match_mat,kp ,exists,ind,vvec[i],vvec[i]);
            class bundm::adjuster testad(par.kp,par.match,.0001,par.T,false,threads);
            struct bundm::inter_par teees = testad.iterate();

            std::vector<Eigen::MatrixXd> it_rret = testad.ret_rot();
            std::vector<Eigen::MatrixXd> it_Kret = testad.ret_K();


            std::vector<float> ang = angels(it_rret[par.numb2ind[kvec[i]]]);

            Eigen::MatrixXd rot(3,3);
            double deg = ang[1];
            double cs = cos(deg);
            double sii = sin(deg);

            rot << cs, 0, sii,
                0, 1, 0,
                -sii, 0, cs;

            cv::Matx33f TR = cv::Matx33f::eye();
            TR(0,2) = 0 - it_Kret[par.numb2ind[kvec[i]]](0,0)*ang[1];
            TR(1,2) = 0;

            std::cout <<"\n"<<"ang[1] "<<ang[1]<<"\n";
            std::cout <<"\n"<<"ang[0] "<<ang[0]<<"\n";
            it_rret[par.numb2ind[kvec[i]]] = rot.transpose()*it_rret[par.numb2ind[kvec[i]]];

            if(vvec[i] != maxLoc){

                TR(1,2) = 24;
            }

            std::vector<float> ang2 = angels(it_rret[par.numb2ind[kvec[i]]]);
            std::cout <<"\n"<<"ang2[1] "<<ang2[1]<<"\n";
            std::cout <<"\n"<<"ang2[0] "<<ang2[0]<<"\n";

            struct bundle_par push;
            push.rot_key = it_rret[par.numb2ind[kvec[i]]];
            push.k_key = it_Kret[par.numb2ind[kvec[i]]];
            push.k_val = it_Kret[par.numb2ind[vvec[i]]];
            push.current_node = kvec[i];
            push.translation = TR;
            push.prev_node = vvec[i];

            tf_par[kvec[i]] = push;



            Eigen::MatrixXd rel_rotation = it_rret[par.numb2ind[kvec[i]]];
            Eigen::MatrixXd add = Eigen::MatrixXd::Zero(3, 3);
            int node = vvec[i];
            std::cout <<"\n"<<"focal "<<kvec[i]<<" " <<it_Kret[par.numb2ind[kvec[i]]](0,0)<<"\n";
            while(node != maxLoc){

                //std::cout <<"\n"<<"tf_par[node].rot_key "<<tf_par[node].rot_key<<"\n";
                rel_rotation = rel_rotation * tf_par[node].rot_key;
                TR = TR * tf_par[node].translation;
                add = add + tf_par[node].k_val - tf_par[node].k_key;
                node = tf_par[node].prev_node;

                add(0,0) = 0;
                add(1,1) = 0;
                cv::waitKey(0);
            }

            TR(0,2) = TR(0,2) -1000;

            //cv::Mat panorama = cv::Mat::zeros(1500,2067,CV_32FC3);
            //cv::Size si = panorama.size();

            {
                //add =tf_par[vvec[i]].k_val - tf_par[vvec[i]].k_key;

                std::cout <<"\n"<<"add "<<add<<"\n";
                add(0,0) = 0;
                add(1,1) = 0;

                class imgm::cylproj transformer2(rel_rotation, it_Kret[par.numb2ind[vvec[i]]],it_Kret[par.numb2ind[kvec[i]]] - add,(*Tnew.img_address)[kvec[i]].size(),0,+200);

                class imgm::cylhom translate(it_Kret[0](0,0),it_Kret[0](0,2),it_Kret[0](1,2),TR,0,0);

                cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[kvec[i]], transformer2,si);

                img_tr = imgm::applyGeometricTransform(img_tr, translate,si);

                img_tr.copyTo(panorama, img_tr);

            }
/*
            {
                std::cout <<"\n"<<"it_rret[par.numb2ind[vvec[i]]] "<<it_rret[par.numb2ind[vvec[i]]]<<"\n";

                std::cout <<"\n"<<"it_rret[par.numb2ind[kvec[i]]] "<<it_rret[par.numb2ind[kvec[i]]]<<"\n";

                class imgm::cylproj transformer(it_rret[par.numb2ind[vvec[i]]],it_Kret[par.numb2ind[vvec[i]]],it_Kret[par.numb2ind[vvec[i]]],(*Tnew.img_address)[vvec[i]].size(),0,0);
                cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[vvec[i]], transformer,si);
                img_tr.copyTo(panorama, img_tr);

                class imgm::cylproj transformer2(it_rret[par.numb2ind[kvec[i]]],it_Kret[par.numb2ind[vvec[i]]],it_Kret[par.numb2ind[kvec[i]]],(*Tnew.img_address)[kvec[i]].size(),0,0);
                cv::Mat img_tr2 = imgm::applyGeometricTransform((*Tnew.img_address)[kvec[i]], transformer2,si);
                img_tr2.copyTo(panorama, img_tr2);


            }
*/
            cv::imshow("Image Display",panorama);
            cv::waitKey(0);

            exists.insert(kvec[i]);
        }



}


}









#include "_stitch.h"

namespace stch {

/*algorithm for stitching:
 * set n = 3
 * find n best matches and adjust for n simultaneously.
 *
 * for every new image calculate the best match to the already adjusted pairs.
 * if the new image matches badly
 * if all options are bad remove the image.
 *
*/

std::vector<double> computeRowSumDividedByZeroCount(const cv::Mat& mat) {

    const int rows = mat.rows;
    const int cols = mat.cols;
    std::vector<double> results;
    results.reserve(rows);

    for (int i = 0; i < rows; ++i) {
        const cv::Mat row = mat.row(i);
        const double row_sum = cv::sum(row)[0];
        const int zero_count = cols - cv::countNonZero(row);
        results.push_back(row_sum / zero_count);
    }

    return results;
}


std::vector<int> getTopNonZeroIndices(const cv::Mat& M, int r, int n) {
    std::vector<std::pair<double, int>> elements;

    for (int c = 0; c < M.cols; ++c) {
        const double val = M.at<double>(r, c);
        if (val > 0) {
            elements.emplace_back(val, c);
        }
    }

    std::vector<int> idx;

    // Return all non-zero indices if <= n exist
    if (elements.size() <= static_cast<size_t>(n)) {
        idx.reserve(elements.size());
        for (const auto& elem : elements) {
            idx.push_back(elem.second);
        }
    }

    else {

        std::sort(elements.begin(), elements.end(),
                  [](const auto& a, const auto& b) { return a.first > b.first; });

        idx.reserve(n);
        for (int i = 0; i < n; ++i) {
            idx.push_back(elements[i].second);
        }
    }

    return idx;
}


Eigen::MatrixXd approximate(const cv::Matx33f &hom,Eigen::MatrixXd K){

    Eigen::MatrixXd H;
    cv::Matx33d homd = static_cast<cv::Matx33d>(hom);
    cv::cv2eigen(homd,H);
    H = H/H(2,2);

    Eigen::MatrixXd KRK = K.inverse() * H * K;
    Eigen::MatrixXd R_approx = ( KRK * KRK.transpose() ).pow(0.5) * (KRK.transpose()).inverse();

    return R_approx;
}



adjust_par prep_opt(const class imgm::pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >>& Hom_mat,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,const std::vector<maths::keypoints> &kp,const std::unordered_set<int> &exists,std::vector<int> indx,int ref,int attach){

    cv::Mat adj_new(indx.size(), indx.size(), T.adj->type());
    std::sort(indx.begin(), indx.end());
    std::vector<maths::keypoints> kp_new;
    std::vector<std::vector<std::vector<cv::DMatch>>> new_match_mat(indx.size(),std::vector<std::vector<cv::DMatch>>(indx.size()));

    for (int i = 0; i < indx.size(); ++i) {
        for (int j = 0; j < indx.size(); ++j) {

            adj_new.at<double>(i,j) = T.adj->at<double>(indx[i],indx[j]);
            new_match_mat[i][j] = match_mat[indx[i]][indx[j]];

        }
    }

    adjust_par ret(&adj_new,T.img_address);
    ret.adj = adj_new;
    ret.T.adj = &ret.adj;
    ret.ref = ref;
    ret.attatch = attach;
    ret.T.focal = T.K[ref](0,0);
    std::map<int, int> ind2numb;
    std::map<int, int> numb2ind;

    for (int i = 0; i < indx.size(); ++i) {

        kp_new.push_back(kp[indx[i]]);
        ind2numb[i] = indx[i];
        numb2ind[indx[i]] = i;
        if(exists.count(indx[i]) > 0){

            ret.T.rot.push_back(Eigen::MatrixXd::Identity(3, 3));
            ret.T.K.push_back(T.K[ref]);

        }else{

            if(T.rot[attach].isIdentity(1e-6)){
                Eigen::MatrixXd r_approx = approximate(Hom_mat[attach][indx[i]],T.K[attach]);
                ret.T.rot.push_back(r_approx);

            }else{
                ret.T.rot.push_back(T.rot[attach]);

            }
            ret.T.K.push_back(T.K[ref]);


        }

    }
    ret.ind2numb = ind2numb;
    ret.numb2ind = numb2ind;

    ret.kp = kp_new;
    ret.match = new_match_mat;

    return ret;
}





void bundleadjust_stitching(class imgm::pan_img_transform &T,class imgm::pan_img_transform &Tnew,const std::vector<std::vector< cv::Matx33f >> &Hom_mat,const std::vector<maths::keypoints> &kpold,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,int threads){

    cv::Mat path_mat = (*T.adj) + (*T.adj).t();
    std::vector<std::vector< cv::Matx33f >> Hom_mat_new(Hom_mat);
    std::vector<maths::keypoints> kp = kpold;
    std::vector<double> connectivity = computeRowSumDividedByZeroCount(path_mat);
    //find best node
    int maxLoc = std::distance(connectivity.begin(),std::max_element(connectivity.begin(), connectivity.end()));
    //all neighbours
    std::vector<std::pair<int, std::vector<int>>> tree = maths::bfs_ordered_with_neighbors(path_mat, maxLoc);

    // find n best matches sort the rest by stitching order

    std::map<int, int> tofrom;

        for(int i = 0;i < T.pair_stitch.size();i++){

            for(int j = 0 ; j < T.pair_stitch[i].size();j=j + 2){

                if(tofrom.count(T.pair_stitch[i][j+1]) < 1){
                   std::cout <<"\n"<<"from: "<<T.pair_stitch[i][j]<<" to: "<<T.pair_stitch[i][j+1]<<"\n";
                   tofrom[T.pair_stitch[i][j+1]] = T.pair_stitch[i][j];
                }
            }
        }

        std::cout <<"\n"<<"sorted: "<<"\n";
        int size = tofrom.size() + 1;
        std::unordered_set<int> calcs;
        calcs.insert(maxLoc);

        std::cout <<"\n"<<"adj: "<<"\n"<<(*T.adj)<<"\n";
        std::vector<int> kvec;
        std::vector<int> vvec;

        //find n best neighbours of maxLoc
        int n = 2;
        std::vector<int> ind = getTopNonZeroIndices(path_mat, maxLoc, n);



        ind.push_back(maxLoc); //<-best matching image
        std::map<int,int> testtt;

        std::unordered_set<int> exists;
        exists.insert(maxLoc);
        std::unordered_set<int> nearest;
        std::map<int, int>::iterator it;

        //std::sort(ind.begin(), ind.end());
        for(int i = 0;i < ind.size();i++){
            nearest.insert(ind[i]);
            std::cout <<"\n"<<"ind[i]: "<<"\n"<<ind[i]<<"\n";
        }

        while(calcs.size() < size){

            for (const auto& elem: calcs) {

                if(tofrom.count(elem) > 0){

                    it = tofrom.find(elem);
                    tofrom.erase(it);

                }

            }
            int c = 0;
            for ( const auto &[key, value]: tofrom ) {

                if(calcs.count(value) > 0){
                    //bool approx = false;

                    calcs.insert(key);

                    if(nearest.count(key) == 0){
                        kvec.push_back(key);
                        vvec.push_back(value);
                        std::cout <<"\n"<<"from: "<<value<<" to: "<<key<<"\n";
                    }

                }

            }

        }



        double focal = 600;
        Eigen::MatrixXd K = Eigen::MatrixXd::Identity(3, 3);
        K(0,0) = focal;
        K(1,1) = focal;
        K(0,2) = (int)((*Tnew.img_address)[maxLoc].cols / 2);
        K(1,2) = (int)((*Tnew.img_address)[maxLoc].rows / 2);

        for(int i = 0;i < T.K.size();i++){
            T.K[i] = K;
            T.rot[i] = Eigen::MatrixXd::Identity(3, 3);
        }

        adjust_par par = prep_opt(T,Hom_mat_new,match_mat,kp ,exists,ind,maxLoc,maxLoc);

        class bundm::adjuster testad(par.kp,par.match,.0001,par.T,false,threads);
        struct bundm::inter_par teees = testad.iterate();
        double error_value = testad.error_value;

        std::vector<Eigen::MatrixXd> rret = testad.ret_rot();
        std::vector<Eigen::MatrixXd> Kret = testad.ret_K();
        Eigen::MatrixXd ref_K = Kret[par.numb2ind[maxLoc]];
        T.K[maxLoc] = Kret[par.numb2ind[maxLoc]];
        T.rot[maxLoc] = rret[par.numb2ind[maxLoc]];

        cv::Mat panorama = cv::Mat::zeros(1500,2067,CV_32FC3);
        cv::Mat img;
        cv::Size si = panorama.size();


            float tx=0 ;
            float ty=0 ;



        for(int i = 0;i < n+1;i++){

            class imgm::cylproj transformer(rret[i],ref_K,Kret[i],(*Tnew.img_address)[par.ind2numb[i]].size(),0,0);

            cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[par.ind2numb[i]], transformer,si);
            img_tr.copyTo(panorama, img_tr);

            exists.insert(par.ind2numb[i]);
            cv::imshow("Image Display",panorama);
            cv::waitKey(0);

        }

        std::vector<struct bundle_par> con;

        for(int i = 0;i < kvec.size();i++){

            std::vector<int> ind;
            ind.push_back(kvec[i]);
            ind.push_back(vvec[i]);

            std::unordered_set<int> exists;
            exists.insert(vvec[i]);
            std::vector<int> ins = getTopNonZeroIndices(path_mat, maxLoc, 10);
/*
            for(int k = 0;k < ins.size();k++){
                if(ins[k] != vvec[i]){
                    ind.push_back(ins[k]);
                    std::cout<<"\n"<<"inserted: "<<ins[k];

                }

                if(ind.size() > 2){break;}
            }
*/


            std::cout <<"\n"<<"rot1: "<<"\n"<<ind[i]<<"\n";
            std::cout <<"\n"<<"rot2: "<<"\n"<<ind[i]<<"\n";

            adjust_par par = prep_opt(T,Hom_mat_new,match_mat,kp ,exists,ind,vvec[i],vvec[i]);
            class bundm::adjuster testad(par.kp,par.match,.0001,par.T,false,threads);
            struct bundm::inter_par teees = testad.iterate();

            std::vector<Eigen::MatrixXd> it_rret = testad.ret_rot();
            std::vector<Eigen::MatrixXd> it_Kret = testad.ret_K();

            struct bundle_par push;
            push.rot_val = it_rret[par.numb2ind[kvec[i]]];
            push.k_key = it_Kret[par.numb2ind[kvec[i]]];
            push.k_val = it_Kret[par.numb2ind[vvec[i]]];
            con.push_back(push);

            //cv::Mat panorama = cv::Mat::zeros(1500,2067,CV_32FC3);
            //cv::Size si = panorama.size();

            {
                class imgm::cylproj transformer(it_rret[par.numb2ind[kvec[i]]],it_Kret[par.numb2ind[vvec[i]]],it_Kret[par.numb2ind[kvec[i]]],(*Tnew.img_address)[kvec[i]].size(),0,0);
                cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[kvec[i]], transformer,si);
                img_tr.copyTo(panorama, img_tr);
            }
/*
            {
                class imgm::cylproj transformer(rret[par.numb2ind[vvec[i]]],Kret[par.numb2ind[vvec[i]]],Kret[par.numb2ind[vvec[i]]],(*Tnew.img_address)[vvec[i]].size(),0,0);
                cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[vvec[i]], transformer,si);
                img_tr.copyTo(panorama, img_tr);
            }
*/
            cv::imshow("Image Display",panorama);
            cv::waitKey(0);

            exists.insert(kvec[i]);
        }



}


}

#include "_stitch.h"

namespace stch {

/*algorithm for stitching:
 *

*/

std::vector<double> computeRowSumDividedByZeroCount(const cv::Mat& mat) {

    const int rows = mat.rows;
    const int cols = mat.cols;
    std::vector<double> results;
    results.reserve(rows);

    for (int i = 0; i < rows; ++i) {
        const cv::Mat row = mat.row(i);
        const double row_sum = cv::sum(row)[0];
        const int zero_count = cols - cv::countNonZero(row);
        results.push_back(row_sum / zero_count);
    }

    return results;
}


std::vector<int> getTopNonZeroIndices(const cv::Mat& M, int r, int n) {
    std::vector<std::pair<double, int>> elements;

    for (int c = 0; c < M.cols; ++c) {
        const double val = M.at<double>(r, c);
        if (val > 0) {
            elements.emplace_back(val, c);
        }
    }

    std::vector<int> idx;

    // Return all non-zero indices if <= n exist
    if (elements.size() <= static_cast<size_t>(n)) {
        idx.reserve(elements.size());
        for (const auto& elem : elements) {
            idx.push_back(elem.second);
        }
    }

    else {

        std::sort(elements.begin(), elements.end(),
                  [](const auto& a, const auto& b) { return a.first > b.first; });

        idx.reserve(n);
        for (int i = 0; i < n; ++i) {
            idx.push_back(elements[i].second);
        }
    }

    return idx;
}


Eigen::MatrixXd approximate(const cv::Matx33f &hom,Eigen::MatrixXd K){

    Eigen::MatrixXd H;
    cv::Matx33d homd = static_cast<cv::Matx33d>(hom);
    cv::cv2eigen(homd,H);
    H = H/H(2,2);

    Eigen::MatrixXd KRK = K.inverse() * H * K;
    Eigen::MatrixXd R_approx = ( KRK * KRK.transpose() ).pow(0.5) * (KRK.transpose()).inverse();

    return R_approx;
}


/*
void transformkp(maths::keypoints &kp,cv::Matx33f Hom){

    std::vector<cv::Point2f> temp;
    for(int i = 0;i < kp.keypoint.size();i++){

        temp.push_back(kp.keypoint[i].pt);

    }

    cv::perspectiveTransform(temp, temp, Hom);

    for(int i = 0;i < kp.keypoint.size();i++){

        kp.keypoint[i].pt = temp[i];

    }

}
*/

void transformkp(maths::keypoints &kp,class imgm::cylproj &transformer,cv::Mat &img,bool prnt){

    for(int i = 0;i < kp.keypoint.size();i++){

        //std::cout<<"before: "<<kp.keypoint[i].pt.x<<" "<<kp.keypoint[i].pt.y<<"\n";
        std::pair<float, float> coord = transformer.forward((float)kp.keypoint[i].pt.x,(float)kp.keypoint[i].pt.y);
        kp.keypoint[i].pt.x = coord.first;
        kp.keypoint[i].pt.y = coord.second;
        //std::cout<<"after: "<<kp.keypoint[i].pt.x<<" "<<kp.keypoint[i].pt.y<<"\n";
        if(prnt == true){cv::circle(img,cv::Point(coord.first, coord.second), 3,cv::Scalar(0, 0, 255),-1);}

    }

}


adjust_par prep_opt(const class imgm::pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >>& Hom_mat,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,const std::vector<maths::keypoints> &kp,const std::unordered_set<int> &exists,std::vector<int> indx,int ref,int attach){

    cv::Mat adj_new(indx.size(), indx.size(), T.adj->type());
    std::sort(indx.begin(), indx.end());
    std::vector<maths::keypoints> kp_new;
    std::vector<std::vector<std::vector<cv::DMatch>>> new_match_mat(indx.size(),std::vector<std::vector<cv::DMatch>>(indx.size()));

    for (int i = 0; i < indx.size(); ++i) {
        for (int j = 0; j < indx.size(); ++j) {

            adj_new.at<double>(i,j) = T.adj->at<double>(indx[i],indx[j]);
            new_match_mat[i][j] = match_mat[indx[i]][indx[j]];

        }
    }

    adjust_par ret(&adj_new,T.img_address);
    ret.adj = adj_new;
    ret.T.adj = &ret.adj;
    ret.ref = ref;
    ret.attatch = attach;
    ret.T.focal = T.K[ref](0,0);
    std::map<int, int> ind2numb;
    std::map<int, int> numb2ind;

    for (int i = 0; i < indx.size(); ++i) {

        kp_new.push_back(kp[indx[i]]);
        ind2numb[i] = indx[i];
        numb2ind[indx[i]] = i;
        if(exists.count(indx[i]) > 0){

            ret.T.rot.push_back(Eigen::MatrixXd::Identity(3, 3));
            ret.T.K.push_back(T.K[ref]);

        }else{

            if(T.rot[attach].isIdentity(1e-6)){
                Eigen::MatrixXd r_approx = approximate(Hom_mat[attach][indx[i]],T.K[attach]);
                ret.T.rot.push_back(r_approx);

            }else{
                //ret.T.rot.push_back(T.rot[attach]);
                Eigen::MatrixXd r_approx = approximate(Hom_mat[attach][indx[i]],T.K[attach]);
                ret.T.rot.push_back(r_approx);

            }
            ret.T.K.push_back(T.K[attach]);


        }

    }
    ret.ind2numb = ind2numb;
    ret.numb2ind = numb2ind;

    ret.kp = kp_new;
    ret.match = new_match_mat;

    return ret;
}


void update_par(class imgm::pan_img_transform &T,std::vector<std::vector< cv::Matx33f >> &Hom_mat,class bundm::adjuster &adjuster,std::vector<maths::keypoints> &kp,std::vector<int> ind,std::unordered_set<int> &exists,adjust_par &par,struct bundm::inter_par &it_par){

    std::vector<Eigen::MatrixXd> rret = adjuster.ret_rot();
    std::vector<Eigen::MatrixXd> Kret = adjuster.ret_K();
    std::unordered_set<int> copy_set = exists;

    for(int i = 0;i < ind.size();i++){

        if(exists.count(ind[i]) == 0){

            std::cout<<"\n"<<"updated: "<<ind[i] <<"to "<<par.attatch <<"\n";
            std::cout<<"\n"<<"updated: "<<ind[i] <<"to "<<par.attatch <<"\n";
            T.K[ind[i]] = Kret[par.numb2ind[ind[i]]];
            T.rot[ind[i]] = rret[par.numb2ind[ind[i]]];
            //transformkp(kp[ind[i]],it_par.hom[par.numb2ind[par.attatch]][par.numb2ind[ind[i]]]);
            copy_set.insert(ind[i]);

        }

        for(int j = 0;j < ind.size();j++){

            Hom_mat[ind[i]][ind[j]] = it_par.hom[par.numb2ind[ind[i]]][par.numb2ind[ind[j]]];

        }

    }

    exists = copy_set;

}

cv::Mat angels(const Eigen::MatrixXd &R){
    cv::Mat rot;
    cv::eigen2cv(R,rot);
/*
    float x = atan2(R(2,1),R(2,2)) ;
    float y = atan2(-R(2,0),sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2))) ;
    float z = atan2(R(1,0),R(0,0)) ;
*/
    cv::Mat ang;
    cv::Rodrigues(rot,ang);
    //std::vector<float> ang = {x,y,z};
    return ang;
}


void tkp(maths::keypoints &kp,cv::Matx33f Hom,cv::Mat &img,bool prnt,float focal){

    std::vector<cv::Vec3f> temp;
    for(int i = 0;i < kp.keypoint.size();i++){

        cv::Vec3f pnt;
        pnt[2] = 1;
        pnt[0] = (float)kp.keypoint[i].pt.x;
        pnt[1] = (float)kp.keypoint[i].pt.y;
        temp.push_back(pnt);

    }

    std::vector<cv::Vec3f> temp2 = maths::applyH_2D(temp, Hom, maths::GEOM_TYPE_POINT);

    for(int i = 0;i < kp.keypoint.size();i++){
        float x_rot = (temp2[i][0])/(temp2[i][2]);
        if(0 > x_rot){
            x_rot = -x_rot;
            x_rot = (float)(2*PI* focal) - std::fmod((float)x_rot,(float)(2*PI* focal));
        }else{
            x_rot = std::fmod((float)x_rot,(float)(2*PI* focal));
        }

        kp.keypoint[i].pt.x = x_rot;
        kp.keypoint[i].pt.y = (temp2[i][1])/(temp2[i][2]);
        if(prnt == true){cv::circle(img,cv::Point(x_rot, (temp2[i][1])/(temp2[i][2])), 3,cv::Scalar(0, 0, 255),-1);}

    }

}


void rotate_kp(maths::keypoints &kp,cv::Mat &img,float degree,float focal,bool print){

        cv::Matx33f Tr = cv::Matx33f::eye();
        Tr(0,2) = focal * degree;
        Tr(1,2) = 0;

        tkp(kp,Tr,img,print,focal);

}


void bundleadjust_stitching(class imgm::pan_img_transform &T,class imgm::pan_img_transform &Tnew,const std::vector<std::vector< cv::Matx33f >> &Hom_mat,const std::vector<maths::keypoints> &kpold,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,int threads){

    cv::Mat path_mat = (*T.adj) + (*T.adj).t();
    std::vector<std::vector< cv::Matx33f >> Hom_mat_new(Hom_mat);
    std::vector<maths::keypoints> kp = kpold;
    std::vector<double> connectivity = computeRowSumDividedByZeroCount(path_mat);
    //find best node
    int maxLoc = std::distance(connectivity.begin(),std::max_element(connectivity.begin(), connectivity.end()));
    //all neighbours
    std::vector<std::pair<int, std::vector<int>>> tree = maths::bfs_ordered_with_neighbors(path_mat, maxLoc);

    // find n best matches sort the rest by stitching order

    std::map<int, int> tofrom;

        for(int i = 0;i < T.pair_stitch.size();i++){

            for(int j = 0 ; j < T.pair_stitch[i].size();j=j + 2){

                if(tofrom.count(T.pair_stitch[i][j+1]) < 1){
                   std::cout <<"\n"<<"from: "<<T.pair_stitch[i][j]<<" to: "<<T.pair_stitch[i][j+1]<<"\n";
                   tofrom[T.pair_stitch[i][j+1]] = T.pair_stitch[i][j];
                }
            }
        }

        std::cout <<"\n"<<"sorted: "<<"\n";
        int size = tofrom.size() + 1;
        std::unordered_set<int> calcs;
        calcs.insert(maxLoc);

        std::cout <<"\n"<<"adj: "<<"\n"<<(*T.adj)<<"\n";
        std::vector<int> kvec;
        std::vector<int> vvec;

        //find n best neighbours of maxLoc
        int n = 2;
        std::vector<int> ind = getTopNonZeroIndices(path_mat, maxLoc, n);



        ind.push_back(maxLoc); //<-best matching image
        std::map<int,int> testtt;

        std::unordered_set<int> exists;
        exists.insert(maxLoc);
        std::unordered_set<int> nearest;
        std::map<int, int>::iterator it;

        //std::sort(ind.begin(), ind.end());
        for(int i = 0;i < ind.size();i++){
            nearest.insert(ind[i]);
            std::cout <<"\n"<<"ind[i]: "<<"\n"<<ind[i]<<"\n";
        }

        while(calcs.size() < size){

            for (const auto& elem: calcs) {

                if(tofrom.count(elem) > 0){

                    it = tofrom.find(elem);
                    tofrom.erase(it);

                }

            }
            int c = 0;
            for ( const auto &[key, value]: tofrom ) {

                if(calcs.count(value) > 0){
                    //bool approx = false;

                    calcs.insert(key);

                    if(nearest.count(key) == 0){
                        kvec.push_back(key);
                        vvec.push_back(value);
                        std::cout <<"\n"<<"from: "<<value<<" to: "<<key<<"\n";
                    }

                }

            }

        }



        double focal = 600;
        Eigen::MatrixXd K = Eigen::MatrixXd::Identity(3, 3);
        K(0,0) = focal;
        K(1,1) = focal;
        K(0,2) = (int)((*Tnew.img_address)[maxLoc].cols / 2);
        K(1,2) = (int)((*Tnew.img_address)[maxLoc].rows / 2);

        cv::Matx33f Tr = cv::Matx33f::eye();
        Tr(0,2) = 0;
        Tr(1,2) = 0;


        for(int i = 0;i < T.K.size();i++){
            T.K[i] = K;
            T.rot[i] = Eigen::MatrixXd::Identity(3, 3);
            //tkp(kp[i],Tr);
        }

        adjust_par par = prep_opt(T,Hom_mat_new,match_mat,kp ,exists,ind,maxLoc,maxLoc);

        class bundm::adjuster testad(par.kp,par.match,.0001,par.T,false,threads);
        struct bundm::inter_par teees = testad.iterate();
        double error_value = testad.error_value;

        std::vector<Eigen::MatrixXd> rret = testad.ret_rot();
        std::vector<Eigen::MatrixXd> Kret = testad.ret_K();
        Eigen::MatrixXd ref_K = Kret[par.numb2ind[maxLoc]];
        T.K[maxLoc] = Kret[par.numb2ind[maxLoc]];
        T.rot[maxLoc] = rret[par.numb2ind[maxLoc]];

        cv::Mat panorama = cv::Mat::zeros(1500,5067,CV_32FC3);
        cv::Mat img;
        cv::Size si = panorama.size();
/*
        Eigen::MatrixXd rot(3,3);
        double deg = ( PI / 180) * (50);
        double cs = cos(deg);
        double sii = sin(deg);

        rot << cs, 0, sii,
               0, 1, 0,
               -sii, 0, cs;


        for(int i = 0;i < 20;i++){

        Eigen::MatrixXd rot(3,3);
        double deg = ( PI / 180) * (0+50*i);
        double cs = cos(deg);
        double sii = sin(deg);

        rot << cs, 0, sii,
               0, 1, 0,
               -sii, 0, cs;


            maths::keypoints kp_temp = kp[par.ind2numb[1]];

            class imgm::cylproj transformer(rret[1],ref_K,Kret[1],(*Tnew.img_address)[par.ind2numb[1]].size(),0,0);

            class imgm::cylproj transformer2(rot*rret[1],ref_K,Kret[1],(*Tnew.img_address)[par.ind2numb[1]].size(),0,0);

            cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[par.ind2numb[1]], transformer2,si);
            img_tr.copyTo(panorama, img_tr);


            transformkp(kp_temp,transformer,panorama,false);
            rotate_kp(kp_temp,panorama,deg,Kret[1](0,0));

            //std::cout<<"\n"<<"trot after: "<<par.ind2numb[i]<<"\n"<<rret[i] <<"\n"<<K;

            cv::imshow("Image Display",panorama);
            cv::waitKey(0);

        }

*/



            float tx=0 ;
            float ty=0 ;





    Eigen::MatrixXd rot(3,3);
    double deg = ( PI / 180) * (0);
    double cs = cos(deg);
    double sii = sin(deg);

        rot << cs, 0, sii,
               0, 1, 0,
               -sii, 0, cs;


        update_par(T,Hom_mat_new,testad,kp,ind,exists,par,teees);
        for(int i = 0;i < n+1;i++){

            class imgm::cylproj transformer2(rret[i],ref_K,Kret[i],(*Tnew.img_address)[par.ind2numb[i]].size(),tx,ty);

            class imgm::cylproj transformer(rret[i],ref_K,Kret[i],(*Tnew.img_address)[par.ind2numb[i]].size(),0,0);


            //class imgm::cylhom transformer(teees.focal[i],Kret[i](0,2),Kret[i](1,2),(Tr*Hom_mat_new[maxLoc][par.ind2numb[i]]).inv(),0,0);

            cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[par.ind2numb[i]], transformer2,si);
            img_tr.copyTo(panorama, img_tr);
            transformkp(kp[par.ind2numb[i]],transformer,panorama,false);
            rotate_kp(kp[par.ind2numb[i]],panorama,0,Kret[i](0,0),false);

            //std::cout<<"\n"<<"trot after: "<<par.ind2numb[i]<<"\n"<<rret[i] <<"\n"<<K;

            cv::imshow("Image Display",panorama);
            cv::waitKey(0);

        }



        for(int i = 0;i < kvec.size();i++){

            std::vector<int> ind;
            ind.push_back(kvec[i]);
            ind.push_back(vvec[i]);

            std::cout<<"kvec[i] "<<kvec[i]<<"\n";
            std::cout<<"vvec[i] "<<vvec[i]<<"\n";
            std::cout<<"<maxLoc "<<maxLoc<<"\n";

            std::vector<int> ins = getTopNonZeroIndices(path_mat, maxLoc, 10);

/*

            for(int k = ins.size();k > 0;k--){
                if((exists.count(ins[k]) > 0) and (ins[k] != vvec[i])){
                    ind.push_back(ins[k]);
                }

                if(ind.size() > 2){break;}
            }

*/
/*
std::cout<<"\n"<<"kvec[i]: "<<kvec[i];
std::cout<<"\n"<<"vvec[i]: "<<vvec[i];
            for(int k = 0;k < ins.size();k++){
                if((exists.count(ins[k]) > 0) and (ins[k] != vvec[i])){
                    ind.push_back(ins[k]);
                    std::cout<<"\n"<<"inserted: "<<ins[k];

                }

                if(ind.size() > 2){break;}
            }
*/

            cv::Mat ang = angels(T.rot[vvec[i]]);

            Eigen::MatrixXd rot(3,3);
            double deg = -ang.at<double>(1);
            double cs = cos(deg);
            double sii = sin(deg);

            rot << cs, 0, sii,
                    0, 1, 0,
                    -sii, 0, cs;

            std::cout<<"ang "<<"\n"<<ang<<"\n";

            cv::Mat tempimg = panorama.clone();

            rotate_kp(kp[vvec[i]],panorama,deg,T.K[vvec[i]](0,0),true);
            std::cout<<"degree "<<"\n"<<T.rot[vvec[i]]<<"\n";
            T.rot[vvec[i]] =  T.rot[vvec[i]] * rot;
            std::cout<<"degree "<<"\n"<<T.rot[vvec[i]]<<"\n";
            cv::imshow("Image Display",panorama);
            cv::waitKey(0);

            panorama = tempimg;

            adjust_par par = prep_opt(T,Hom_mat_new,match_mat,kp ,exists,ind,maxLoc,vvec[i]);
            class bundm::adjuster testad(par.kp,par.match,.0001,par.T,false,threads);
            struct bundm::inter_par teees = testad.iterate();

            std::vector<Eigen::MatrixXd> rret = testad.ret_rot();
            std::vector<Eigen::MatrixXd> Kret = testad.ret_K();
            //Eigen::MatrixXd ref_K = Kret[par.numb2ind[vvec[i]]];


            update_par(T,Hom_mat_new,testad,kp,ind,exists,par,teees);

            class imgm::cylproj transformer2(T.rot[kvec[i]],Kret[par.numb2ind[vvec[i]]],T.K[kvec[i]],(*Tnew.img_address)[kvec[i]].size(),0,0);

            T.rot[vvec[i]] = rot.transpose() * T.rot[vvec[i]];
            T.rot[kvec[i]] = rot.transpose() * T.rot[kvec[i]];

            std::cout<<"kvec[i] M"<<"\n"<<Kret[par.numb2ind[kvec[i]]]<<"\n";
            std::cout<<"vvec[i] M"<<"\n"<<Kret[par.numb2ind[vvec[i]]]<<"\n";

            cv::Mat show_temp = panorama.clone();
            cv::Mat img_tr_tmp = imgm::applyGeometricTransform((*Tnew.img_address)[kvec[i]], transformer2,si);
            img_tr_tmp.copyTo(show_temp, img_tr_tmp);
            cv::imshow("Image Display",show_temp);
            cv::waitKey(0);

            class imgm::cylproj transformer(T.rot[kvec[i]],Kret[par.numb2ind[vvec[i]]],T.K[kvec[i]],(*Tnew.img_address)[kvec[i]].size(),0,0);



            //class imgm::cylproj transformer(rret[par.numb2ind[kvec[i]]],Kret[par.numb2ind[vvec[i]]],Kret[par.numb2ind[kvec[i]]],(*Tnew.img_address)[kvec[i]].size(),0,0);
            cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[kvec[i]], transformer,si);

            img_tr.copyTo(panorama, img_tr);
            transformkp(kp[kvec[i]],transformer2,panorama,false);

            rotate_kp(kp[kvec[i]],panorama,-deg,T.K[kvec[i]](0,0),true);
            rotate_kp(kp[vvec[i]],panorama,-deg,T.K[vvec[i]](0,0),false);

            tempimg = panorama;

            rotate_kp(kp[kvec[i]],panorama,0,Kret[par.numb2ind[kvec[i]]](0,0),false);
            cv::imshow("Image Display",panorama);
            cv::waitKey(0);

            tempimg = panorama;

        }


}


}
