#include "_stitch.h"
//center keypoints


namespace stch {

struct stitch_data get_proj_parameters(
    const std::vector<cv::Mat>& images,
    std::vector<Eigen::MatrixXd>& R,
    std::vector<Eigen::MatrixXd>& K,
    int ref,
    std::vector<double> &con){
    stitch_data par_stitch;

    const double focal = K[ref](0, 0);  // Focal length from first camera
    cv::Ptr<cv::detail::SphericalWarper> warper = cv::makePtr<cv::detail::SphericalWarper>(focal);
    std::vector<cv::Mat> warped_images;
    std::vector<cv::Point> corners;

    for (int i = 0; i < images.size(); i++) {
        if(con[i] > 0){

            const int w_ref = images[i].cols;
            const int h_ref = images[i].rows;
            cv::Mat cvK, cvR;
            Eigen::Matrix3d R_adj = R[i];

            const double f_i = K[i](0, 0);
            const double s = focal / f_i;
            const double c_x = K[i](0, 2);
            const double c_y = K[i](1, 2);

            Eigen::Matrix3d K_adj;
            K_adj << f_i, 0,w_ref-c_x,
                    0, f_i,h_ref-c_y,
                    0, 0, 1;

            cv::eigen2cv(K_adj, cvK);
            cv::eigen2cv(R_adj, cvR);
            cvK.convertTo(cvK, CV_32F);
            cvR.convertTo(cvR, CV_32F);

            cv::Mat warped_image;
            cv::Point warped_corner = warper->warp(
                images[i], cvK, cvR, cv::INTER_LINEAR, cv::BORDER_CONSTANT, warped_image
            );

            warped_images.push_back(warped_image);
            corners.push_back(warped_corner);

        }

    }

    par_stitch.corners = corners;
    par_stitch.imgs = warped_images;

    return par_stitch;
}


std::vector<NodeConnection> orderNodesByConnection(const cv::Mat &M) {
    int n = M.rows;
    std::vector<NodeConnection> result;

    if (n == 0) return result;

    // 1. Find initial node with maximum VALID connectivity (sum of weights > 0)
    int firstNode = -1;
    double maxSum = -1.0;
    for (int i = 0; i < n; ++i) {
        double sum = 0.0;
        for (int j = 0; j < n; ++j) {
            double weight = M.at<double>(i, j);
            if (weight > 0 && i != j) {  // Exclude self-connections
                sum += weight;
            }
        }
        if (sum > maxSum || (sum == maxSum && i < firstNode)) {
            maxSum = sum;
            firstNode = i;
        }
    }

    std::cout <<"\n"<<"maxnode: "<<firstNode<<"\n";

    std::vector<bool> added(n, false);
    result.push_back(NodeConnection{firstNode, -1});
    added[firstNode] = true;

    // 2. Iteratively add remaining nodes with valid connections
    while (result.size() < static_cast<size_t>(n)) {
        double maxStrength = -1.0;
        int nextNode = -1;
        int connectedNode = -1;

        // Check all unadded nodes
        for (int candidate = 0; candidate < n; ++candidate) {
            if (!added[candidate]) {
                double currentMax = -1.0;
                int currentConnected = -1;

                // Find strongest VALID connection to existing nodes
                for (int addedNode = 0; addedNode < n; ++addedNode) {
                    if (added[addedNode]) {
                        double weight = M.at<double>(candidate, addedNode);
                        if (weight > 0 && weight > currentMax) {
                            currentMax = weight;
                            currentConnected = addedNode;
                        }
                    }
                }

                // Only consider candidates with valid connections
                if (currentMax > 0) {
                    if (currentMax > maxStrength ||
                        (currentMax == maxStrength && candidate < nextNode)) {
                        maxStrength = currentMax;
                        nextNode = candidate;
                        connectedNode = currentConnected;
                    }
                }
            }
        }

        if (nextNode != -1) {
            result.push_back(NodeConnection{nextNode, connectedNode});
            added[nextNode] = true;
        } else {
            // This should never happen in a connected graph
            break;
        }
    }

    return result;
}


adjust_par prep_opt(class imgm::pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >>& Hom_mat,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,const std::vector<util::keypoints> &kp,std::vector<int> indx,int ref,int attach){

    cv::Mat adj_new(indx.size(), indx.size(), T.adj->type());
    std::sort(indx.begin(), indx.end());
    std::vector<util::keypoints> kp_new;
    std::vector<std::vector<std::vector<cv::DMatch>>> new_match_mat(indx.size(),std::vector<std::vector<cv::DMatch>>(indx.size()));

    for (int i = 0; i < indx.size(); ++i) {
        for (int j = 0; j < indx.size(); ++j) {

            adj_new.at<double>(i,j) = T.adj->at<double>(indx[i],indx[j]);
            new_match_mat[i][j] = match_mat[indx[i]][indx[j]];

        }
    }

    adjust_par ret(&adj_new,T.img_address);
    adj_new = adj_new + adj_new.t();
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


        ret.T.rot.push_back(T.rot[indx[i]]);
        ret.T.K.push_back(T.K[indx[i]]);


        }

    ret.ind2numb = ind2numb;
    ret.numb2ind = numb2ind;

    ret.kp = kp_new;
    ret.match = new_match_mat;

    return ret;
}


Eigen::MatrixXd approximate_rot(Eigen::MatrixXd &R_i,
                                Eigen::MatrixXd &K_i,
                                Eigen::MatrixXd &K_j,
                                Eigen::MatrixXd Hom) {

    Eigen::MatrixXd M = K_j.inverse() * Hom * K_i;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d Vt = svd.matrixV().transpose();

    // rotation (det=1)
    if (U.determinant() * Vt.determinant() < 0) {
        Vt.row(2) *= -1;
    }

    // Optimal rotation from SVD: R_opt = U * Vt
    Eigen::Matrix3d R_opt = U * Vt;

    return R_opt;
}


struct stitch_result bundleadjust_stitching(class imgm::pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >> &Hom_mat,const std::vector<util::keypoints> &kp,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,int threads,std::atomic<double>* f_adress){

    struct stitch_result res;
/*
    for(int j = 0; j < (*T.adj).rows;j++){
        cv::Mat outimg;
        std::vector<cv::KeyPoint> key;
        for(int i = 0;i < kp[j].keypoint.size();i++){
            cv::KeyPoint K;
            K.pt.x = kp[j].keypoint[i].pt.x + ((*T.img_address)[j].cols / 2);
            K.pt.y = kp[j].keypoint[i].pt.y + ((*T.img_address)[j].rows / 2);
            key.push_back(K);
        }

        //cv::drawKeypoints( (*T.img_address)[j], key, outimg, cv::Scalar::all(-1));

        //cv::imshow("Display window", outimg);
        //cv::waitKey(0);
    }
*/
    cv::Mat path_mat = (*T.adj) + (*T.adj).t();
    std::cout <<"\n"<<"path_mat: "<<"\n"<<path_mat<<"\n";

    std::vector<std::vector< cv::Matx33f >> Hom_mat_new(Hom_mat);

    //find best node
    int maxLoc = std::distance(T.connectivity.begin(),std::max_element(T.connectivity.begin(), T.connectivity.end()));

    std::cout <<"\n"<<"maxLoc: "<<maxLoc<<"\n";

    std::vector<NodeConnection> paths_clac = orderNodesByConnection(path_mat);

    std::vector<int> ind;
    for(int i = 0;i<paths_clac.size();i++){

        std::cout <<"\n"<<"added: "<<paths_clac[i].nodeAdded<<"\n";
        std::cout <<"\n"<<"conn: "<<paths_clac[i].connectedTo<<"\n";

    }
    double add_to_fraction = 0;
    if(not (f_adress == NULL)){

        add_to_fraction = (1.0/3.0) * (1.0/(double)(paths_clac.size()-1));
    }

    //*progress = 0;

    ind.push_back(paths_clac[0].nodeAdded);
    ind.push_back(paths_clac[1].nodeAdded);

    Eigen::MatrixXf homACf;
    cv::cv2eigen(Hom_mat[paths_clac[0].nodeAdded][paths_clac[1].nodeAdded],homACf);
    Eigen::MatrixXd homACd = homACf.cast<double>();
    T.rot[paths_clac[0].nodeAdded] = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd R_initial = approximate_rot(T.rot[paths_clac[0].nodeAdded],
                                T.K[paths_clac[0].nodeAdded],
                                T.K[paths_clac[0].nodeAdded],
                                homACd);


    T.rot[paths_clac[1].nodeAdded] = R_initial;
    T.K[paths_clac[1].nodeAdded] = T.K[paths_clac[0].nodeAdded];

    adjust_par par = prep_opt(T,Hom_mat_new,match_mat,kp ,ind,maxLoc,maxLoc);


    cv::Mat adjclone = (*par.T.adj);
    class bundm::adjuster testad(par.kp,par.match,.0001,par.T,threads);
    std::vector<Eigen::MatrixXd> rret = testad.ret_rot();
    std::vector<Eigen::MatrixXd> Kret = testad.ret_K();


    struct bundm::inter_par teees = testad.iterate();
    if(not (f_adress == NULL)){*f_adress = *f_adress + add_to_fraction; }
    rret = testad.ret_rot();
    Kret = testad.ret_K();


    for(int i = 0;i < ind.size();i++){

            T.rot[par.ind2numb[i]] = rret[i];
            T.K[par.ind2numb[i]] = Kret[i];

    }

    for (int l = 2;l<paths_clac.size();l++){
        if(not (f_adress == NULL)){*f_adress = *f_adress + add_to_fraction; }
        //*progress = *progress + .1;
        Eigen::MatrixXd K = T.K[paths_clac[l].connectedTo];
        K(0,2) = 0;
        K(1,2) = 0;
        T.K[paths_clac[l].nodeAdded] = K;



        ind.push_back(paths_clac[l].nodeAdded);
        std::vector<cv::Mat> img_data(ind.size());

        Eigen::MatrixXf homACf;
        cv::cv2eigen(Hom_mat[paths_clac[l].connectedTo][paths_clac[l].nodeAdded],homACf);
        Eigen::MatrixXd homACd = homACf.cast<double>();

        Eigen::MatrixXd R_initial = approximate_rot(T.rot[paths_clac[l].connectedTo],
                                T.K[paths_clac[l].connectedTo],
                                K,
                                homACd);

        T.rot[paths_clac[l].nodeAdded] =R_initial*T.rot[paths_clac[l].connectedTo];


        adjust_par par = prep_opt(T,Hom_mat_new,match_mat,kp ,ind,maxLoc,maxLoc);
        for(int i = 0;i < ind.size();i++){

            img_data[i] = (*T.img_address)[par.ind2numb[i]];

        }

        class bundm::adjuster testad(par.kp,par.match,.0001,par.T,threads);

        struct bundm::inter_par teees = testad.iterate();
        Hom_mat_new = teees.hom;

        std::vector<Eigen::MatrixXd> rret = testad.ret_rot();
        std::vector<Eigen::MatrixXd> Kret = testad.ret_K();

        for(int i = 0;i < ind.size();i++){

            T.rot[par.ind2numb[i]] = rret[i];
            T.K[par.ind2numb[i]] = Kret[i];

        }

    }

    std::vector<Eigen::MatrixXd> Crret = T.rot;
    res.K = T.K;

    for(int i = 0;i<paths_clac.size();i++){

        res.K[paths_clac[i].nodeAdded](0,2) = res.K[paths_clac[i].nodeAdded](0,2) + (*T.img_address)[paths_clac[i].nodeAdded].cols / 2;
        res.K[paths_clac[i].nodeAdded](1,2) = res.K[paths_clac[i].nodeAdded](1,2) + (*T.img_address)[paths_clac[i].nodeAdded].rows / 2 ;
    }

    par = prep_opt(T,Hom_mat_new,match_mat,kp ,ind,maxLoc,maxLoc);

    res.connectivity = T.connectivity;
    res.rot = strg::straightenPanorama(Crret);

    struct stch::stitch_data ret = get_proj_parameters((*T.img_address),res.rot,res.K,maxLoc,res.connectivity);
    res.maxLoc = maxLoc;

    std::vector<int> indices;  // Use int instead of Eigen::Index
    for (int i = 0; i < T.connectivity.size(); i++) {
        if (T.connectivity[i] > 0) {
            indices.push_back(i);
            res.ord.push_back(par.numb2ind[i]);
            cv::Mat masks_temp = blnd::createSurroundingMask(ret.imgs[par.numb2ind[i]], true, 1);
            res.masks.push_back(masks_temp);
        }
    }

    res.ind = indices;

    res.corners = ret.corners;
    Eigen::VectorXi indices_eigen = Eigen::Map<Eigen::VectorXi>(indices.data(), indices.size());
    Eigen::MatrixXd mat_temp;

    cv::cv2eigen(path_mat,mat_temp);
    Eigen::MatrixXd new_mat = mat_temp(indices_eigen, indices_eigen);
    cv::eigen2cv(new_mat,res.adj);

    res.imgs = ret.imgs;
    res.prev_size = util::get_pan_dimension(res.corners,res.imgs);

    return res;

}


}
