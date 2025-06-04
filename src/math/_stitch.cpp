#include "_stitch.h"
//center keypoints


namespace stch {

struct stitch_data get_proj_parameters(
    const std::vector<cv::Mat>& images,
    std::vector<Eigen::MatrixXd>& R,
    std::vector<Eigen::MatrixXd>& K,
    int maxLoc){
    int ref = maxLoc;
    stitch_data par_stitch;

    const double focal = K[ref](0, 0);  // Focal length from first camera
    cv::Ptr<cv::detail::SphericalWarper> warper = cv::makePtr<cv::detail::SphericalWarper>(focal);
    std::vector<cv::Mat> warped_images;
    std::vector<cv::Point> corners;
    std::vector<cv::Size> sizes;

    const int w_ref = images[0].cols;
    const int h_ref = images[0].rows;

    for (size_t i = 0; i < images.size(); ++i) {

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

        // Convert to OpenCV format (float)
        cv::eigen2cv(K_adj, cvK);
        cv::eigen2cv(R_adj, cvR);
        cvK.convertTo(cvK, CV_32F);
        cvR.convertTo(cvR, CV_32F);

        // Warp the image (dst is output, returns top-left corner)
        cv::Mat warped_image;
        cv::Point warped_corner = warper->warp(
            images[i], cvK, cvR, cv::INTER_LINEAR, cv::BORDER_CONSTANT, warped_image
        );

        // Store results

        warped_images.push_back(warped_image);
        corners.push_back(warped_corner);
        sizes.push_back(warped_image.size());
    }

    par_stitch.corners = corners;
    par_stitch.imgs = warped_images;

    return par_stitch;
}

std::vector<OverlapInfo> computeOverlapInfo(
    const std::vector<cv::Mat>& warped_images,
    const std::vector<cv::Point>& corners,
    const cv::Mat& adj) {

    // Precompute grayscale images and masks
    std::vector<cv::Mat> gray_images;
    std::vector<cv::Mat> masks;

    for (const auto& img : warped_images) {
        cv::Mat gray;
        if (img.channels() == 3) {
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        } else if (img.channels() == 4) {
            cv::cvtColor(img, gray, cv::COLOR_BGRA2GRAY);
        } else {
            gray = img.clone();
        }
        gray_images.push_back(gray);
        masks.push_back(blnd::createSurroundingMask(img,true, 1));
    }

    std::vector<OverlapInfo> results;

    for (int i = 0; i < adj.rows; ++i) {
        for (int j = 0; j < adj.cols; ++j) {
            if (adj.at<double>(i, j) > 0) {
                const cv::Point& corner_i = corners[i];
                const cv::Point& corner_j = corners[j];
                const cv::Mat& img_i = warped_images[i];
                const cv::Mat& img_j = warped_images[j];

                // Global rectangles
                cv::Rect rect_i(corner_i.x, corner_i.y, img_i.cols, img_i.rows);
                cv::Rect rect_j(corner_j.x, corner_j.y, img_j.cols, img_j.rows);

                // Intersection rectangle
                cv::Rect overlap_rect = rect_i & rect_j;
                if (overlap_rect.width <= 0 || overlap_rect.height <= 0) {
                    results.push_back({i, j, 0.0, 0.0, 0.0});
                    continue;
                }

                // Local ROIs within the images
                cv::Rect roi_i_local(overlap_rect.x - corner_i.x, overlap_rect.y - corner_i.y, overlap_rect.width, overlap_rect.height);
                cv::Rect roi_j_local(overlap_rect.x - corner_j.x, overlap_rect.y - corner_j.y, overlap_rect.width, overlap_rect.height);

                // Extract ROIs from masks and grayscale images
                cv::Mat mask_i_roi = masks[i](roi_i_local);
                cv::Mat mask_j_roi = masks[j](roi_j_local);
                cv::Mat gray_i_roi = gray_images[i](roi_i_local);
                cv::Mat gray_j_roi = gray_images[j](roi_j_local);

                // Combined valid region mask
                cv::Mat combined_mask;
                cv::bitwise_and(mask_i_roi, mask_j_roi, combined_mask);

                // Calculate overlap area
                double area = cv::countNonZero(combined_mask);

                // Calculate total intensities in the overlap region
                cv::Mat masked_i, masked_j;
                cv::bitwise_and(gray_i_roi, gray_i_roi, masked_i, combined_mask);
                cv::bitwise_and(gray_j_roi, gray_j_roi, masked_j, combined_mask);
                double I_i = cv::sum(masked_i)[0];
                double I_j = cv::sum(masked_j)[0];

                results.push_back({i, j, area, I_i, I_j});
            }
        }
    }

    return results;
}



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



adjust_par prep_opt(class imgm::pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >>& Hom_mat,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,const std::vector<maths::keypoints> &kp,std::vector<int> indx,int ref,int attach){

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




cv::Mat projectToCylinder(
    const std::vector<cv::Mat>& images,
    std::vector<Eigen::MatrixXd>& R,
    std::vector<Eigen::MatrixXd>& K,
    int maxLoc
) {
    int ref = maxLoc;
    // Step 1: Initialize cylindrical warper
    const double focal = K[ref](0, 0);  // Focal length from first camera
    cv::Ptr<cv::detail::SphericalWarper> warper = cv::makePtr<cv::detail::SphericalWarper>(focal);
    std::vector<cv::Mat> warped_images;
    std::vector<cv::Point> corners;
    std::vector<cv::Size> sizes;

    const int w_ref = images[0].cols;
    const int h_ref = images[0].rows;

    // Step 2: Warp images and record positions
    for (size_t i = 0; i < images.size(); ++i) {
        // Convert Eigen matrices to OpenCV format
        cv::Mat cvK, cvR;
        Eigen::Matrix3d R_adj = R[i];//*R[ref].transpose();

        const double f_i = K[i](0, 0);
        const double s = focal / f_i;
        const double c_x = K[i](0, 2);
        const double c_y = K[i](1, 2);

        Eigen::Matrix3d K_adj;
        K_adj << f_i, 0,w_ref-c_x,
                 0, f_i,h_ref-c_y,
                 0, 0, 1;

        // Convert to OpenCV format (float)
        cv::eigen2cv(K_adj, cvK);
        cv::eigen2cv(R_adj, cvR);
        cvK.convertTo(cvK, CV_32F);
        cvR.convertTo(cvR, CV_32F);

        // Warp the image (dst is output, returns top-left corner)
        cv::Mat warped_image;
        cv::Point warped_corner = warper->warp(
            images[i], cvK, cvR, cv::INTER_LINEAR, cv::BORDER_CONSTANT, warped_image
        );

        // Store results

        warped_images.push_back(warped_image);
        corners.push_back(warped_corner);
        sizes.push_back(warped_image.size());
    }

    // Step 3: Compute panorama bounds
    int min_x = INT_MAX, min_y = INT_MAX;
    int max_x = INT_MIN, max_y = INT_MIN;

    for (size_t i = 0; i < corners.size(); ++i) {
        min_x = std::min(min_x, corners[i].x);
        min_y = std::min(min_y, corners[i].y);
        max_x = std::max(max_x, corners[i].x + sizes[i].width);
        max_y = std::max(max_y, corners[i].y + sizes[i].height);
    }

    // Step 4: Create panorama canvas
    cv::Mat panorama(
        max_y - min_y,  // Height
        max_x - min_x,  // Width
        CV_8UC3,
        cv::Scalar(0, 0, 0)
    );

    // Step 5: Copy warped images into panorama
    for (size_t i = 0; i < warped_images.size(); ++i) {
        cv::Rect roi(
            corners[i].x - min_x,  // x offset
            corners[i].y - min_y,  // y offset
            sizes[i].width,
            sizes[i].height
        );
        warped_images[i].copyTo(panorama(roi),warped_images[i]);
    }

    return panorama;
}


std::vector<Eigen::MatrixXd> rot_to_frame(std::vector<Eigen::MatrixXd> &R,std::vector<NodeConnection> &paths_clac){

    std::vector<Eigen::MatrixXd> Rot;

    for (int l = 1;l<paths_clac.size();l++){
        std::cout<<"nodes: "<<paths_clac[l].nodeAdded<<" "<<paths_clac[l].connectedTo;

    }

    return Rot;
}



Eigen::MatrixXd approximate_rot(Eigen::MatrixXd &R_i,
                                Eigen::MatrixXd &K_i,
                                Eigen::MatrixXd &K_j,
                                Eigen::MatrixXd Hom) {
    // Compute relative transformation matrix
    Eigen::MatrixXd M = K_j.inverse() * Hom * K_i;

    // SVD decomposition: M = U * S * Vᵀ
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d Vt = svd.matrixV().transpose();

    // Ensure proper rotation (det=1)
    if (U.determinant() * Vt.determinant() < 0) {
        Vt.row(2) *= -1;
    }

    // Optimal rotation from SVD: R_opt = U * Vt
    Eigen::Matrix3d R_opt = U * Vt;

    // Compute absolute rotation for new image
    Eigen::Matrix3d R_j = R_i * R_opt;

    return R_j;
}


std::vector<std::vector<int>> getPaths(const std::vector<NodeConnection>& N) {
    // Create a map to quickly look up parent nodes
    std::unordered_map<int, int> parent_map;
    for (const auto& nc : N) {
        parent_map[nc.nodeAdded] = nc.connectedTo;
    }

    std::vector<std::vector<int>> result;

    // Build paths for each node
    for (const auto& nc : N) {
        std::vector<int> path;
        int current_node = nc.nodeAdded;
        int parent = nc.connectedTo;

        // Start with the node itself
        path.push_back(current_node);

        // Follow parent chain until root (parent == -1)
        while (parent != -1) {
            path.push_back(parent);
            current_node = parent;

            // Get next parent from the map
            auto it = parent_map.find(current_node);
            if (it == parent_map.end()) {
                break;  // Should never happen per problem constraints
            }
            parent = it->second;
        }

        result.push_back(path);
    }

    return result;
}


void projectToSphere(const cv::Mat& input, cv::Mat& output, const Eigen::MatrixXd& K, const Eigen::MatrixXd& R, int outputWidth, int outputHeight,double center,double xc) {
    // Extract camera parameters
    double f = K(0, 0);
    double px = K(0, 2);
    double py = K(1, 2);

    // Create mapping matrices
    cv::Mat map_x(outputHeight, outputWidth, CV_32FC1);
    cv::Mat map_y(outputHeight, outputWidth, CV_32FC1);

    for (int y = 0; y < outputHeight; ++y) {
        for (int x = 0; x < outputWidth; ++x) {
            // Calculate theta for cylindrical coordinates
            double theta = (static_cast<double>(x-xc) / K(0,0));

            // 3D point on the cylinder (world coordinates)
            double X_world = f * sin(theta);
            double Z_world = f * cos(theta);
            double Y_world = static_cast<double>(y-300-center);

            // Transform to camera coordinates
            Eigen::Vector3d P_world(X_world, Y_world, Z_world);
            Eigen::Vector3d P_cam = R.transpose() * P_world;

            if (P_cam.z() > 0) {
                // Project to image coordinates
                double u = (f * P_cam.x() / P_cam.z()) + px;
                double v = (f * P_cam.y() / P_cam.z()) + py;

                map_x.at<float>(y, x) = static_cast<float>(u);
                map_y.at<float>(y, x) = static_cast<float>(v);
            } else {
                // Mark as invalid (behind the camera)
                map_x.at<float>(y, x) = -1;
                map_y.at<float>(y, x) = -1;
            }
        }
    }

    // Apply remapping
    remap(input, output, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
}


std::vector<std::vector< cv::Matx33f >> bundleadjust_stitching(class imgm::pan_img_transform &T,class imgm::pan_img_transform &Tnew,const std::vector<std::vector< cv::Matx33f >> &Hom_mat,const std::vector<maths::keypoints> &kpold,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,int threads){

    threads = 12;
    cv::Mat path_mat = (*T.adj) + (*T.adj).t();
    std::cout <<"\n"<<"path_mat: "<<"\n"<<path_mat<<"\n";

    std::vector<std::vector< cv::Matx33f >> Hom_mat_new(Hom_mat);
    std::vector<maths::keypoints> kp = kpold;
    std::vector<double> connectivity = computeRowSumDividedByZeroCount(path_mat);
    //find best node
    int maxLoc = std::distance(connectivity.begin(),std::max_element(connectivity.begin(), connectivity.end()));

    std::cout <<"\n"<<"maxLoc: "<<maxLoc<<"\n";

    std::vector<NodeConnection> paths_clac = orderNodesByConnection(path_mat);

    std::vector<std::vector<int>> rev = getPaths(paths_clac);
    std::vector<int> ind;
    for(int i = 0;i<paths_clac.size();i++){

        std::cout <<"\n"<<"added: "<<paths_clac[i].nodeAdded<<"\n";
        std::cout <<"\n"<<"conn: "<<paths_clac[i].connectedTo<<"\n";

    }

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
    class bundm::adjuster testad(par.kp,par.match,.0001,par.T);
    std::vector<Eigen::MatrixXd> rret = testad.ret_rot();
    std::vector<Eigen::MatrixXd> Kret = testad.ret_K();


    for(int i = 0;i < ind.size();i++){

            std::cout <<"Kf "<<"\n"<<T.K[par.ind2numb[i]]<<"\n";
            std::cout <<"Rf "<<"\n"<<T.rot[par.ind2numb[i]]<<"\n";

    }


    struct bundm::inter_par teees = testad.iterate();
    rret = testad.ret_rot();
    Kret = testad.ret_K();


    for(int i = 0;i < ind.size();i++){

            T.rot[par.ind2numb[i]] = rret[i];
            T.K[par.ind2numb[i]] = Kret[i];
            std::cout <<"Kf "<<"\n"<<T.K[par.ind2numb[i]]<<"\n";
            std::cout <<"Rf "<<"\n"<<T.rot[par.ind2numb[i]]<<"\n";

    }



    std::vector<std::vector< cv::Matx33f >> hommat;

    for (int l = 2;l<paths_clac.size();l++){

        Eigen::MatrixXd K = T.K[paths_clac[l].connectedTo];
        T.K[paths_clac[l].nodeAdded] = K;
        K(0,2) = 0;
        K(1,2) = 0;


        ind.push_back(paths_clac[l].nodeAdded);
        std::vector<cv::Mat> img_data(ind.size());

        Eigen::MatrixXf homACf;
        cv::cv2eigen(Hom_mat[paths_clac[l].connectedTo][paths_clac[l].nodeAdded],homACf);
        Eigen::MatrixXd homACd = homACf.cast<double>();

        Eigen::MatrixXd R_initial = approximate_rot(T.rot[paths_clac[l].connectedTo],
                                T.K[paths_clac[l].connectedTo],
                                T.K[paths_clac[l].connectedTo],
                                homACd);

        T.rot[paths_clac[l].nodeAdded] = R_initial;


        adjust_par par = prep_opt(T,Hom_mat_new,match_mat,kp ,ind,maxLoc,maxLoc);
        for(int i = 0;i < ind.size();i++){

            img_data[i] = (*T.img_address)[par.ind2numb[i]];

        }

        class bundm::adjuster testad(par.kp,par.match,.0001,par.T);

        struct bundm::inter_par teees = testad.iterate();
        Hom_mat_new = teees.hom;

        std::vector<Eigen::MatrixXd> rret = testad.ret_rot();
        std::vector<Eigen::MatrixXd> Kret = testad.ret_K();

        for(int i = 0;i < ind.size();i++){

            T.rot[par.ind2numb[i]] = rret[i];
            T.K[par.ind2numb[i]] = Kret[i];

        }

    }


            std::cout <<"\n"<<"mr: "<<T.rot[maxLoc]<<"\n";

            cv::Mat panorama = cv::Mat::zeros(1800,4067,CV_32FC3);
            cv::Size si = panorama.size();

            std::vector<Eigen::MatrixXd> Crret = T.rot;
            std::vector<Eigen::MatrixXd> CKret = T.K;
            int add = maxLoc;

            cv::Matx33f Tr = cv::Matx33f::eye();

            Tr(0,2) = (*T.img_address)[0].rows/2;
            Tr(1,2) = (*T.img_address)[0].cols/2;


            std::vector<cv::Mat> img = (*T.img_address);
            Eigen::MatrixXd Tre = Eigen::Matrix3d::Identity();

            Tre(0,2) = (*T.img_address)[0].rows/2;
            Tre(1,2) = (*T.img_address)[0].cols/2;



    double temp = CKret[rev[0][0]](0,0);
    for(int i = 0;i < rev.size();i++){
        std::cout<<"\n";

        std::cout<<"\n";

        CKret[rev[i][0]](0,2) = CKret[rev[i][0]](0,2)+ (*T.img_address)[rev[i][0]].rows / 2;
        CKret[rev[i][0]](1,2) = CKret[rev[i][0]](1,2)+ (*T.img_address)[rev[i][0]].cols / 2;
        //CKret[rev[i][0]](0,0) = temp;
        //CKret[rev[i][0]](1,1) = temp;
        std::cout<<"i: "<<CKret[rev[i][0]]<<"\n";

    }

    struct stitch_data test = get_proj_parameters(
            (*T.img_address),
            Crret,
            CKret,
            maxLoc
        );

    test.adj = path_mat;

    std::vector<double> GAIN = gain::gain_compensation(test.imgs,test.adj,test.corners);

    std::vector<cv::Mat> imggain = (*T.img_address);

    int locind = maxLoc;
    cv::Mat imgpan = projectToCylinder(
        (*T.img_address),
        Crret,
        CKret,
        locind
    );

    cv::imshow("Image Display", imgpan);
    cv::waitKey(0);

    for(int i = 0;i < GAIN.size();i++){

            std::cout <<"GAIN "<<"\n"<<GAIN[i]<<"\n";
            imggain[i] = imggain[i] / GAIN[i];

    }





        cv::Mat imgpan2 = projectToCylinder(
            imggain,
            Crret,
            CKret,
            locind
        );

        cv::imshow("Image Display", imgpan2);
        cv::waitKey(0);




return hommat;

}




}
