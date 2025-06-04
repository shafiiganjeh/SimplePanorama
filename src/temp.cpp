
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

// Compute the up-vector (normal to the plane of camera X-vectors)
Eigen::Vector3d computePanoramaUpVector(const std::vector<Eigen::Matrix3d>& rotations) {
    std::vector<Eigen::Vector3d> X_vectors;
    for (const auto& R : rotations) {
        X_vectors.push_back(R.col(0).normalized()); // X-axis = first column
    }

    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    for (const auto& X : X_vectors) {
        C += X * X.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(C);
    Eigen::Vector3d u = eigensolver.eigenvectors().col(0);

    // Ensure the up-vector points "upwards" (positive Y direction)
    if (u.dot(Eigen::Vector3d::UnitY()) < 0) {
        u = -u; // Flip sign if pointing downward
    }
    return u;
}

// Compute rotation matrix using Rodrigues' formula
Eigen::Matrix3d rodriguesRotation(const Eigen::Vector3d& axis, double angle) {
    Eigen::Matrix3d K;
    K << 0, -axis.z(), axis.y(),
         axis.z(), 0, -axis.x(),
         -axis.y(), axis.x(), 0;
    return Eigen::Matrix3d::Identity() + sin(angle) * K + (1 - cos(angle)) * K * K;
}

int main() {
    // Example: Cameras tilted 30° around Z-axis (X-vectors in a plane)
    std::vector<Eigen::Matrix3d> rotations;
    Eigen::Matrix3d base_rot = Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    for (int i = 0; i < 10; ++i) {
        Eigen::Matrix3d R = base_rot * Eigen::AngleAxisd(0.1*i, Eigen::Vector3d::UnitY()).toRotationMatrix();
        rotations.push_back(R);
    }

    // Step 1-3: Compute up-vector 'u'
    Eigen::Vector3d u = computePanoramaUpVector(rotations);
    std::cout << "Computed up-vector: " << u.transpose() << std::endl;

    // Step 4: Compute correction rotation (align u with target_up = [0,1,0])
    Eigen::Vector3d target_up = Eigen::Vector3d::UnitY();
    Eigen::Vector3d axis = target_up.cross(u); // Correct axis direction
    double axis_norm = axis.norm();

    Eigen::Matrix3d R_correct;

    if (axis_norm < 1e-6) {
        // Handle colinear case (u already aligned with target_up)
        if (u.dot(target_up) < 0) {
            // 180° rotation around X-axis (prevents mirroring)
            R_correct = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
        } else {
            R_correct = Eigen::Matrix3d::Identity();
        }
    } else {
        axis /= axis_norm;
        double angle = acos(u.dot(target_up));
        R_correct = rodriguesRotation(axis, angle);
    }

    // Step 5: Apply correction to all rotations (R_correct * R)
    std::vector<Eigen::Matrix3d> corrected_rotations;
    for (const auto& R : rotations) {
        corrected_rotations.push_back(R_correct * R);
    }

    // Verify corrected up-vector
    Eigen::Vector3d u_corrected = computePanoramaUpVector(corrected_rotations);
    std::cout << "Corrected up-vector: " << u_corrected.transpose() << std::endl;

    return 0;
}

        std::pair<float, float> cylproj::inv(float x, float y){

            x = x - tx;
            y = y - ty;
            float x_;
            float y_;
            float z_;

            float theta = (x - cx_a ) / (f_a);
            //float theta = (x - cx_a ) / (f_a);
            float h = (y - cy_a ) / (f_a);

            //theta = fmod(theta + M_PI, 2 * M_PI) - M_PI;

            float zx = sin(theta);
            float zy = h;
            float zz = cos(theta);

            z_ = v[6]*zx+v[7]*zy+v[8]*zz;
            if ((z_ <= 0)) {

                return {-1, -1};

            }

            x_ = v[0]*zx+v[1]*zy+v[2]*zz;
            y_ = v[3]*zx+v[4]*zy+v[5]*zz;

            zx = x_;
            zy = y_;
            zz = z_;

            zx = zx / zz;
            zy = zy / zz;

            float map_x;
            float map_y;


            if (abs(x-tx) < 2*PI*f_a ) {
                map_x = f_b * zx + cx_b ;
                map_y = f_b * zy + cy_b ;
            }else{

                map_x = -1;
                map_y = -1;

            }


            return {map_x, map_y};
        }







    std::vector<cv::DMatch> adj_calculator::clean_matches(const struct maths::keypoints &kp1,const struct maths::keypoints &kp2,std::vector<cv::DMatch> match,const  cv::Matx33f &H,const std::vector<float> &T){

        std::vector<cv::DMatch> clean_match;
        std::vector<cv::Vec2f> true_;
        std::vector<cv::Vec2f> pred_;

        for (int i = 0;i < match.size();i++){

            true_.push_back(kp1.keypoint[match[i].queryIdx ].pt);
            pred_.push_back(kp2.keypoint[match[i].trainIdx ].pt);
        }

        cv::perspectiveTransform(pred_, pred_, H);

        std::vector<cv::Vec2f> val;
        std::vector<float> dist;

        cv::absdiff(true_,pred_,val);

        int out = 0;

        for (int i = 0; i < val.size();i++){

            if (( val[i][0] < T[0] ) and ( val[i][1] < T[1] )){

                clean_match.push_back(match[i]);

            }

        }

        return clean_match;
    }


#include <opencv2/opencv.hpp>
#include <cmath>
#include <limits>

using namespace cv;
using namespace std;

Size computeCylindricalCanvasSize(const Mat& K, const Size& imageSize) {
    double f = K.at<double>(0, 0);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    double min_s = numeric_limits<double>::max();
    double max_s = -numeric_limits<double>::max();
    double min_t = numeric_limits<double>::max();
    double max_t = -numeric_limits<double>::max();

    // Function to process a single pixel (u, v)
    auto processPoint = [&](int u, int v) {
        double x = (u - cx) / f;
        double y = (v - cy) / f;

        double theta = atan(x);
        double h = y / sqrt(x*x + 1);

        double s = f * theta;
        double t = f * h;

        min_s = min(min_s, s);
        max_s = max(max_s, s);
        min_t = min(min_t, t);
        max_t = max(max_t, t);
    };

    // Sample along all edges at 1-pixel intervals
    // Top and bottom edges
    for (int u = 0; u < imageSize.width; ++u) {
        processPoint(u, 0);                // Top edge
        processPoint(u, imageSize.height-1); // Bottom edge
    }

    // Left and right edges
    for (int v = 0; v < imageSize.height; ++v) {
        processPoint(0, v);               // Left edge
        processPoint(imageSize.width-1, v); // Right edge
    }

    // Calculate canvas dimensions with 5% padding
    double width = max_s - min_s;
    double height = max_t - min_t;

    return Size(static_cast<int>(ceil(width * 1.05)),
           static_cast<int>(ceil(height * 1.05)));
}

// Usage example:
int main() {
    Mat K = (Mat_<double>(3,3) << 800, 0, 640,
                                   0, 800, 360,
                                   0, 0, 1);
    Size imgSize(1280, 720);

    Size canvasSize = computeCylindricalCanvasSize(K, imgSize);
    cout << "Optimal canvas size: " << canvasSize << endl;

    return 0;
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


adjust_par prep_adjust(const cv::Mat &adj,const std::vector<maths::keypoints> &kp,const class imgm::pan_img_transform &T,std::vector<int> indx,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,int stickto,int stickto_ref,const std::vector<std::vector<cv::Matx33f>> &Hom_mat,bool approx,std::unordered_set<int> &exists){

    int size = indx.size();
    std::sort(indx.begin(), indx.end());
    std::vector<maths::keypoints> kp_new;
    std::vector<std::vector<std::vector<cv::DMatch>>> new_match_mat(size,std::vector<std::vector<cv::DMatch>>(size));
    cv::Mat adj_new(indx.size(), indx.size(), adj.type());

    for (int i = 0; i < indx.size(); ++i) {
        for (int j = 0; j < indx.size(); ++j) {

            adj_new.at<double>(i,j) = adj.at<double>(indx[i],indx[j]);
            new_match_mat[i][j] = match_mat[indx[i]][indx[j]];
            //std::cout <<"\n"<<"mmsize: "<<"\n"<<new_match_mat[i][j].size()<<"\n";
        }
    }
    double focal = T.K[stickto](0,0);

    //class imgm::pan_img_transform t_new(&adj_new,T.img_address);
    adjust_par ret(&adj_new,T.img_address);
    ret.adj = adj_new;
    ret.T.adj = &ret.adj;
    ret.T.focal = focal;

    for (int i = 0; i < indx.size(); ++i) {

        kp_new.push_back(kp[indx[i]]);
        if((indx[i] == stickto) or (exists.count(indx[i]) > 0)){

            ret.T.rot.push_back(Eigen::MatrixXd::Identity(3, 3));
            ret.T.K.push_back(T.K[stickto]);

        }else if((approx == false) and (indx[i] != stickto)){

            ret.T.rot.push_back(T.rot[stickto_ref]);
            ret.T.K.push_back(T.K[stickto_ref]);

        }else{
            Eigen::MatrixXd r_approx = approximate(Hom_mat[stickto][indx[i]],T.K[stickto]);
            //std::cout <<"\n"<<"r_approx: "<<"\n"<<r_approx<<"\n";
            ret.T.rot.push_back(r_approx);
            ret.T.K.push_back(T.K[stickto_ref]);
        }

    }

    ret.kp = kp_new;
    ret.match = new_match_mat;

    return ret;
}


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


void resort_hom(const std::vector<int> &ind,const std::vector<std::vector<cv::Matx33f>> &hom, std::vector<std::vector<cv::Matx33f>> &hom_sorted,std::unordered_set<int> &exists){

    std::vector<int> index_sorted = ind;
    std::sort(index_sorted.begin(), index_sorted.end());
    for(int i = 0;i < index_sorted.size();i++){
        for(int j = 0;j < index_sorted.size();j++){
            if((exists.count(index_sorted[i]) == 0) or (exists.count(index_sorted[j]) == 0)){
                hom_sorted[index_sorted[i]][index_sorted[j]] = hom[i][j];
            }
        }

    }

}

//ändere mit menge statt stickto
void update_par(class bundm::adjuster &adjuster,const std::vector<std::vector<cv::Matx33f>> &hom,class imgm::pan_img_transform &T,const std::vector<int> &ind,std::vector<maths::keypoints> &kp,std::unordered_set<int> &exists,bool update_all,int stickto){

    std::vector<int> index_sorted = ind;
    std::vector<Eigen::MatrixXd> rotiter = adjuster.ret_rot();
    std::vector<Eigen::MatrixXd> kit = adjuster.ret_K();
    std::sort(index_sorted.begin(), index_sorted.end());
    for(int i = 0;i < index_sorted.size();i++){

        if((exists.count(index_sorted[i]) == 0) or (update_all == true)){

            transformkp(kp[index_sorted[i]],hom[stickto][index_sorted[i]]);
            T.K[index_sorted[i]] = kit[i];
            T.rot[index_sorted[i]] = rotiter[i];

        }

    }

}

cv::Matx33f get_img_tr(const cv::Matx33f &H,const cv::Mat &attach){


            std::vector<cv::Vec2f> cor;

            cor.push_back(cv::Vec2f(0,0));
            cor.push_back(cv::Vec2f(0,attach.rows));
            cor.push_back(cv::Vec2f(attach.cols,0));
            cor.push_back(cv::Vec2f(attach.cols,attach.rows));

            cv::perspectiveTransform(cor, cor, H);

            std::cout<<"\n"<< "00: "<<cor[1]<<"\n";

            float xstart = std::min( std::min( cor[0][0], cor[1][0]), (float)0);
            float xend   = std::max( std::max( cor[2][0], cor[3][0]), (float)attach.cols);
            float ystart = std::min( std::min( cor[0][1], cor[2][1]), (float)0);
            float yend   = std::max( std::max( cor[1][1], cor[3][1]), (float)attach.rows);

            // create translation matrix
            cv::Matx33f T = cv::Matx33f::zeros();
            T(0, 0) = 1;
            T(1, 1) = 1;
            T(2, 2) = 1;
            T(0, 2) = attach.cols-cor[3][0];
            T(1, 2) = attach.rows-cor[3][1];


    return T;
}

cv::Size get_size(const cv::Mat &img,const cv::Matx33f &hom){

    std::vector<cv::Vec2f> cor;
    cor.push_back(cv::Vec2f(0,0));
    cor.push_back(cv::Vec2f(0,img.rows));
    cor.push_back(cv::Vec2f(img.cols,0));
    cor.push_back(cv::Vec2f(img.cols,img.rows));

    cv::perspectiveTransform(cor, cor, hom);

    float xstart = std::min( std::min( cor[0][0], cor[1][0]), (float)0);
    float xend   = std::max( std::max( cor[2][0], cor[3][0]), (float)img.cols);
    float ystart = std::min( std::min( cor[0][1], cor[2][1]), (float)0);
    float yend   = std::max( std::max( cor[1][1], cor[3][1]), (float)img.rows);
    return cv::Size(xend - xstart + 1, yend - ystart + 1);

}


std::vector<float> angels(const Eigen::MatrixXd &R){

    float x = atan2(R(2,1),R(2,2)) ;
    float y = atan2(-R(2,0),sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2))) ;
    float z = atan2(R(1,0),R(0,0)) ;

    std::vector<float> ang = {x,y,z};
    return ang;
}


void bundleadjust_stitching(class imgm::pan_img_transform &T,class imgm::pan_img_transform &Tnew,const std::vector<std::vector< cv::Matx33f >> &Hom_mat,const std::vector<maths::keypoints> &kpold,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,int threads){


    std::vector<maths::keypoints> kp = kpold;
    cv::Mat path_mat = (*T.adj) + (*T.adj).t();
    std::vector<std::vector< cv::Matx33f >> Hom_mat_new(Hom_mat);
    std::vector<double> connectivity = computeRowSumDividedByZeroCount(path_mat);
    int maxLoc = std::distance(connectivity.begin(),std::max_element(connectivity.begin(), connectivity.end()));
    //all neighbours
    std::vector<std::pair<int, std::vector<int>>> tree = maths::bfs_ordered_with_neighbors(path_mat, maxLoc);

    std::vector<int> test = getTopNonZeroIndices(path_mat, 6, 10);
    for(int t = 0 ;t<test.size() ;t++){

        std::cout<<"\n"<<"in val"<<test[t]<<" ";
    }

    // find n best matches sort the rest by stitching order
    struct bundle_par insert;
    std::map<int, int> tofrom;
    std::map<int, int>::iterator it;

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
        int n = 1;
        std::vector<int> ind = getTopNonZeroIndices(path_mat, maxLoc, n);
        ind.push_back(maxLoc); //<-best matching image
        std::map<int,int> testtt;

        std::unordered_set<int> exists;
        std::unordered_set<int> nearest;
        //std::sort(ind.begin(), ind.end());
        for(int i = 0;i < ind.size();i++){
            nearest.insert(ind[i]);
            testtt[ind[i]] = i;

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
                    std::vector<int> indx(2);
                    indx[1] = key;
                    indx[0] = value;


                    if(nearest.count(key) == 0){
                        kvec.push_back(key);
                        vvec.push_back(value);
                        std::cout <<"\n"<<"from: "<<value<<" to: "<<key<<"\n";
                    }

                }

            }

        }


        double focal = 1200;
        Eigen::MatrixXd K = Eigen::MatrixXd::Identity(3, 3);
        K(0,0) = focal;
        K(1,1) = focal;
        for(int i = 0;i < T.K.size();i++){
            T.K[i] = K;
            T.rot[i] = Eigen::MatrixXd::Identity(3, 3);
        }

        std::cout<<"indis: "<<ind.size()<<"\n";

        adjust_par par = prep_adjust((*T.adj),kp,T,ind,match_mat,maxLoc,maxLoc,Hom_mat,true,exists);
        class bundm::adjuster testad(par.kp,par.match,.0001,par.T,true,threads);
        struct bundm::inter_par teees = testad.iterate();
        double error_value = testad.error_value;

        resort_hom(ind,teees.hom,Hom_mat_new,exists);
        update_par(testad,Hom_mat_new,T,ind,kp,exists,true,maxLoc);

        for(int i = 0;i < ind.size();i++){
            exists.insert(ind[i]);
        }

        //teees.focal[2] = 1500;
        cv::Matx33f Tr = cv::Matx33f::eye();
        Tr(0,2) = 200;
        Tr(1,2) = 150;
        cv::Mat panorama = cv::Mat::zeros(2000,3067,CV_32FC3);
        cv::Mat img;
        cv::Size si = panorama.size();

        std::vector<Eigen::MatrixXd> rret = testad.ret_rot();
        std::vector<Eigen::MatrixXd> Kret = testad.ret_K();


        for(int i = 0;i < n+1;i++){

            cv::Size sz = get_size((*Tnew.img_address)[ind[i]],Hom_mat_new[maxLoc][ind[i]]);
            cv::Matx33f Ts = get_img_tr(Hom_mat_new[maxLoc][ind[i]],(*Tnew.img_address)[ind[i]]);
            /*
            std::vector<float> ang = angels(T.rot[ind[i]]);
            std::cout<<"rotx "<<"\n"<<ang[0]<<"\n";
            std::cout<<"roty "<<"\n"<<ang[1]<<"\n";
            std::cout<<"rotz "<<"\n"<<ang[2]<<"\n";
            */
            //std::cout<<"translation "<<"\n"<<Ts<<"\n";
            std::cout<<"\n"<<"--------------------------------"<<"\n";
            std::cout<<"T.K[maxLoc] "<<"\n"<<T.K[maxLoc]<<"\n";
            std::cout<<"T.K[ind[i]] "<<"\n"<<T.K[ind[i]]<<"\n";
            std::cout<<"\n"<<"--------------------------------"<<"\n";
            float tx=400 ;
            float ty=150 ;
            class imgm::cylproj transformer(T.rot[ind[i]],T.K[maxLoc],T.K[ind[i]],(*Tnew.img_address)[ind[i]].size(),tx,ty);
            //class imgm::cylhom transformer(teees.focal[testtt[ind[i]]],Kret[testtt[ind[i]]](0,2),Kret[testtt[ind[i]]](1,2),(Tr*Hom_mat_new[maxLoc][ind[i]]).inv(),ang[0],ang[1]);
            cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[ind[i]], transformer,si);

            img_tr.copyTo(panorama, img_tr);
            cv::imshow("Image Display",panorama);
            cv::waitKey(0);

        }


for(int i = 0;i < kvec.size();i++){
    float tx=400 ;
    float ty=150 ;

    std::cout<<"\n"<<"--------------------------------"<<"\n";
    std::cout<<"vvec[i] "<<"\n"<<vvec[i]<<"\n";
    std::cout<<"maxLoc "<<"\n"<<maxLoc<<"\n";
    std::cout<<"\n"<<"--------------------------------"<<"\n";

    bool approx = false;
    if(vvec[i] == maxLoc){
        approx = true;
    }

    //std::vector<int> ind = getTopNonZeroIndices(path_mat, kvec[i], 2);
    std::vector<int> ind;
    ind.push_back(vvec[i]);
    ind.push_back(kvec[i]);
    adjust_par par2 = prep_adjust((*T.adj),kp,T,ind,match_mat,maxLoc,vvec[i],Hom_mat,approx,exists);

    for(int j = 0;j < 2;j++){

        std::cout<<"\n"<<"--------------------------------"<<"\n";
        std::cout<<"par.T.rot "<<"\n"<<par2.T.rot[j]<<"\n";
        std::cout<<"par.T.K "<<"\n"<<par2.T.K[j]<<"\n";
        std::cout<<"\n"<<"--------------------------------"<<"\n";

    }

    class bundm::adjuster testad2(par2.kp,par2.match,.0001,par2.T,false,threads);
    struct bundm::inter_par teees2 = testad2.iterate();
    std::cout<<"\n"<<"Iterated"<<"\n";
    resort_hom(ind,teees2.hom,Hom_mat_new,exists);
    update_par(testad2,Hom_mat_new,T,ind,kp,exists,false,vvec[i]);
    exists.insert(kvec[i]);

    class imgm::cylproj transformer(T.rot[kvec[i]],T.K[vvec[i]],T.K[kvec[i]],(*Tnew.img_address)[kvec[i]].size(),tx,ty);

    cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[ind[i]], transformer,si);

    img_tr.copyTo(panorama, img_tr);
    cv::imshow("Image Display",panorama);
    cv::waitKey(0);

}






/*
std::unordered_set<int> exists2;
exists2.insert(maxLoc);
//stick/ref
        adjust_par par2 = prep_adjust((*T.adj),kp,T,ind,match_mat,maxLoc,ind[1],Hom_mat,false,exists2);
        class bundm::adjuster testad2(par2.kp,par2.match,.0001,par2.T,false,threads);

        for(int i = 0;i < n+1;i++){

            std::cout<<"\n"<<"--------------------------------"<<"\n";
            std::cout<<"par.T.rot "<<"\n"<<par2.T.rot[i]<<"\n";
            std::cout<<"par.T.K "<<"\n"<<par2.T.K[i]<<"\n";
            std::cout<<"\n"<<"--------------------------------"<<"\n";


        }

        struct bundm::inter_par teees2 = testad2.iterate();




        bool approx = false;
        std::unordered_set<int> removed;
        std::unordered_set<int> keep;
        std::map<int,int> testmap;

        for(int i = 0;i < kvec.size();i++){

            bool repeat = true;
            if(vvec[i] != maxLoc){
                approx = false;
            }else{
                approx = true;
            }

            if(removed.count(vvec[i]) > 0){

                bool found_pair = false;
                std::vector<int> alt = getTopNonZeroIndices(path_mat, kvec[i], path_mat.rows);
                for(const int a : alt){

                    if((removed.count(a) == 0) and (a != kvec[i])){

                        found_pair = true;
                        vvec[i] = a;
                        break;

                    }

                }

                if(found_pair == false){removed.insert(kvec[i]);}

            }

            if(exists.count(vvec[i]) > 0){

                std::cout <<"\n"<<"value: "<<vvec[i]<<" key: "<<kvec[i]<<"\n";
                std::cout <<"\n"<<"path_mat "<<path_mat<<"\n";
                std::vector<int> stick;
                std::vector<int> ind = getTopNonZeroIndices(path_mat, kvec[i], 2);
                for(const int &v : ind){
                        std::cout <<"\n"<<"test: "<<v<<"\n";
                    if(exists.count(v) > 0){
                        stick.push_back(v);
                        std::cout <<"\n"<<"inserted: "<<v<<"\n";
                    }
                }
                stick.push_back(kvec[i]);
                //stick[0] = kvec[i];
                //stick[1] = vvec[i];
                bool leave = false;

                while(repeat){
                    float ty;
                    float tx;
                    if(approx == false){
                        //transformkp(kp[kvec[i]],Hom_mat_new[testmap[vvec[i]]][vvec[i]]);
                    }

                    adjust_par par = prep_adjust((*T.adj),kp,T,stick,match_mat,vvec[i],Hom_mat,approx);
                    class bundm::adjuster testad(par.kp,par.match,.0001,par.T,false,threads);
                    struct bundm::inter_par teees = testad.iterate();
                    double rep_error = testad.error_value;

                    if(rep_error > 4*error_value){
                        if(approx == false){
                        approx = true;
                        }else{removed.insert(kvec[i]);repeat = false;}
                    }else{
                        //cv::imshow("Image Display",(*Tnew.img_address)[kvec[i]]);
                        //cv::waitKey(0);
                        //cv::imshow("Image Display",(*Tnew.img_address)[vvec[i]]);
                        //cv::waitKey(0);
                        std::cout<<"\n"<<"sticking " << kvec[i]<<"\n";
                        testmap[kvec[i]] = vvec[i];
                        std::cout<<"inserted with error " << rep_error<<"\n";
                        repeat = false;
                        resort_hom(stick,teees.hom,Hom_mat_new);//entferne stickto
                        update_par(testad,Hom_mat_new,T,stick,kp,exists,false,vvec[i]);
                        exists.insert(kvec[i]);



                        std::vector<Eigen::MatrixXd> rret = testad.ret_rot();
                        std::vector<Eigen::MatrixXd> Kret = testad.ret_K();

                        //class imgm::cylhom transformer(800,Kret[0](0,2),Kret[0](1,2),(Tr*Hom_mat_new[vvec[i]][kvec[i]]).inv(),0,0);


                        tx=400 ;
                        ty=150 ;



                        Eigen::MatrixXd rbase;
                        Eigen::MatrixXd rattach;
                        Eigen::MatrixXd kbase;
                        Eigen::MatrixXd kattach;


                        if(kvec[i] < vvec[i]){
                            rbase = rret[0];
                            kbase = Kret[0];
                            kattach = Kret[1];
                            rattach = rret[1];
                        }else{
                            rbase = rret[1];
                            kbase = Kret[1];
                            kattach = Kret[0];
                            rattach = rret[0];
                        }

                        Eigen::MatrixXd test = kbase * rbase *  rattach.transpose() * kattach.inverse();

                        class imgm::cylproj transformer1(rbase,kattach,kbase,(*Tnew.img_address)[kvec[i]].size(),tx,ty);

                        cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[kvec[i]], transformer1,si);

                        img_tr.copyTo(panorama, img_tr);
                        exists.insert(kvec[i]);
                        cv::imshow("Image Display",panorama);
                        cv::waitKey(0);

                    }

                }


            }else{

                keep.insert(kvec[i]);

            }


        }
        std::cout <<"\n"<<"keepsize: "<<keep.size()<<"\n";
        cv::imshow("Image Display",panorama);
        cv::waitKey(0);
        /*
                for(int i = 0;i < ind.size();i++){

            cv::warpPerspective((*Tnew.img_address)[ind[i]],img, Tr*Hom_mat_new[maxLoc][ind[i]],panorama.size(),cv::INTER_LINEAR);
            img.copyTo(panorama, img);

        }
        cv::imshow("Image Display",panorama);
        cv::waitKey(0);
/*

        */




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


adjust_par prep_adjust(const cv::Mat &adj,const std::vector<maths::keypoints> &kp,const class imgm::pan_img_transform &T,std::vector<int> indx,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,int stickto,const std::vector<std::vector<cv::Matx33f>> &Hom_mat,bool approx){

    int size = indx.size();
    std::sort(indx.begin(), indx.end());
    std::vector<maths::keypoints> kp_new;
    std::vector<std::vector<std::vector<cv::DMatch>>> new_match_mat(size,std::vector<std::vector<cv::DMatch>>(size));
    cv::Mat adj_new(indx.size(), indx.size(), adj.type());

    for (int i = 0; i < indx.size(); ++i) {
        for (int j = 0; j < indx.size(); ++j) {

            adj_new.at<double>(i,j) = adj.at<double>(indx[i],indx[j]);
            new_match_mat[i][j] = match_mat[indx[i]][indx[j]];
            //std::cout <<"\n"<<"mmsize: "<<"\n"<<new_match_mat[i][j].size()<<"\n";
        }
    }
    double focal = T.K[stickto](0,0);

    //class imgm::pan_img_transform t_new(&adj_new,T.img_address);
    adjust_par ret(&adj_new,T.img_address);
    ret.adj = adj_new;
    ret.T.adj = &ret.adj;
    ret.T.focal = focal;

    for (int i = 0; i < indx.size(); ++i) {

        kp_new.push_back(kp[indx[i]]);
        if(indx[i] == stickto){

            //std::cout <<"\n"<<"R indx[i] == stickto: "<<"\n"<<Eigen::MatrixXd::Identity(3, 3)<<"\n";
            //std::cout <<"\n"<<"K indx[i] == stickto: "<<"\n"<<T.K[indx[i]]<<"\n";
            ret.T.rot.push_back(Eigen::MatrixXd::Identity(3, 3));
            ret.T.K.push_back(T.K[indx[i]]);

        }else if((approx == false) and (indx[i] != stickto)){
            //std::cout <<"\n"<<"R approx == false: "<<"\n"<<T.rot[stickto]<<"\n";
            //std::cout <<"\n"<<"K approx == false: "<<"\n"<<T.K[stickto]<<"\n";
            ret.T.rot.push_back(T.rot[stickto]);
            ret.T.K.push_back(T.K[stickto]);

        }else{
            Eigen::MatrixXd K = T.K[stickto];
            Eigen::MatrixXd r_approx = approximate(Hom_mat[stickto][indx[i]],K);
            //std::cout <<"\n"<<"r_approx: "<<"\n"<<r_approx<<"\n";
            ret.T.rot.push_back(r_approx);
            ret.T.K.push_back(K);
        }

    }

    ret.kp = kp_new;
    ret.match = new_match_mat;

    return ret;
}


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


void resort_hom(const std::vector<int> &ind,const std::vector<std::vector<cv::Matx33f>> &hom, std::vector<std::vector<cv::Matx33f>> &hom_sorted){

    std::vector<int> index_sorted = ind;
    std::sort(index_sorted.begin(), index_sorted.end());
    for(int i = 0;i < index_sorted.size();i++){
        for(int j = 0;j < index_sorted.size();j++){

            hom_sorted[index_sorted[i]][index_sorted[j]] = hom[i][j];

        }

    }

}

//ändere mit menge statt stickto
void update_par(class bundm::adjuster &adjuster,const std::vector<std::vector<cv::Matx33f>> &hom,class imgm::pan_img_transform &T,const std::vector<int> &ind,std::vector<maths::keypoints> &kp,int stickto,bool update_all){

    std::vector<int> index_sorted = ind;
    std::vector<Eigen::MatrixXd> rotiter = adjuster.ret_rot();
    std::vector<Eigen::MatrixXd> kit = adjuster.ret_K();
    std::sort(index_sorted.begin(), index_sorted.end());
    //std::cout <<"\n"<<"stickto: "<<"\n"<<stickto<<"\n";
    for(int i = 0;i < index_sorted.size();i++){

        if((index_sorted[i] != stickto) or (update_all == true) ){

            transformkp(kp[index_sorted[i]],hom[stickto][index_sorted[i]]);
            T.K[index_sorted[i]] = kit[i];
            T.rot[index_sorted[i]] = rotiter[i];

        }

    }

}

cv::Matx33f get_img_tr(const cv::Matx33f &H,const cv::Mat &attach){


            std::vector<cv::Vec2f> cor;

            cor.push_back(cv::Vec2f(0,0));
            cor.push_back(cv::Vec2f(0,attach.rows));
            cor.push_back(cv::Vec2f(attach.cols,0));
            cor.push_back(cv::Vec2f(attach.cols,attach.rows));

            cv::perspectiveTransform(cor, cor, H);

            std::cout<<"\n"<< "00: "<<cor[1]<<"\n";

            float xstart = std::min( std::min( cor[0][0], cor[1][0]), (float)0);
            float xend   = std::max( std::max( cor[2][0], cor[3][0]), (float)attach.cols);
            float ystart = std::min( std::min( cor[0][1], cor[2][1]), (float)0);
            float yend   = std::max( std::max( cor[1][1], cor[3][1]), (float)attach.rows);

            // create translation matrix
            cv::Matx33f T = cv::Matx33f::zeros();
            T(0, 0) = 1;
            T(1, 1) = 1;
            T(2, 2) = 1;
            T(0, 2) = attach.cols-cor[3][0];
            T(1, 2) = attach.rows-cor[3][1];


    return T;
}

cv::Size get_size(const cv::Mat &img,const cv::Matx33f &hom){

    std::vector<cv::Vec2f> cor;
    cor.push_back(cv::Vec2f(0,0));
    cor.push_back(cv::Vec2f(0,img.rows));
    cor.push_back(cv::Vec2f(img.cols,0));
    cor.push_back(cv::Vec2f(img.cols,img.rows));

    cv::perspectiveTransform(cor, cor, hom);

    float xstart = std::min( std::min( cor[0][0], cor[1][0]), (float)0);
    float xend   = std::max( std::max( cor[2][0], cor[3][0]), (float)img.cols);
    float ystart = std::min( std::min( cor[0][1], cor[2][1]), (float)0);
    float yend   = std::max( std::max( cor[1][1], cor[3][1]), (float)img.rows);
    return cv::Size(xend - xstart + 1, yend - ystart + 1);

}


std::vector<float> angels(const Eigen::MatrixXd &R){

    float x = atan2(R(2,1),R(2,2)) ;
    float y = atan2(-R(2,0),sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2))) ;
    float z = atan2(R(1,0),R(0,0)) ;

    std::vector<float> ang = {x,y,z};
    return ang;
}


void bundleadjust_stitching(class imgm::pan_img_transform &T,class imgm::pan_img_transform &Tnew,const std::vector<std::vector< cv::Matx33f >> &Hom_mat,const std::vector<maths::keypoints> &kpold,const std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,int threads){

    std::vector<maths::keypoints> kp = kpold;
    cv::Mat path_mat = (*T.adj) + (*T.adj).t();
    std::vector<std::vector< cv::Matx33f >> Hom_mat_new(Hom_mat);
    std::vector<double> connectivity = computeRowSumDividedByZeroCount(path_mat);
    int maxLoc = std::distance(connectivity.begin(),std::max_element(connectivity.begin(), connectivity.end()));
    //all neighbours
    std::vector<std::pair<int, std::vector<int>>> tree = maths::bfs_ordered_with_neighbors(path_mat, maxLoc);

    // find n best matches sort the rest by stitching order
    struct bundle_par insert;
    std::map<int, int> tofrom;
    std::map<int, int>::iterator it;

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
        std::unordered_set<int> nearest;
        //std::sort(ind.begin(), ind.end());
        for(int i = 0;i < ind.size();i++){
            nearest.insert(ind[i]);
            exists.insert(ind[i]);
            testtt[ind[i]] = i;

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
                    std::vector<int> indx(2);
                    indx[1] = key;
                    indx[0] = value;


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
        for(int i = 0;i < T.K.size();i++){
            T.K[i] = K;
            T.rot[i] = Eigen::MatrixXd::Identity(3, 3);
        }

        std::cout<<"indis: "<<ind.size()<<"\n";

        adjust_par par = prep_adjust((*T.adj),kp,T,ind,match_mat,maxLoc,Hom_mat,true);
        class bundm::adjuster testad(par.kp,par.match,.0001,par.T,true,threads);
        struct bundm::inter_par teees = testad.iterate();
        double error_value = testad.error_value;

        resort_hom(ind,teees.hom,Hom_mat_new);
        update_par(testad,Hom_mat_new,T,ind,kp,maxLoc,true);
        //teees.focal[2] = 1500;
        cv::Matx33f Tr = cv::Matx33f::eye();
        Tr(0,2) = 200;
        Tr(1,2) = 150;
        cv::Mat panorama = cv::Mat::zeros(2000,3067,CV_32FC3);
        cv::Mat img;
        cv::Size si = panorama.size();

        std::vector<Eigen::MatrixXd> rret = testad.ret_rot();
        std::vector<Eigen::MatrixXd> Kret = testad.ret_K();
/*
        std::cout<<"K[maxLoc]: "<<T.K[maxLoc]<<"\n";
        std::cout<<"T.K[0]: "<<T.K[0]<<"\n";


        class imgm::cylproj transformer(T.rot[maxLoc],T.K[maxLoc],T.K[maxLoc],(*Tnew.img_address)[maxLoc].size());
        cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[maxLoc], transformer,si);
        img_tr.copyTo(panorama, img_tr);
        cv::imshow("Image Display",panorama);
        cv::waitKey(0);

        class imgm::cylproj transformer2(T.rot[ind[0]],T.K[maxLoc],T.K[ind[0]],(*Tnew.img_address)[ind[1]].size());
        cv::Mat img_tr2 = imgm::applyGeometricTransform((*Tnew.img_address)[ind[0]], transformer2,si);
        img_tr2.copyTo(panorama, img_tr2);
        cv::imshow("Image Display",panorama);
        cv::waitKey(0);

        class imgm::cylproj transformer3(T.rot[ind[1]],T.K[maxLoc],T.K[ind[1]],(*Tnew.img_address)[ind[1]].size());
        cv::Mat img_tr3 = imgm::applyGeometricTransform((*Tnew.img_address)[ind[1]], transformer3,si);
        img_tr3.copyTo(panorama, img_tr3);
        cv::imshow("Image Display",panorama);
        cv::waitKey(0);
*/


        for(int i = 0;i < n+1;i++){

            cv::Size sz = get_size((*Tnew.img_address)[ind[i]],Hom_mat_new[maxLoc][ind[i]]);
            cv::Matx33f Ts = get_img_tr(Hom_mat_new[maxLoc][ind[i]],(*Tnew.img_address)[ind[i]]);
            /*
            std::vector<float> ang = angels(T.rot[ind[i]]);
            std::cout<<"rotx "<<"\n"<<ang[0]<<"\n";
            std::cout<<"roty "<<"\n"<<ang[1]<<"\n";
            std::cout<<"rotz "<<"\n"<<ang[2]<<"\n";
            */
            //std::cout<<"translation "<<"\n"<<Ts<<"\n";
            std::cout<<"\n"<<"--------------------------------"<<"\n";
            std::cout<<"T.K[maxLoc] "<<"\n"<<T.K[maxLoc]<<"\n";
            std::cout<<"T.K[ind[i]] "<<"\n"<<T.K[ind[i]]<<"\n";
            std::cout<<"\n"<<"--------------------------------"<<"\n";
            float tx=400 ;
            float ty=150 ;
            class imgm::cylproj transformer(T.rot[ind[i]],T.K[maxLoc],T.K[ind[i]],(*Tnew.img_address)[ind[i]].size(),tx,ty);
            //class imgm::cylhom transformer(teees.focal[testtt[ind[i]]],Kret[testtt[ind[i]]](0,2),Kret[testtt[ind[i]]](1,2),(Tr*Hom_mat_new[maxLoc][ind[i]]).inv(),ang[0],ang[1]);
            cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[ind[i]], transformer,si);

            img_tr.copyTo(panorama, img_tr);
            cv::imshow("Image Display",panorama);
            cv::waitKey(0);

        }





        bool approx = false;
        std::unordered_set<int> removed;
        std::unordered_set<int> keep;
        std::map<int,int> testmap;

        for(int i = 0;i < kvec.size();i++){

            bool repeat = true;
            if(vvec[i] != maxLoc){
                approx = false;
            }else{
                approx = true;
            }

            if(removed.count(vvec[i]) > 0){

                bool found_pair = false;
                std::vector<int> alt = getTopNonZeroIndices(path_mat, kvec[i], path_mat.rows);
                for(const int a : alt){

                    if((removed.count(a) == 0) and (a != kvec[i])){

                        found_pair = true;
                        vvec[i] = a;
                        break;

                    }

                }

                if(found_pair == false){removed.insert(kvec[i]);}

            }

            if(exists.count(vvec[i]) > 0){

                std::cout <<"\n"<<"value: "<<vvec[i]<<" key: "<<kvec[i]<<"\n";
                std::cout <<"\n"<<"path_mat "<<path_mat<<"\n";
                /*
                std::vector<int> stick;
                std::vector<int> ind = getTopNonZeroIndices(path_mat, kvec[i], 1);
                for(const int &v : ind){
                        std::cout <<"\n"<<"test: "<<v<<"\n";
                    if(exists.count(v) > 0){
                        stick.push_back(v);
                        std::cout <<"\n"<<"inserted: "<<v<<"\n";
                    }
                }
                stick.push_back(kvec[i]);
                */
                std::vector<int> stick(2);
                stick[0] = kvec[i];
                stick[1] = vvec[i];
                bool leave = false;

                while(repeat){
                    float ty;
                    float tx;
                    if(approx == false){
                        //transformkp(kp[kvec[i]],Hom_mat_new[testmap[vvec[i]]][vvec[i]]);
                    }

                    adjust_par par = prep_adjust((*T.adj),kp,T,stick,match_mat,vvec[i],Hom_mat,approx);
                    class bundm::adjuster testad(par.kp,par.match,.0001,par.T,false,threads);
                    struct bundm::inter_par teees = testad.iterate();
                    double rep_error = testad.error_value;

                    if(rep_error > 4*error_value){
                        if(approx == false){
                        approx = true;
                        }else{removed.insert(kvec[i]);repeat = false;}
                    }else{
                        //cv::imshow("Image Display",(*Tnew.img_address)[kvec[i]]);
                        //cv::waitKey(0);
                        //cv::imshow("Image Display",(*Tnew.img_address)[vvec[i]]);
                        //cv::waitKey(0);
                        std::cout<<"\n"<<"sticking " << kvec[i]<<"\n";
                        testmap[kvec[i]] = vvec[i];
                        std::cout<<"inserted with error " << rep_error<<"\n";
                        repeat = false;
                        resort_hom(stick,teees.hom,Hom_mat_new);//entferne stickto
                        update_par(testad,Hom_mat_new,T,stick,kp,false,vvec[i]);
                        exists.insert(kvec[i]);



                        std::vector<Eigen::MatrixXd> rret = testad.ret_rot();
                        std::vector<Eigen::MatrixXd> Kret = testad.ret_K();

                        //class imgm::cylhom transformer(800,Kret[0](0,2),Kret[0](1,2),(Tr*Hom_mat_new[vvec[i]][kvec[i]]).inv(),0,0);


                        tx=400 ;
                        ty=150 ;



                        Eigen::MatrixXd rbase;
                        Eigen::MatrixXd rattach;
                        Eigen::MatrixXd kbase;
                        Eigen::MatrixXd kattach;


                        if(kvec[i] < vvec[i]){
                            rbase = rret[0];
                            kbase = Kret[0];
                            kattach = Kret[1];
                            rattach = rret[1];
                        }else{
                            rbase = rret[1];
                            kbase = Kret[1];
                            kattach = Kret[0];
                            rattach = rret[0];
                        }

                        Eigen::MatrixXd test = kbase * rbase *  rattach.transpose() * kattach.inverse();

                        class imgm::cylproj transformer1(rbase,kattach,kbase,(*Tnew.img_address)[kvec[i]].size(),tx,ty);

                        cv::Mat img_tr = imgm::applyGeometricTransform((*Tnew.img_address)[kvec[i]], transformer1,si);

                        img_tr.copyTo(panorama, img_tr);
                        exists.insert(kvec[i]);
                        cv::imshow("Image Display",panorama);
                        cv::waitKey(0);

                    }

                }


            }else{

                keep.insert(kvec[i]);

            }


        }
        std::cout <<"\n"<<"keepsize: "<<keep.size()<<"\n";
        cv::imshow("Image Display",panorama);
        cv::waitKey(0);


}


}


