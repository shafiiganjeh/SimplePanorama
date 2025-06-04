
#ifndef PAN_H
#define PAN_H

#include "_panorama.h"
#include <opencv2/core/eigen.hpp>
#include <limits>
#include  "full_bunde.cpp"
namespace pan{



void getEulerAnglesZYX(const Eigen::Matrix3d& R, double& yaw, double& pitch, double& roll) {
    // Calculate pitch (around Y-axis)
    pitch = std::asin(-R(2, 0));

    // Check for gimbal lock (cos(pitch) near zero)
    double cos_pitch = std::cos(pitch);
    const double epsilon = std::numeric_limits<double>::epsilon();

    if (cos_pitch > epsilon) {
        // No gimbal lock
        yaw = std::atan2(R(1, 0) / cos_pitch, R(0, 0) / cos_pitch);
        roll = std::atan2(R(2, 1) / cos_pitch, R(2, 2) / cos_pitch);
    } else {
        // Gimbal lock case
        yaw = std::atan2(-R(0, 1), R(1, 1));
        roll = 0.0; // Arbitrary choice
    }
}


    void panorama::load_resized(int width){

        for(auto& elem : f_list) {

            cv::Mat temp_img = imgm::file_to_cv(elem);

            temp_img = imgm::resizeKeepAspectRatio(temp_img, width);
            img_data.push_back( temp_img );

        }

    }

    cv::Mat panorama::get_adj(){

        return adj;

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

    std::vector<cv::Mat> warped_images;
    std::vector<cv::Point> corners;
    std::vector<cv::Size> sizes;

    const int w_ref = images[0].cols;
    const int h_ref = images[0].rows;

    // Step 2: Warp images and record positions
    for (size_t i = 0; i < images.size(); ++i) {
        // Convert Eigen matrices to OpenCV format
        cv::Mat cvK, cvR;
        cv::Ptr<cv::detail::SphericalWarper> warper = cv::makePtr<cv::detail::SphericalWarper>(K[i](0, 0));
        Eigen::Matrix3d R_adj = R[i];
        Eigen::Matrix3d hom = K[ref]*K[i].inverse();
        Eigen::Vector3d pp;
        pp << w_ref/2,h_ref/2,1;
        pp = hom * pp;
        pp = pp/pp[2];

        const double f_i = K[i](0, 0);
        const double s = focal / f_i;
        const double c_x = K[i](0, 2);
        const double c_y = K[i](1, 2);
        const int w_i = K[ref](0, 2);
        const int h_i = K[ref](1, 2);

        Eigen::Matrix3d K_adj;
        K_adj << f_i, 0,w_ref - c_x,
                 0, f_i,h_ref - c_y,
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


    std::vector<std::vector<std::vector<cv::DMatch>>> panorama::get_match_mat(){

        return match_mat;

    }


    std::vector<std::vector< cv::Matx33f >> panorama::get_hom_mat(){

        return hom_mat;

    }


    void panorama::get_adj_par(int threads){


            class maths::adj_calculator ctest(img_data,keypnts);

            ctest.get_threads(threads);
            std::vector<std::thread> thread_vector;

            for(int i = 0;i < threads;i++){

                std::thread tobj(&maths::adj_calculator::cal_adj,&ctest,img_data,i);
                thread_vector.push_back(std::move(tobj));

            }


            for(int i = 0;i < threads;i++){

                thread_vector[i].join();

            }

            adj = ctest.adj;
            hom_mat = ctest.hom_mat;
            match_mat = ctest.match_mat;

        }


    Eigen::MatrixXd rotate(double angle,double x,double y){

        double sinus = sin(angle);
        double cosinus = cos(angle);

        Eigen::Matrix3d tr;
        tr << 1,0,x,0,1,y,0,0,1;

        Eigen::Matrix3d mtr;
        mtr << 1,0,-x,0,1,-y,0,0,1;

        Eigen::Matrix3d rot;
        rot << cosinus,-sinus, 0 , sinus, cosinus,0,0,0,1;

        rot = tr * rot * mtr;

        return rot;

    }


    void panorama::create_panorama(int threads,struct config conf){


        load_images();

        if (conf.bundle_adjust == false){



        }
    //images_to_cylinder(600);
        int threadsimg;
        if(threads > img_data.size()){
            threadsimg = img_data.size();
        }else{
            threadsimg = threads;
        }
        calculate_keypoints(threadsimg);
        get_adj_par(threadsimg);

        std::vector<struct maths::adj_str> adddd;

        adddd = maths::extract_adj(adj);

        //std::cout <<"\n"<<"ad size: "<<adddd[0].adj<<"\n";

        imgm::pan_img_transform Tr(&adj,&img_data);

        imgm::pan_img_transform Tr2(&adj,&img_data);

        Eigen::Matrix3d tr;
        tr << 1,0,0,0,1,50,0,0,1;

        Tr.focal = conf.focal;
        imgm::calc_stitch_from_adj(Tr,hom_mat,match_mat,keypnts);
        stch::bundleadjust_stitching(Tr,Tr2,hom_mat,keypnts,match_mat,threads);


        std::cout <<"\n"<<"k1: "<<Tr.K[0]<<"\n";
        std::cout <<"\n"<<"k2: "<<Tr.K[1]<<"\n";
        std::cout <<"\n"<<"r1: "<<Tr.rot[0]<<"\n";
        std::cout <<"\n"<<"r2: "<<Tr.rot[1]<<"\n";

        class bundm::adjuster testad(keypnts,match_mat,conf.lambda,Tr,threads);
        struct bundm::inter_par teees = testad.iterate();
        std::vector<Eigen::MatrixXd> Ks = testad.ret_K();
        std::vector<Eigen::MatrixXd> rot = testad.ret_rot();


        std::cout << "augm: " << testad.timer.getDuration("augm") << " seconds\n";
        std::cout << "itpar: " << testad.timer.getDuration("itpar") << " seconds\n";
        std::cout << "gete: " << testad.timer.getDuration("gete") << " seconds\n";
        std::cout << "total: " << testad.timer.getDuration("total for") << " seconds\n";

/*
        struct full::parameters parf = full::main_fkt(keypnts,match_mat,Tr.K,Tr.rot,adj,0);

        Eigen::MatrixXd Tre = Eigen::Matrix3d::Identity();

        Tre(0,2) = img_data[0].rows/2;
        Tre(1,2) = img_data[0].cols/2;


        for (int i = 0;i < adj.rows;i++){

            std::cout<<parf.K[i]<<"\n";
            parf.K[i] = Tre*parf.K[i];

        }

        int locind = 0;
        cv::Mat imgpan = projectToCylinder(
            img_data,
            parf.R,
            parf.K,
            locind
        );

        cv::imshow("Image Display", imgpan);
        cv::waitKey(0);


/*


        cv::Mat panorama = cv::Mat::zeros(1800,4067,CV_32FC3);
        cv::Size si = panorama.size();

        for(int i = 0;i<rot.size();i++){


            Eigen::MatrixXd Hom = (Ks[0] * rot[0] * rot[i].transpose() * Ks[i].inverse());
            cv::Mat homcv;
            cv::Matx33f eeeye = cv::Matx33f::eye();

            cv::eigen2cv(Hom,homcv);

            class imgm::cylhom h1(homcv);
            class imgm::cylhom h2(eeeye);
            cv::Mat img_tr2 = imgm::applyGeometricTransform((*Tr2.img_address)[i], h1,si);
            cv::Mat img_tr3 = imgm::applyGeometricTransform((*Tr2.img_address)[0], h2,si);

            img_tr3.copyTo(panorama, img_tr3);
            img_tr2.copyTo(panorama, img_tr2);

        }

        cv::imshow("Image Display",panorama);
        cv::waitKey(0);



        int locind = 0;
        cv::Mat imgpan = projectToCylinder(
            img_data,
            rot,
            Ks,
            locind
        );

        cv::imshow("Image Display", imgpan);
        cv::waitKey(0);
        //class bundm::adjuster testad(keypnts,match_mat,conf.lambda,Tr);
        //struct bundm::inter_par teees = testad.iterate();


        std::vector<std::vector< cv::Matx33f >> hoom = stch::bundleadjust_stitching(Tr,Tr2,hom_mat,keypnts,match_mat,threads);

        class bundm::adjuster testad(keypnts,match_mat,conf.lambda,Tr);
        struct bundm::inter_par teees = testad.iterate();
        std::vector<Eigen::MatrixXd> Ks = testad.ret_K();
        std::vector<Eigen::MatrixXd> rot = testad.ret_rot();


        imgm::calc_stitch_from_adj(Tr2,teees.hom,match_mat,keypnts);


        int s = 0;
        for (int i = 0;i < adj.rows;i++){
            for(int j = 0;j < adj.cols;j++){

                if (0 <  adj.at<double>(i,j)){
                    s = s + match_mat[i][j].size();
                }
            }
        }
        std::cout<<"\n"<<"matchsize: "<<s<<"\n";

        //struct bundm::inter_par teees = testad.iterate();

        Ks = testad.ret_K();
        rot = testad.ret_rot();



        locind = 0;
        imgpan = projectToCylinder(
            (*Tr.img_address),
            rot,
            Ks,
            locind
        );
        cv::imshow("Image Display", imgpan);
        cv::waitKey(0);

        imgm::calc_stitch_from_adj(Tr2,hoom,match_mat,keypnts);

        blnd::no_blend(Tr2,img_data);
*/
/*




/*
for (int i =0;i<teees.focal.size();i++){

    std::cout<<"\n"<<teees.focal[i];
}
        if (conf.gain_compensation == false){



        }

        //images_to_cylinder(teees.focal,Tr2.im_center);
        switch(conf.blend) {
            case SIMPLE_BLEND:

                blnd::no_blend(Tr2,img_data);

            case NO_BLEND:

            default:
            break;
        }
*/

    }

}
#endif
