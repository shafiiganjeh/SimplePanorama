
#ifndef PAN_H
#define PAN_H

#include "_panorama.h"
#include <opencv2/core/eigen.hpp>


namespace pan{

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
    //images_to_cylinder(1500);
        calculate_keypoints(threads);
        get_adj_par(threads);

        std::vector<struct maths::adj_str> adddd;

        adddd = maths::extract_adj(adj);

        //std::cout <<"\n"<<"ad mat: "<<adj<<"\n";
        std::cout <<"\n"<<"ad size: "<<adddd.size()<<"\n";
        //std::cout <<"\n"<<"ad size: "<<adddd[0].adj<<"\n";

        imgm::pan_img_transform Tr(&adj,&img_data);

        imgm::pan_img_transform Tr2(&adj,&img_data);

        Eigen::Matrix3d tr;
        tr << 1,0,0,0,1,50,0,0,1;

        Tr.focal = conf.focal;
        imgm::calc_stitch_from_adj(Tr,hom_mat);
/*
        class bundm::adjuster testad(keypnts,match_mat,adj,conf.lambda,Tr);
        struct bundm::inter_par teees = testad.iterate();

        imgm::calc_stitch_from_adj(Tr2,teees.hom);

        std::vector<Eigen::MatrixXd> Ksd = testad.ret_K();




        std::vector<Eigen::MatrixXd> Ks = testad.ret_K();
        std::vector<Eigen::MatrixXd> rot = testad.ret_rot();

        //Ks[0](0,2) = 0;
        //Ks[0](1,2) = 0;
        //Ks[1](0,2) = 0;
        //Ks[1](1,2) = 0;

        Eigen::MatrixXd homeig =   Ks[1] * rot[1] * rot[0].transpose() * Ks[0].inverse();
        homeig = homeig / homeig(2,2);



        std::cout<<"\n"<< "Ks[0]: "<<Ks[0]<<"\n";
        std::cout<<"\n"<< "rot[0]: "<<rot[0]<<"\n";
        std::cout<<"\n"<< "rot[1]: "<<rot[1]<<"\n";
        std::cout<<"\n"<< "Ks[1].inverse(): "<<Ks[1].inverse()<<"\n";

        Eigen::Vector3d p0;
        p0 <<0,0,1;

        p0 = homeig * p0;

        std::cout<<"\n"<<"zeropo"<< p0;

        cv::Mat cvhom2;

        cv::eigen2cv(homeig,cvhom2);

        cv::Mat dest2;
        std::cout<<"\n"<< "cvhom2: "<<cvhom2<<"\n";

        dest2 = imgm::stitch(img_data[0], img_data[1],cvhom2);
*/
/*
        cv::imshow("Image Display", dest2);
        cv::waitKey(0);

*/
      // teees.focal;

     //imgm::calc_stitch_from_adj(Tr2,teees.hom);
    // for(int i = 0;i<teees.focal.size();i++){
  //       std::cout <<"\n"<<"focal: "<<teees.focal[i]<<"\n";

  //  }




/*
        cv::Mat panorama = imgm::stitch(img_data[0], img_data[1],teees.hom[0][1]);
        maths::translation Ti = maths::get_translation(img_data[0], img_data[1],teees.hom[0][1]);
        panorama = imgm::stitch(panorama, img_data[2],Ti.T*teees.hom[0][2]);



        cv::imshow("Image Display", panorama);
        cv::waitKey(0);


for (int i =0;i<teees.focal.size();i++){

    std::cout<<"\n"<<teees.focal[i];
}
        if (conf.gain_compensation == false){



        }
*/
        images_to_cylinder(1500);
        switch(conf.blend) {
            case SIMPLE_BLEND:

                blnd::simple_blend(Tr,img_data);

            case NO_BLEND:

            default:
            break;
        }
    }

}
#endif
