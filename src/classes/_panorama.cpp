
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


    void panorama::create_panorama(int threads,struct config conf){


        load_images();

        if (conf.bundle_adjust == false){

            //images_to_cylinder(1200);

        }
        //images_to_cylinder(3000);
        calculate_keypoints(threads);
        get_adj_par(threads);

        std::vector<struct maths::adj_str> adddd;

        adddd = maths::extract_adj(adj);

        //std::cout <<"\n"<<"ad mat: "<<adj<<"\n";
        std::cout <<"\n"<<"ad size: "<<adddd.size()<<"\n";
        //std::cout <<"\n"<<"ad size: "<<adddd[0].adj<<"\n";

        imgm::pan_img_transform Tr(&adj,&img_data);
        imgm::pan_img_transform Tr2(&adj,&img_data);



        Tr.focal = conf.focal;
        imgm::calc_stitch_from_adj(Tr,hom_mat);

       class bundm::adjuster testad(keypnts,match_mat,adj,conf.lambda,Tr);
       struct bundm::inter_par teees = testad.iterate();
       std::vector<Eigen::MatrixXd> Ks = testad.ret_K();
       std::vector<Eigen::MatrixXd> rot = testad.ret_rot();

       Eigen::MatrixXd homeig = rot[0] * Ks[0].inverse();
       std::cout<<"\n" <<"rot: " <<rot[0]<<"\n";
       std::cout<<"\n" <<"k: " <<Ks[0]<<"\n";

       cv::Mat cvhom;
       cv::eigen2cv(homeig,cvhom);

       cv::Mat dest;


/*
       std::vector<cv::Vec2f> cor;
        cor.push_back(cv::Vec2f(0,0));
        cor.push_back(cv::Vec2f(0,img_data[0].rows));
        cor.push_back(cv::Vec2f(img_data[0].cols,0));
        cor.push_back(cv::Vec2f(img_data[0].cols,img_data[0].rows));

        cv::perspectiveTransform(cor, cor, cvhom);

        float xstart = std::min( std::min( cor[0][0], cor[1][0]), (float)0);
        float xend   = std::max( std::max( cor[2][0], cor[3][0]), (float)img_data[0].cols);
        float ystart = std::min( std::min( cor[0][1], cor[2][1]), (float)0);
        float yend   = std::max( std::max( cor[1][1], cor[3][1]), (float)img_data[0].rows);

        std::cout<<"\n" <<"size: " <<cv::Size(xend - xstart + 1, yend - ystart + 1)<<"\n";




       cv::warpPerspective(img_data[0],dest,cvhom,cv::Size(img_data[0].cols+100, img_data[0].rows +100), cv::INTER_LINEAR);
       cv::imshow("Image Display", dest);
       cv::waitKey(0);
*/

       teees.focal;

     imgm::calc_stitch_from_adj(Tr2,teees.hom);
    // for(int i = 0;i<teees.focal.size();i++){
  //       std::cout <<"\n"<<"focal: "<<teees.focal[i]<<"\n";

  //  }
     //images_to_cylinder(2000);



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
        switch(conf.blend) {
            case SIMPLE_BLEND:

                blnd::no_blend(Tr2,img_data);

            case NO_BLEND:

            default:
            break;
        }
    }

}
#endif
