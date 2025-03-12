
#include "_img_manipulation.h"
#include <math.h>
#define PI 3.14159265

namespace imgm {

    pan_img_transform::pan_img_transform(const cv::Mat* adj_mat,const std::vector<cv::Mat> *imags){

            H_1j.resize(adj_mat->cols);
            img2pan.resize(adj_mat->cols);
            pan2img.resize(adj_mat->cols);
            translation.resize(adj_mat->cols);

            img_address = imags;
            adj = adj_mat;

            for (const cv::Mat& mt : *imags){

                std::vector<float> size(2);
                size[0] = mt.rows;
                size[1] = mt.cols;
                img_dimensions.push_back(size);

            }



    }

    cv::Mat resize_image( const cv::Mat& img, int target_width){
            int width = img.cols,
            height = img.rows;

            cv::Mat square = cv::Mat( target_width, target_width, img.type() ) ;
            square.setTo(cv::Scalar(255, 255, 255));

            float scale = ( ( float ) target_width ) / (( width >= height ) ? width : height);

            cv::Rect roi;
            if ( width >= height ){
                roi.width = target_width;
                roi.x = 0;
                roi.height = height * scale;
                roi.y = ( target_width - roi.height ) / 2;
            }
            else{
                roi.y = 0;
                roi.height = target_width;
                roi.width = width * scale;
                roi.x = ( target_width - roi.width ) / 2;
            }

            cv::resize( img, square( roi ), roi.size(),0,0,cv::INTER_AREA);

            return square;
    }


    cv::Mat resizeKeepAspectRatio(const cv::Mat& input, int desiredWidth) {

            int originalWidth = input.cols;
            int originalHeight = input.rows;

            double scale = static_cast<double>(desiredWidth) / originalWidth;
            int desiredHeight = cvRound(originalHeight * scale);

            int interpolation = (desiredWidth > originalWidth) ?
                                cv::INTER_LINEAR :   // Enlarging
                                cv::INTER_AREA;       // Shrinking


            cv::Mat resizedImage;
            cv::resize(input, resizedImage, cv::Size(desiredWidth, desiredHeight), 0, 0, interpolation);

            return resizedImage;
    }


    cv::Mat file_to_cv(std::string path){

            std::ifstream file(path, std::ios::binary);
            cv::Mat image;

            if (!file.is_open()) {
                std::cerr << "Error: Cannot open the file at " << path << std::endl;
                return image;
            }

            file.seekg(0, std::ios::end);
            int size = file.tellg();
            file.seekg(0, std::ios::beg);

            std::vector<unsigned char> buffer(size);
            if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
                std::cerr << "Error: Unable to read the file into the buffer" << std::endl;
                return image;
            }

            image = cv::imdecode(buffer, cv::IMREAD_COLOR);
            if (image.empty()) {
                std::cerr << "Error: Image decoding failed" << std::endl;
            }

            //cv::imshow("Image Display", image);
            //cv::waitKey(0);
            return image;
    }


    cv::Matx33d rotate(double angle,double x,double y){

        double sinus = sin(angle);
        double cosinus = cos(angle);

        Eigen::Matrix3d tr;
        tr << 1,0,x,0,1,y,0,0,1;

        Eigen::Matrix3d mtr;
        mtr << 1,0,-x,0,1,-y,0,0,1;

        Eigen::Matrix3d rot;
        rot << cosinus,-sinus, 0 , sinus, cosinus,0,0,0,1;

        rot = tr * rot * mtr;
        cv::Mat rotcv;

        cv::eigen2cv(rot,rotcv);

        return rotcv;

    }


    cv::Matx33f angle_correction(const cv::Mat &base,const cv::Matx33f &H){


            double px = base.cols / 2;
            double py = base.rows / 2;
            cv::Vec3d u(base.cols/2, base.rows/2, 1);
            cv::Vec3d v(base.cols, base.rows/2, 1);

            px = H(0,0)/H(2,0) - px;
            py = H(1,0)/H(2,0) - py;
            double angle = atan(py/px);

            if(angle != angle){
                cv::Matx33f ret = cv::Matx33f::eye();
                return ret;
            }

            double px_tr = (base.cols / 2) * sqrt(H(0,0)*H(0,0) + H(1,0)*H(1,0));
            double py_tr = (base.cols / 2) * sqrt(H(0,0)*H(0,0) + H(1,0)*H(1,0));

            u = H * u;
            u = u / u[2];

            cv::Matx33d rota;
            rota = rotate(-1*angle,u[0],u[1]);
            std::cout<<"\n"<<" rotpre "<<v<<"\n";
            v = rota * v;
            std::cout<<"\n"<<" rot "<<v<<"\n";

            double Th = v[1] - (base.rows/2);
            double Tw = v[0] - base.cols;
            std::cout<<"\n"<<"l: "<<(base.cols / 2)<<" lt "<<px_tr<<"\n";
            cv::Matx33d T = cv::Matx33d::zeros();
            T(0, 0) = 1;
            T(1, 1) = 1;
            T(2, 2) = 1;
            T(0, 2) = Tw;
            T(1, 2) = -Th;
            std::cout<<"\n"<<"Th: "<<Th<<" Tw "<<Tw<<"\n";
            rota =  T * rota ;

            cv::Matx33f matFloat = static_cast<cv::Matx33f>(rota);

            return matFloat;
    }


    cv::Mat stitch(const cv::Mat &base, const cv::Mat &attach,const cv::Matx33f &H){
            // compute corners

            std::vector<cv::Vec2f> cor;

            cor.push_back(cv::Vec2f(0,0));
            cor.push_back(cv::Vec2f(0,attach.rows));
            cor.push_back(cv::Vec2f(attach.cols,0));
            cor.push_back(cv::Vec2f(attach.cols,attach.rows));

            cv::perspectiveTransform(cor, cor, H);

            std::cout<<"\n"<< "00: "<<cor[1]<<"\n";

            float xstart = std::min( std::min( cor[0][0], cor[1][0]), (float)0);
            float xend   = std::max( std::max( cor[2][0], cor[3][0]), (float)base.cols);
            float ystart = std::min( std::min( cor[0][1], cor[2][1]), (float)0);
            float yend   = std::max( std::max( cor[1][1], cor[3][1]), (float)base.rows);

            // create translation matrix
            cv::Matx33f T = cv::Matx33f::zeros();
            T(0, 0) = 1;
            T(1, 1) = 1;
            T(2, 2) = 1;
            T(0, 2) = -xstart;
            T(1, 2) = -ystart;
            std::cout<<"\n"<< "H: "<<H<<"\n";
            //std::cout <<"translation "<<T<<"\n";

            T = T * H;

            //T = H;
            // warp second image
            cv::Mat panorama;
            cv::warpPerspective(attach, panorama, T, cv::Size(xend - xstart + 1, yend - ystart + 1), cv::INTER_LINEAR);


            cv::Mat roi(panorama, cv::Rect(-xstart,-ystart,base.cols, base.rows));
            base.copyTo(roi, base);

            return panorama;
}

    cv::Mat project(const cv::Mat& imags,float xc,float yc,float f){

            const int rows = imags.rows;
            const int cols = imags.cols;
            const int size = std::max(rows,cols);


            std::vector<float> v_123(size);
            std::iota(v_123.begin(), v_123.end(), 0);
            std::vector<float> v_111(size,1);

            Eigen::Matrix<float, 1, Eigen::Dynamic> a(Eigen::Map<Eigen::RowVectorXf> (v_123.data(),size));
            Eigen::Matrix<float, Eigen::Dynamic, 1> b(Eigen::Map<Eigen::RowVectorXf> (v_123.data(),size));

            Eigen::MatrixXf map_x(cols, rows);
            Eigen::MatrixXf map_y(rows, cols);
            Eigen::MatrixXf temp_x(cols, rows);

            map_x = a(Eigen::all,Eigen::seq(0,cols - 1)).replicate(1,rows);
            temp_x = a(Eigen::all,Eigen::seq(0,cols - 1)).replicate(1,rows);
            map_y = a(Eigen::all,Eigen::seq(0,rows - 1)).replicate(cols,1);

            //map_y = a.replicate(rows,1);


            for (int i = 0 ; i<rows*cols;i++){

                map_y(i) = (map_y(i)-yc)/cos((map_x(i)-xc)/f) + yc;
                map_x(i) = tan((map_x(i)-xc)/f) * f + xc;

            }

/*
        for (int i = 0 ; i<rows*cols;i++){

            map_x(i) =f/(cos((map_y(i)-yc)/f)*cos((map_x(i)-xc)/f))  * ((cos((map_y(i)-yc)/f)*sin((map_x(i)-xc)/f))) + xc;
            map_y(i)= f/(cos((map_y(i)-yc)/f)*cos((temp_x(i)-xc)/f))  * sin((map_y(i)-yc)/f) + yc;

        }
*/
            cv::Mat dst(imags.size(), imags.type());
            cv::Mat vec_x(imags.size(), CV_32FC1,map_x.data());
            cv::Mat vec_y(imags.size(), CV_32FC1,map_y.data());

        //std::cout << vec_x<<"\n";
        //std::cout << vec_y<<"\n";

            cv::remap(imags, dst,vec_x, vec_y, cv::INTER_AREA);

            return dst;
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



void calc_stitch_from_adj(pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >> &Hom){

    std::unordered_set<int> visited;
    cv::Mat path_mat = (*T.adj) + (*T.adj).t();

    std::vector<double> connectivity = computeRowSumDividedByZeroCount(path_mat);

    int maxLoc = std::distance(connectivity.begin(),std::max_element(connectivity.begin(), connectivity.end()));


    std::vector<std::pair<int, std::vector<int>>> tree = maths::bfs_ordered_with_neighbors(path_mat, maxLoc);

    std::map<int, std::pair<int,double>> paths = maths::path_table(*T.adj,tree ,maxLoc);

    cv::Mat panorama((*T.img_address)[maxLoc].size(),(*T.img_address)[maxLoc].type());//placeholder

    T.stitch_order.push_back(maxLoc);
    T.img2pan[maxLoc] = cv::Matx33f::eye();
    T.pan2img[maxLoc] = cv::Matx33f::eye();
    T.translation[maxLoc].T = cv::Matx33f::eye();
    T.translation[maxLoc].xstart = 0;
    T.translation[maxLoc].xend = (*T.img_address)[maxLoc].cols;
    T.translation[maxLoc].ystart = 0;
    T.translation[maxLoc].yend = (*T.img_address)[maxLoc].rows;

    Eigen::MatrixXd K(3,3);
    K << T.focal,0,0,0,T.focal,0,0,0,1;

    cv::Mat translation = cv::Mat::eye(3,3, CV_32F);
    std::map<int, Eigen::MatrixXd> rotations;
    std::map<int, Eigen::MatrixXd> Ks;
    rotations[tree[0].first] = Eigen::MatrixXd::Identity(3,3) ;
    Ks[tree[0].first] = K;
    std::cout<<"tree[0].first "<<tree[0].first<<"\n";
    for (const std::pair<int, std::vector<int>> &node : tree){
        visited.insert(node.first);

        //std::cout <<"node.first "<<node.first<<"\n";


        for (const int &node_visit : node.second){

            if(visited.count(node_visit) == 0){
                cv::Mat H = cv::Mat::eye(3,3, CV_32F);

                visited.insert(node_visit);

                int node_current = node_visit;
                while(-1 != paths[node_current].first){

                    H = (Hom[paths[node_current].first][node_current])*H;

                    //std::cout<<"paths[node_current].first "<<paths[node_current].first<<"nodevisit "<<node_current<<"\n";

                    if (!(rotations.count(node_visit))){

                        //std::cout <<"node_visit "<<node_visit<<"\n";
                        Eigen::MatrixXd H_eigen;

                        cv::cv2eigen(static_cast<cv::Matx33d>(Hom[node.first][node_visit]),H_eigen);

                        K(0,2) = H_eigen(0,2);
                        K(1,2) = H_eigen(1,2);
                        std::cout <<"Ks"<<K;
                        Eigen::MatrixXd R_H = K.inverse() * H_eigen * K ;//* rotations[node.first];
                        //std::cout <<"rh "<<K<<"\n";
                        R_H = ( R_H * R_H.transpose() ).pow(0.5) * (R_H.transpose()).inverse();
                        //std::cout<<"\n"<<"rotation: "<<node_visit<<" "<<R_H<<"\n";
                        rotations[node_visit] = R_H;
                        Ks[node_visit] = K;
                    }

                    node_current = paths[node_current].first;


                }

                T.H_1j[node_visit] = H;

                maths::translation Tr;
                H = translation*H;


                T.stitch_order.push_back(node_visit);

                H = H / H.at<float>(2,2);
                Tr = maths::get_translation(panorama, (*T.img_address)[node_current],H);
                panorama.create(Tr.yend - Tr.ystart + 1, Tr.xend - Tr.xstart + 1, panorama.type());

                std::cout<<"\n"<<"size"<<panorama.size()<<"\n";

                cv::Mat Hinv = Tr.T*H;

                Ks[node_visit](0,2) = Tr.T(0,2);
                Ks[node_visit](1,2) = Tr.T(1,2);

                T.img2pan[node_visit] = Hinv;
                Hinv = Hinv.inv();
                T.pan2img[node_visit] = Hinv;


                T.translation[node_visit] = Tr;
                translation = Tr.T*translation;

            }
        }
    }
    //cv::imshow("Image Display",panorama);
    //cv::waitKey(0);
    T.H_1j[T.stitch_order[0]] = cv::Matx33f::eye();

    for (int i = T.stitch_order.size() - 1 ; i > 0 ;i-- ){

        for (int j = i-1;j >= 0 ;j--  ){

            T.img2pan[T.stitch_order[j]] = T.translation[T.stitch_order[i]].T * T.img2pan[T.stitch_order[j]];

        }
    }

    cv::Vec3f cent_img1 = {(float)(*T.img_address)[T.stitch_order[0]].cols / 2,(float)(*T.img_address)[T.stitch_order[0]].rows / 2,1};
    cent_img1 = T.img2pan[T.stitch_order[0]] * cent_img1;
    cent_img1 = cent_img1 / cent_img1[2];

    for(int i = 0;i <T.img2pan.size();i++){

            cv::Vec3f cent = {(float)(*T.img_address)[i].cols / 2,(float)(*T.img_address)[i].rows / 2,1};
            cent = T.img2pan[i] * cent;
            cent = cent / cent[2];
            cent = cent_img1 - cent;
            cv::Vec2f center;
            center[0] = cent[0];
            center[1] = cent[1];
            T.im_center.push_back(center);

            std::cout <<"\n"<<"center "<<center<<"\n";
            //cv::Matx33f R_c = angle_correction((*T.img_address)[i],T.img2pan[i]);
            //T.img2pan[i] = R_c * T.img2pan[i] ;
    }

    for(int i = 0 ; i < path_mat.rows;i++ ){
        if (rotations.count(i)){
            T.rot.push_back( rotations[i] );
            T.K.push_back( Ks[i] );
        }else{
            T.rot.push_back( Eigen::Matrix3d::Zero() );
            T.K.push_back( Eigen::Matrix3d::Zero() );
        }

    }

}


pan_img_transform bundleadjust_stitching(pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >> &Hom){

    pan_img_transform T_bundle(T.adj,T.img_address);




    return T_bundle;

}

}

