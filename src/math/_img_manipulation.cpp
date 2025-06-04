
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

    std::pair<float, float> cylhom::inv(float x, float y){
            // Inverse rotation transformation

            x = x - 1000;
            y = y - 500;

            float map_x = H.val[0]*x+H.val[1]*y+H.val[2];
            float map_y = H.val[3]*x+H.val[4]*y+H.val[5];
            float divisor = H.val[6]*x+H.val[7]*y+H.val[8];

            map_x = map_x/divisor;
            map_y = map_y/divisor;

            return {map_x, map_y};
        }

        std::pair<float, float> cylproj::inv(float x, float y){


            float x_;
            float y_;
            float z_;

            float theta = (x -tx - (cx_a )) / (f_a);

            float zx = f_a*sin(theta);
            float zy = (y -ty- (cy_a ));
            float zz = f_a*cos(theta);

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


            if (abs(x) < 2*PI*f_b ) {
                map_x = f_a * zx + 400;
                map_y = f_a * zy + 300;
            }else{

                return {-1, -1};

            }


            return {map_x, map_y};
        }



        std::pair<float, float> cylproj::forward(float x, float y){

            float zx = x;
            float zy = y;
            float zz = 1;

            float x_ = kb_inv[0]*zx+kb_inv[1]*zy+kb_inv[2]*zz;
            float y_ = kb_inv[3]*zx+kb_inv[4]*zy+kb_inv[5]*zz;
            float z_ = kb_inv[6]*zx+kb_inv[7]*zy+kb_inv[8]*zz;

            zx = x_;
            zy = y_;
            zz = z_;

            x_ = v_f[0]*zx+v_f[1]*zy+v_f[2]*zz;
            y_ = v_f[3]*zx+v_f[4]*zy+v_f[5]*zz;
            z_ = v_f[6]*zx+v_f[7]*zy+v_f[8]*zz;

            float theta;

            theta = atan2(x_,z_);
            float h = y_/sqrt(x_*x_ + z_*z_);

            float map_x = (f_a * theta +cx_a);
            float map_y = f_a * h + cy_a;


            return {map_x, map_y};
        }

/*
        std::pair<float, float> cylproj::inv(float x, float y){

            float x_;
            float y_;
            float z_;

            z_ = kb[6]*x+kb[7]*y+kb[8]*1;
            x_ = kb[0]*x+kb[1]*y+kb[2]*1;
            y_ = kb[3]*x+kb[4]*y+kb[5]*1;

            float zx = sin(x_/ z_);
            float zy = y_ / z_;
            float zz = cos(x_/ z_);

            z_ = v[6]*zx+v[7]*zy+v[8]*zz;

            if ((z_ <= 0)) {

                return {-1, -1};

            }

            x_ = v[0]*zx+v[1]*zy+v[2]*zz;
            y_ = v[3]*zx+v[4]*zy+v[5]*zz;


            zx = x_;
            zy = y_;
            zz = z_;

            z_ = ka[6]*zx+ka[7]*zy+ka[8]*zz;
            x_ = ka[0]*zx+ka[1]*zy+ka[2]*zz;
            y_ = ka[3]*zx+ka[4]*zy+ka[5]*zz;

            zx = x_ / z_;
            zy = y_ / z_;

            float map_x;
            float map_y;


            if (abs(x-tx) < 2*PI*f_b ) {
                map_x = zx;
                map_y = zy;
            }else{

                map_x = -1;
                map_y = -1;

            }


            return {map_x, map_y};
        }

*/
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

            // warp second image
            cv::Mat panorama;
            cv::warpPerspective(attach, panorama, T, cv::Size(xend - xstart + 1, yend - ystart + 1), cv::INTER_LINEAR);

            cv::imshow("Image Display", panorama);
            cv::waitKey(0);

            cv::Mat roi(panorama, cv::Rect(-xstart,-ystart,base.cols, base.rows));
            base.copyTo(roi, base);

            cv::imshow("Image Display", panorama);
            cv::waitKey(0);

            return panorama;
}

    cv::Mat project(const cv::Mat& imags,float xc,float yc,float f,cv::Matx33f hom){

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



void calc_stitch_from_adj(pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >> &Hom,std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,std::vector<maths::keypoints> &keypnts){

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
    //K << T.focal,0,0,0,T.focal,0,0,0,1;

    cv::Mat translation = cv::Mat::eye(3,3, CV_32F);
    std::map<int, Eigen::MatrixXd> rotations;
    std::map<int, Eigen::MatrixXd> Ks;
    rotations[tree[0].first] = Eigen::MatrixXd::Identity(3,3);
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

                std::vector<int> pt;
                Eigen::MatrixXd R_H = Eigen::MatrixXd::Identity(3,3);
                while(-1 != paths[node_current].first){

                    Eigen::MatrixXd H_cast;
                    cv::Matx33d homd = static_cast<cv::Matx33d>(Hom[paths[node_current].first][node_current]);
                    cv::cv2eigen(homd,H_cast);
                    Eigen::MatrixXd R_app = maths::approximate(H_cast,                          K,keypnts[paths[node_current].first].keypoint,keypnts[node_current].keypoint,match_mat[paths[node_current].first][node_current],Eigen::Matrix3d::Identity());
                    R_H = R_app*R_H;

                    H = (Hom[paths[node_current].first][node_current])*H;

                    pt.push_back(paths[node_current].first);
                    pt.push_back(node_current);

                    std::cout<<"paths[node_current].first: "<<paths[node_current].first<<" nodevisit: "<<node_current<<"\n";

                    node_current = paths[node_current].first;


                }

                rotations[node_visit] = R_H.transpose();
                T.pair_stitch.push_back(pt);

                T.H_1j[node_visit] = H;

                Ks[node_visit] = K;

                maths::translation Tr;
                H = translation*H;


                T.stitch_order.push_back(node_visit);

                H = H / H.at<float>(2,2);
                Tr = maths::get_translation(panorama, (*T.img_address)[node_current],H);
                //panorama.create(Tr.yend - Tr.ystart + 1, Tr.xend - Tr.xstart + 1, panorama.type());
                panorama.create(10, 10, panorama.type());

                std::cout<<"\n"<<"size"<<panorama.size()<<"\n";

                cv::Mat Hinv = Tr.T*H;

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

    }

    for(int i = 0 ; i < path_mat.rows;i++ ){
        if (rotations.count(i)){
            T.rot.push_back( rotations[i] );
            T.K.push_back( K);

        }else{
            T.rot.push_back( Eigen::Matrix3d::Identity() );
            T.K.push_back( Eigen::Matrix3d::Identity() );
        }

    }

}





}

