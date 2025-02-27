
#include "_img_manipulation.h"


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

    cv::Mat stitch(const cv::Mat &base, const cv::Mat &attach,const cv::Matx33f &H){
            // compute corners

            std::vector<cv::Vec2f> cor;

            cor.push_back(cv::Vec2f(0,0));
            cor.push_back(cv::Vec2f(0,attach.rows));
            cor.push_back(cv::Vec2f(attach.cols,0));
            cor.push_back(cv::Vec2f(attach.cols,attach.rows));

            cv::perspectiveTransform(cor, cor, H);

            float xstart = std::min( std::min( cor[0][0], cor[1][0]), (float)0);
            float xend   = std::max( std::max( cor[2][0], cor[3][0]), (float)base.cols);
            float ystart = std::min( std::min( cor[0][1], cor[2][1]), (float)0);
            float yend   = std::max( std::max( cor[1][1], cor[3][1]), (float)base.rows);

            // create translation matrix
            cv::Matx33f T = cv::Matx33f::zeros();
            cv::Matx33f id = cv::Matx33f::eye();
            T(0, 0) = 1;
            T(1, 1) = 1;
            T(2, 2) = 1;
            T(0, 2) = -xstart;
            T(1, 2) = -ystart;
            std::cout <<"translation "<<T<<"\n";

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

//imags
//adj
void calc_stitch_from_adj(pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >> &Hom){

    pan_img_transform pan_var(T.adj,T.img_address );

    std::unordered_set<int> visited;
    cv::Mat row_sum;

    cv::reduce(*T.adj, row_sum, 1, cv::REDUCE_SUM, CV_64F);
    double min=0, max=0;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(row_sum, &min, &max, &minLoc, &maxLoc);

    std::vector<std::pair<int, std::vector<int>>> tree = maths::bfs_ordered_with_neighbors(*T.adj, maxLoc.y);
    std::map<int, std::pair<int,double>> paths = maths::path_table(*T.adj,tree ,maxLoc.y);

    cv::Mat panorama((*T.img_address)[maxLoc.y].size(),(*T.img_address)[maxLoc.y].type());//placeholder
    panorama = (*T.img_address)[maxLoc.y];
    //Mat1f m(rows, cols);

    T.stitch_order.push_back(maxLoc.y);
    T.img2pan[maxLoc.y] = cv::Matx33f::eye();
    T.pan2img[maxLoc.y] = cv::Matx33f::eye();
    T.translation[maxLoc.y].T = cv::Matx33f::eye();
    T.translation[maxLoc.y].xstart = 0;
    T.translation[maxLoc.y].xend = (*T.img_address)[maxLoc.y].cols;
    T.translation[maxLoc.y].ystart = 0;
    T.translation[maxLoc.y].yend = (*T.img_address)[maxLoc.y].rows;

    Eigen::MatrixXd K(3,3);
    K << T.focal,0,0,0,T.focal,0,0,0,1;

    cv::Mat translation = cv::Mat::eye(3,3, CV_32F);
    std::map<int, Eigen::MatrixXd> rotations;
    rotations[tree[0].first] = Eigen::MatrixXd::Identity(3,3);

    for (const std::pair<int, std::vector<int>> &node : tree){
        visited.insert(node.first);

        //std::cout <<"node.first "<<node.first<<"\n";

        for (const int &node_visit : node.second){

            if (!(rotations.count(node_visit))){

                //std::cout <<"node_visit "<<node_visit<<"\n";
                Eigen::MatrixXd H_eigen;

                cv::cv2eigen(static_cast<cv::Matx33d>(Hom[node.first][node_visit]),H_eigen);

                Eigen::MatrixXd R_H = K.inverse() * H_eigen * K ;//* rotations[node.first];
                //std::cout <<"rh "<<K<<"\n";
                R_H = ( R_H * R_H.transpose() ).pow(0.5) * (R_H.transpose()).inverse();
                rotations[node_visit] = R_H;
            }

            if(visited.count(node_visit) == 0){
                cv::Mat H = cv::Mat::eye(3,3, CV_32F);

                visited.insert(node_visit);

                int node_current = node_visit;
                while(-1 != paths[node_current].first){

                    H = (Hom[paths[node_current].first][node_current])*H;
                    node_current = paths[node_current].first;

                }
                T.H_1j[node_visit] = H;

                maths::translation Tr;
                H = translation*H;

                T.stitch_order.push_back(node_visit);

                Tr = maths::get_translation(panorama, (*T.img_address)[node_current],H);
                panorama.create(Tr.yend - Tr.ystart + 1, Tr.xend - Tr.xstart + 1, panorama.type());

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

    for(int i = 0 ; i < rotations.size();i++ ){

        T.rot.push_back( rotations[i] );

    }

}

}

