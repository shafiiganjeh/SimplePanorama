
#include "_img_manipulation.h"


namespace imgm {

    pan_img_transform::pan_img_transform(const cv::Mat& adj){

            img2pan.resize(adj.cols);
            pan2img.resize(adj.cols);
            translation.resize(adj.cols);

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

            T = T * H;
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

class pan_img_transform calc_stitch_from_adj(const std::vector<cv::Mat> &imags,const std::vector<std::vector< cv::Matx33f >> &Hom,const cv::Mat& adj){

    pan_img_transform pan_var(adj);

    std::unordered_set<int> visited;
    cv::Mat row_sum;

    cv::reduce(adj, row_sum, 1, cv::REDUCE_SUM, CV_64F);
    double min=0, max=0;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(row_sum, &min, &max, &minLoc, &maxLoc);

    std::vector<std::pair<int, std::vector<int>>> tree = maths::bfs_ordered_with_neighbors(adj, maxLoc.y);
    std::map<int, std::pair<int,double>> paths = maths::path_table(adj,tree ,maxLoc.y);

    cv::Mat panorama(imags[maxLoc.y].size(),imags[maxLoc.y].type());//placeholder
    panorama = imags[maxLoc.y];
    //Mat1f m(rows, cols);

    pan_var.stitch_order.push_back(maxLoc.y);
    pan_var.img2pan[maxLoc.y] = cv::Matx33f::eye();
    pan_var.pan2img[maxLoc.y] = cv::Matx33f::eye();
    pan_var.translation[maxLoc.y].T = cv::Matx33f::eye();
    pan_var.translation[maxLoc.y].xstart = 0;
    pan_var.translation[maxLoc.y].xend = imags[maxLoc.y].cols;
    pan_var.translation[maxLoc.y].ystart = 0;
    pan_var.translation[maxLoc.y].yend = imags[maxLoc.y].rows;


    cv::Mat translation = cv::Mat::eye(3,3, CV_32F);


    for (const std::pair<int, std::vector<int>> &node : tree){
        visited.insert(node.first);

        for (const int &node_visit : node.second){

            if(visited.count(node_visit) == 0){
                cv::Mat H = cv::Mat::eye(3,3, CV_32F);
                visited.insert(node_visit);

                int node_current = node_visit;
                while(-1 != paths[node_current].first){

                    H = (Hom[paths[node_current].first][node_current])*H;
                    node_current = paths[node_current].first;

                }
                maths::translation Tr;
                H = translation*H;

                pan_var.stitch_order.push_back(node_visit);
                //pan_var.img2pan[node_visit] = H;

                //panorama = stitch(panorama, imags[node_visit],H);

                Tr = maths::get_translation(panorama, imags[node_current],H);
                panorama.create(Tr.yend - Tr.ystart + 1, Tr.xend - Tr.xstart + 1, panorama.type());

                cv::Mat Hinv = Tr.T*H;
                pan_var.img2pan[node_visit] = Hinv;
                Hinv = Hinv.inv();
                pan_var.pan2img[node_visit] = Hinv;


                pan_var.translation[node_visit] = Tr;
                translation = Tr.T*translation;

            }
        }
    }
    //cv::imshow("Image Display",panorama);
    //cv::waitKey(0);
    for (int i = pan_var.stitch_order.size() - 1 ; i > 0 ;i-- ){

        for (int j = i-1;j >= 0 ;j--  ){

            pan_var.img2pan[pan_var.stitch_order[j]] = pan_var.translation[pan_var.stitch_order[i]].T * pan_var.img2pan[pan_var.stitch_order[j]];

        }
    }

    return pan_var;

}

}

