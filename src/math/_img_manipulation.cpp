
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

                std::vector<int> size(2);
                size[0] = mt.rows;
                size[1] = mt.cols;
                img_dimensions.push_back(size);

            }

    }


    void elementwiseOperation(const cv::Mat& A, const cv::Mat& B, cv::Mat& result, Operation op){

        constexpr float EPS = 1e-6f;

        CV_Assert(A.type() == CV_32FC3);
        CV_Assert(B.type() == CV_32FC1);
        CV_Assert(A.size() == B.size());
        CV_Assert(op == MULTIPLY or op == DIVIDE);

        if (result.empty() or result.size() != A.size() or result.type() != A.type()) {
            result.create(A.size(), A.type());
        }

        if (A.isContinuous() and B.isContinuous() and result.isContinuous()) {
            int totalPixels = A.rows * A.cols;
            const cv::Vec3f* aPtr = A.ptr<cv::Vec3f>(0);
            const float* bPtr = B.ptr<float>(0);
            cv::Vec3f* resPtr = result.ptr<cv::Vec3f>(0);

            if (op == MULTIPLY) {
                for (int i = 0; i < totalPixels; ++i) {
                    resPtr[i] = aPtr[i] * bPtr[i];
                }
            }
            else {
                for (int i = 0; i < totalPixels; i++) {
                    float divisor = bPtr[i];
                    divisor = std::copysign(std::max(std::abs(divisor), EPS), divisor);
                    resPtr[i] = aPtr[i] / divisor;
                }
            }
        }

        else {
            for (int r = 0; r < A.rows; r++) {
                const cv::Vec3f* aRow = A.ptr<cv::Vec3f>(r);
                const float* bRow = B.ptr<float>(r);
                cv::Vec3f* resRow = result.ptr<cv::Vec3f>(r);

                if (op == MULTIPLY) {
                    for (int c = 0; c < A.cols; ++c) {
                        resRow[c] = aRow[c] * bRow[c];
                    }
                }
                else {
                    for (int c = 0; c < A.cols; c++) {
                        float divisor = bRow[c];
                        divisor = std::copysign(std::max(std::abs(divisor), EPS), divisor);
                        resRow[c] = aRow[c] / divisor;
                    }
                }
            }
        }
    }

    //resize pad boarders
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

            return image;
    }


/*legacy
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
*/

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


//calculate flat projection panorama (unused, mostly legacy).
void calc_stitch_from_adj(pan_img_transform &T,const std::vector<std::vector< cv::Matx33f >> &Hom,std::vector<std::vector<std::vector<cv::DMatch>>> &match_mat,std::vector<util::keypoints> &keypnts){

    std::unordered_set<int> visited;
    cv::Mat path_mat = (*T.adj) + (*T.adj).t();

    std::vector<double> connectivity = computeRowSumDividedByZeroCount(path_mat);

    int maxLoc = std::distance(connectivity.begin(),std::max_element(connectivity.begin(), connectivity.end()));

    std::vector<std::pair<int, std::vector<int>>> tree = util::bfs_ordered_with_neighbors(path_mat, maxLoc);

    std::map<int, std::pair<int,double>> paths = util::path_table(*T.adj,tree ,maxLoc);

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
    cv::Size test_size;

    cv::Mat translation = cv::Mat::eye(3,3, CV_32F);
    std::cout<<"tree[0].first "<<tree[0].first<<"\n";
    for (const std::pair<int, std::vector<int>> &node : tree){
        visited.insert(node.first);

        for (const int &node_visit : node.second){

            if(visited.count(node_visit) == 0){
                cv::Mat H = cv::Mat::eye(3,3, CV_32F);

                visited.insert(node_visit);
                int node_current = node_visit;

                std::vector<int> pt;
                while(-1 != paths[node_current].first){

                    H = (Hom[paths[node_current].first][node_current])*H;

                    pt.push_back(paths[node_current].first);
                    pt.push_back(node_current);

                    std::cout<<"paths[node_current].first: "<<paths[node_current].first<<" nodevisit: "<<node_current<<"\n";

                    node_current = paths[node_current].first;


                }

                T.pair_stitch.push_back(pt);

                T.H_1j[node_visit] = H;

                util::translation Tr;
                H = translation*H;

                T.stitch_order.push_back(node_visit);

                H = H / H.at<float>(2,2);
                Tr = util::get_translation(test_size, (*T.img_address)[node_current],H);
                test_size.height =  Tr.yend - Tr.ystart + 1;
                test_size.width = Tr.xend - Tr.xstart + 1;

                if(test_size.height > 30000 or test_size.width > 30000){

                    test_size.height = std::numeric_limits<int>::quiet_NaN();
                    test_size.width = std::numeric_limits<int>::quiet_NaN();

                }

                std::cout<<"\n"<<"flat size"<<test_size<<"\n";

                cv::Mat Hinv = Tr.T*H;

                T.img2pan[node_visit] = Hinv;
                Hinv = Hinv.inv();
                T.pan2img[node_visit] = Hinv;

                T.translation[node_visit] = Tr;
                translation = Tr.T*translation;

            }
        }
    }

    T.flat_pan_dim = test_size;
    T.H_1j[T.stitch_order[0]] = cv::Matx33f::eye();

    for (int i = T.stitch_order.size() - 1 ; i > 0 ;i-- ){

        for (int j = i-1;j >= 0 ;j--  ){

            T.img2pan[T.stitch_order[j]] = T.translation[T.stitch_order[i]].T * T.img2pan[T.stitch_order[j]];

        }
    }

    for(int i = 0 ; i < path_mat.rows;i++ ){

        T.rot.push_back( Eigen::Matrix3d::Identity() );
        T.K.push_back( K);

    }

}


}

