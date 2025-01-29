

#include "_img_manipulation.h"


namespace imgm {

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
            T(0, 0) = 1;
            T(1, 1) = 1;
            T(2, 2) = 1;
            T(0, 2) = -xstart;
            T(1, 2) = -ystart;

            T = T * H;
            // warp second image
            cv::Mat panorama;
            cv::warpPerspective(attach, panorama, T, cv::Size(xend - xstart + 1, yend - ystart + 1), cv::INTER_LINEAR);

            // copy base image panorama
            cv::Mat roi(panorama, cv::Rect(-xstart,-ystart,base.cols, base.rows));
            base.copyTo(roi, base);

            return panorama;
}


}

