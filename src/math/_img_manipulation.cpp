
#include "_img_manipulation.h"

namespace imgm {

cv::Mat resize_image( const cv::Mat& img, int target_width)
{
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

}

