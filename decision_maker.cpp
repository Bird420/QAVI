#include "decision_maker.h"

cv::vector<cv::Vec3f> SemiCircleDetection::detectSemiCircle(cv::Mat src);
{
    cv::vector<cv::Vec3f> circles;
    cv::Mat src_gray;
    if(!src.data) {
        exit(1);
    }
    cvtColor( src, src_gray, CV_BGR2GRAY );
    GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
    HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
    return circles;
}
