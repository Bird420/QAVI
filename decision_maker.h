#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace SemiCircleDetection
{
  /**
   * @brief function to detect semicircle
   *
   * Will detect all semicircle in a cv::Mat
   *
   * @param  src The pictire matrix
   * @return vector of Vec3f (x, y, radius of center)
   */
    cv::vector<cv::Vec3f> detectSemiCircle(cv::Mat src);
}

class DecisionMaker
{
};
