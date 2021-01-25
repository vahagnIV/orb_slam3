#ifndef ORB_SLAM3_INCLUDE_IMAGE_UTILS_H_
#define ORB_SLAM3_INCLUDE_IMAGE_UTILS_H_

// https://chao-ji.github.io/jekyll/update/2018/07/19/BilinearResize.html

#include <typedefs.h>

namespace orb_slam3 {
namespace image_utils {

template <typename TI, typename TO>
void ResizeImage(const Eigen::Matrix<TI, Eigen::Dynamic, Eigen::Dynamic>& in,
                 Eigen::Matrix<TO, Eigen::Dynamic, Eigen::Dynamic>& out_resized,
                 size_t edge_left, size_t edge_right, size_t edge_top,
                 size_t edge_bottom) {


  double s_y = static_cast<double>(in.rows() - 1) / (out_resized.rows() - 1 - edge_top - edge_bottom);
  double s_x = static_cast<double>(in.cols() - 1) / (out_resized.cols() - 1 - edge_left - edge_right);

  for (size_t row = 0; row < out_resized.rows() - edge_left - edge_right; ++row) {
#pragma omp parallel for
    for (size_t col = 0; col < out_resized.cols() - edge_top - edge_bottom; ++col) {
      double pre_y = s_y * row;
      double pre_x = s_x * col;
      int top_left_x = pre_x;
      int top_left_y = pre_y;

      double w_x = (pre_x - top_left_x);
      double w_y = (pre_y - top_left_y);
      TO & val = out_resized(row + edge_top, col + edge_right);
      if (top_left_x == in.cols() - 1 && top_left_y == in.rows() - 1) {
        val = in(top_left_y, top_left_x);
      } else if (top_left_x == in.cols() - 1) {
        val = (1 - w_y) * in(top_left_y, top_left_x) +
                                w_y * in(top_left_y + 1, top_left_x);
      } else if (top_left_y == in.rows() - 1) {
        val = (1 - w_x) * in(top_left_y, top_left_x) +
                                w_x * in(top_left_y, top_left_x + 1);
      } else {
        double X = (1 - w_x) * in(top_left_y, top_left_x) +
                   w_x * in(top_left_y, top_left_x + 1);
        double Y = (1 - w_x) * in(top_left_y + 1, top_left_x) +
                   w_x * in(top_left_y + 1, top_left_x + 1);
         val = (1 - w_y) * X + w_y * Y;
      }
    }
  }
}

}  // namespace image_utils
}  // namespace orb_slam3
#endif  // ORB_SLAM3_INCLUDE_IMAGE_UTILS_H_