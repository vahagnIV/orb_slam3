#ifndef ORB_SLAM3_INCLUDE_IMAGE_UTILS_H_
#define ORB_SLAM3_INCLUDE_IMAGE_UTILS_H_

#include <typedefs.h>

namespace orb_slam3 {
namespace image_utils {

precision_t Approximate(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                        Eigen::Vector3d& p3, float px, float py) {
  Eigen::Matrix<double, 3, 3> c;
  c << p1, p2, p3;
  // std::cout << c << std::endl;
  Eigen::Matrix<double, 3, 1> identity;
  identity << 1, 1, 1;
  Eigen::Matrix<double, 3, 1> coefficients = c.transpose().inverse() * identity;
  return (1 - coefficients[0] * px - coefficients[1] * py) / coefficients[2];
}

template <typename TI, typename TO>
void ResizeImage(
    const Eigen::Matrix<TI, Eigen::Dynamic, Eigen::Dynamic>& in,
    Eigen::Matrix<TO, Eigen::Dynamic, Eigen::Dynamic>& out_resized) {
  for (size_t row = 0; row < out_resized.rows(); ++row) {
    for (size_t col = 0; col < out_resized.cols(); ++col) {
      double pre_y =
          static_cast<float>(in.rows() - 1) / (out_resized.rows() - 1) * row;
      double pre_x =
          static_cast<float>(in.cols() - 1) / (out_resized.cols() - 1) * col;
      int nearest_x = std::round(pre_x);
      int nearest_y = std::round(pre_y);
      int top_left_x = pre_x;
      int top_left_y = pre_y;

      Eigen::Vector3d p1, p2, p3;
      if (std::abs(pre_x - nearest_x) < 1e-7 ||
          std::abs(pre_y - nearest_y) < 1e-7) {
        out_resized(row, col) = in(top_left_y, top_left_x);
        continue;
      } else if (nearest_x == top_left_x) {
        if (nearest_y == top_left_y) {
          p1 << top_left_x, top_left_y, in(top_left_y, top_left_x);
          p2 << top_left_x + 1, top_left_y, in(top_left_y, top_left_x + 1);
          p3 << top_left_x, top_left_y + 1, in(top_left_y + 1, top_left_x);
        } else {
          p1 << top_left_x, top_left_y, in(top_left_y, top_left_x);
          p2 << top_left_x + 1, top_left_y + 1,
              in(top_left_y + 1, top_left_x + 1);
          p3 << top_left_x, top_left_y + 1, in(top_left_y + 1, top_left_x);
        }
      } else {
        if (nearest_y == top_left_y) {
          p1 << top_left_x, top_left_y, in(top_left_y, top_left_x);
          p2 << top_left_x + 1, top_left_y, in(top_left_y, top_left_x + 1);
          p3 << top_left_x + 1, top_left_y + 1,
              in(top_left_y + 1, top_left_x + 1);
        } else {
          p1 << top_left_x, top_left_y + 1, in(top_left_y + 1, top_left_x);
          p2 << top_left_x + 1, top_left_y, in(top_left_y, top_left_x + 1);
          p3 << top_left_x + 1, top_left_y + 1,
              in(top_left_y + 1, top_left_x + 1);
        }
      }

      float val = Approximate(p1, p2, p3, pre_x, pre_y);
      out_resized(row, col) = std::max(std::min(val, 255.f), 0.f);
    }
  }
}

}  // namespace image_utils
}  // namespace orb_slam3
#endif  // ORB_SLAM3_INCLUDE_IMAGE_UTILS_H_