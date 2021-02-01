//
// Created by vahagn on 1/23/21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_MONOCULAR_CAMERA_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_MONOCULAR_CAMERA_H_

#include <g2o/core/base_vertex.h>
#include <typedefs.h>
#include <features/key_point.h>

namespace orb_slam3 {
namespace camera {

#define DistCoeffsLength 5

class MonocularCamera : public g2o::BaseVertex<DistCoeffsLength + 4, Eigen::VectorXd> {
 public:
  typedef Eigen::Matrix<double, DistCoeffsLength + 4, 1> Estimate_t;
  typedef Eigen::Matrix<double, DistCoeffsLength, 1> DistCoeffs_t;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  MonocularCamera(unsigned width, unsigned height) :
      width_(width),
      height_(height) {

    setEstimate(Estimate_t::Zero());
  }

 public: // ====  g2o =============

  bool read(std::istream & /*is*/) override {
    assert(!"Read is not Implemented yet");
    return false;
  }

  bool write(std::ostream & /*os*/) const override {
    assert(!"Write is not Implemented yet");
    return false;
  }

  void setToOriginImpl() override {
    assert(!"Set to Origin is not Implemented yet");
  }

  void oplusImpl(const double * update) override {
    Eigen::VectorXd::ConstMapType v(update, MonocularCamera::Dimension);
    this->_estimate += v;
  }

 public:
  /*!
   *
   */
  void ComputeImageBounds();

  /*!
   * Width of the image
   * @return unsigned
   */
  unsigned Width() const { return width_; }

  /*!
   * Height of the image
   * @return
   */
  unsigned Height() const { return height_; }

  inline const precision_t & ImageBoundMinX() const { return min_X_; }
  inline const precision_t & ImageBoundMinY() const { return min_Y_; }
  inline const precision_t & ImageBoundMaxX() const { return max_X_; }
  inline const precision_t & ImageBoundMaxY() const { return max_Y_; }
  inline const precision_t & GridElementWidthInv() const { return grid_element_width_inv_; }
  inline const precision_t & GridElementHeightInv() const { return grid_element_height_inv_; }

  /*!
   * Projects a 3D point on the image plane using the camera model
   * @param vector The 3D point in the camera frame
   * @return 2D image point
   */
  TPoint2D Map(const TPoint3D & vector) const;

  /*!
   * Undistorts keypoint
   * @param keypoints input
   * @param out_undistorted_keypoints output
   */
  void UndistortKeyPoints(const std::vector<features::KeyPoint> & keypoints,
                          std::vector<features::KeyPoint> & out_undistorted_keypoints) const;

  precision_t Fx() noexcept { return _estimate[0]; }
  precision_t Fy() noexcept { return _estimate[1]; }
  precision_t Cx() noexcept { return _estimate[2]; }
  precision_t Cy() noexcept { return _estimate[3]; }
  precision_t K1() noexcept { return _estimate[4]; }
  precision_t K2() noexcept { return _estimate[5]; }
  precision_t P1() noexcept { return _estimate[6]; }
  precision_t P2() noexcept { return _estimate[7]; }
  precision_t K3() noexcept { return _estimate[8]; }

#if DistCoeffsLength == 8
  precision_t K4() noexcept { return _estimate[9] ; }
  precision_t K5() noexcept { return _estimate[10]; }
  precision_t K6() noexcept { return _estimate[11]; }
#endif

  void SetFx(precision_t fx) noexcept { _estimate[0] = fx; }
  void SetFy(precision_t fy) noexcept { _estimate[1] = fy; }
  void SetCx(precision_t cx) noexcept { _estimate[2] = cx; }
  void SetCy(precision_t cy) noexcept { _estimate[3] = cy; }
  void SetK1(precision_t k1) noexcept { _estimate[4] = k1; }
  void SetK2(precision_t k2) noexcept { _estimate[5] = k2; }
  void SetP1(precision_t p1) noexcept { _estimate[6] = p1; }
  void SetP2(precision_t p2) noexcept { _estimate[7] = p2; }
  void SetK3(precision_t k3) noexcept { _estimate[8] = k3; }

#if DistCoeffsLength == 8
  void SetK4(precision_t k4) noexcept { _estimate[9] = k4; }
  void SetK5(precision_t k5) noexcept { _estimate[10] = k5; }
  void SetK6(precision_t k6) noexcept { _estimate[11] = k6; }
#endif

 protected:
  void ComputeDistortion(const double x, const double y, double & xd, double & yd) const;

 protected:
  unsigned width_;
  unsigned height_;
  precision_t min_X_, max_X_, min_Y_, max_Y_;
  precision_t grid_element_width_inv_, grid_element_height_inv_;

};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_MONOCULAR_CAMERA_H_
