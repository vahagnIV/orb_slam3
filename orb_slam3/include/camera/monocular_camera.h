//
// Created by vahagn on 1/23/21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_MONOCULAR_CAMERA_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_MONOCULAR_CAMERA_H_

#include <g2o/core/base_vertex.h>
#include <typedefs.h>
#include <map/key_point.h>

namespace orb_slam3 {
namespace camera {

#define DistCoeffsLength 5

class MonocularCamera : public g2o::BaseVertex<DistCoeffsLength + 4, Eigen::VectorXd> {
 public:
  typedef Eigen::Matrix<double, DistCoeffsLength + 4, 1> Estimate_t;
  typedef Eigen::Matrix<double, DistCoeffsLength, 1> DistCoeffs_t;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  MonocularCamera(unsigned width, unsigned height) : width_(width), height_(height) {}

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
   * Width of the image
   * @return unsigned
   */
  unsigned Width() const { return width_; }

  /*!
   * Height of the image
   * @return
   */
  unsigned Height() const { return height_; }

  /*!
   * Projects a 3D point on the image plane using the camera model
   * @param vector The 3D point in the camera frame
   * @return 2D image point
   */
  TPoint2D Map(const TPoint3D & vector) const ;

  void UndistortKeyPoints(const std::vector<map::KeyPoint> & keypoints, std::vector<map::KeyPoint> & out_undistorted_keypoints) const;


 protected:
  void ComputeDistortion(const double x, const double y, double & xd, double & yd) const;

 protected:
  unsigned width_;
  unsigned height_;

};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_MONOCULAR_CAMERA_H_
