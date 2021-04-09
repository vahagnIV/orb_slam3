//
// Created by vahagn on 1/23/21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_MONOCULAR_CAMERA_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_MONOCULAR_CAMERA_H_

// === optimization ===
#include <g2o/core/base_vertex.h>

// == orb-slam3 ===
#include <typedefs.h>
#include <features/key_point.h>
#include "idistortion_model.h"
#include "icamera.h"

namespace orb_slam3 {
namespace camera {

#ifndef DISTORTION_MODEL_PARAMS_MAX
#define DISTORTION_MODEL_PARAMS_MAX 10
#endif

class MonocularCamera
    : public ICamera, protected g2o::BaseVertex<DISTORTION_MODEL_PARAMS_MAX + CAMERA_PARAMS_COUNT, Eigen::VectorXd> {
 public:

  typedef decltype(_estimate)::Scalar Scalar;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  MonocularCamera(unsigned width, unsigned height) :
      width_(width),
      height_(height),
      distortion_model_(nullptr) {

    setEstimate(Eigen::Matrix<double, DISTORTION_MODEL_PARAMS_MAX + 4, 1>::Zero());
  }
  virtual ~MonocularCamera() { delete distortion_model_; }

 public: // ====  optimization =============

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
    fx_inv_ = _estimate[0] ? 1 / _estimate[0] : 1;
    fx_inv_ = _estimate[1] ? 1 / _estimate[1] : 1;
  }

 public:

  template<typename TDistortionModel>
  TDistortionModel * CreateDistortionModel() {
    if (DISTORTION_MODEL_PARAMS_MAX < TDistortionModel::DistrortionSize + CAMERA_PARAMS_COUNT) {
      throw std::runtime_error("Please increase DISTORTION_MODEL_PARAMS_MAX and rebuild the program...");
    }

    delete distortion_model_;
    TDistortionModel * model = new TDistortionModel(&_estimate);
    distortion_model_ = model;
    return model;
  }

  const IDistortionModel * GetDistortionModel() const override {
    return distortion_model_;
  }

  void ComputeJacobian(const TPoint3D & pt,
                       ProjectionJacobianType & out_jacobian) const override;


  bool IsInFrustum(const HomogenousPoint & point) const override;

  bool IsInScaleInvarianceRegion(const TPoint3D & point) const override;

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

  /*!
   * Projects a 3D point on the image plane using the camera model
   * @param point3d The 3D point in the camera frame
   * @return 2D image point
   */
  TPoint2D Map(const TPoint3D & point3d) const;

  /*!
   * Undistorts keypoint
   * @param points input
   * @param undistorted_points output
   */
  bool UndistortPoint(const TPoint2D & point, TPoint2D & undistorted_point) const;
  bool DistortPoint(const TPoint2D & undistorted, TPoint2D & distorted) const;
  void UnprojectPoint(const TPoint2D & point, HomogenousPoint & unprojected) const;
  void ProjectPoint(const TPoint3D & point, TPoint2D & projected) const;
  void ProjectAndDistort(const TPoint3D & point, TPoint2D & out_projected) const;
  bool UnprojectAndUndistort(const TPoint2D & point, HomogenousPoint & unprojected) const;

  inline const Scalar & Fx() const noexcept { return this->_estimate[0]; }
  inline const Scalar & Fy() const noexcept { return this->_estimate[1]; }
  inline const Scalar & Cx() const noexcept { return this->_estimate[2]; }
  inline const Scalar & Cy() const noexcept { return this->_estimate[3]; }
  inline const Scalar & FxInv() const noexcept { return fx_inv_; }
  inline const Scalar & FyInv() const noexcept { return fy_inv_; }

  void SetFx(Scalar fx) noexcept {
    _estimate[0] = fx;
    fx_inv_ = fx ? 1 / fx : 0;
  }
  void SetFy(Scalar fy) noexcept {
    _estimate[1] = fy;
    fy_inv_ = fy ? 1 / fy : 1;
  }
  void SetCx(Scalar cx) noexcept { _estimate[2] = cx; }
  void SetCy(Scalar cy) noexcept { _estimate[3] = cy; }

#if DistCoeffsLength == 8
  inline const double & K4() noexcept { return _estimate[9] ; }
  inline const double & K5() noexcept { return _estimate[10]; }
  inline const double & K6() noexcept { return _estimate[11]; }
#endif

#if DistCoeffsLength == 8
  void SetK4(precision_t k4) noexcept { _estimate[9] = k4; }
  void SetK5(precision_t k5) noexcept { _estimate[10] = k5; }
  void SetK6(precision_t k6) noexcept { _estimate[11] = k6; }
#endif

 protected:
  unsigned width_;
  unsigned height_;
  double min_X_, max_X_, min_Y_, max_Y_;
  double fx_inv_, fy_inv_;
  IDistortionModel * distortion_model_;

};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_MONOCULAR_CAMERA_H_
