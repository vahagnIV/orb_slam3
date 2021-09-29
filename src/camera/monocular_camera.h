//
// Created by vahagn on 1/23/21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_MONOCULAR_CAMERA_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_MONOCULAR_CAMERA_H_

// === optimization ===
#include <g2o/core/base_vertex.h>

// == orb-slam3 ===
#include "../typedefs.h"
#include "../features/key_point.h"
#include "src/camera/distortions/idistortion_model.h"
#include "icamera.h"

namespace orb_slam3 {
namespace camera {

#ifndef DISTORTION_MODEL_PARAMS_MAX
#define DISTORTION_MODEL_PARAMS_MAX 10
#endif

class MonocularCamera
    : public ICamera {
 public:

  MonocularCamera(unsigned width, unsigned height) :
      width_(width),
      height_(height),
      distortion_model_(nullptr) {
  }

  MonocularCamera() :
      width_(0),
      height_(0),
      distortion_model_(nullptr) {
  }
  virtual ~MonocularCamera() { delete distortion_model_; }

 public: // ====  optimization =============

  TMatrix33 K() const {
    TMatrix33 K;
    K << Fx(), 0, Cx(),
        0, Fy(), Cy(),
        0, 0, 1;
    return K;
  }

 public:

  void SetDistortionModel(IDistortionModel * model) {
    distortion_model_ = model;
  }

  const IDistortionModel * GetDistortionModel() const override {
    return distortion_model_;
  }

  void ComputeJacobian(const TPoint3D & pt,
                       ProjectionJacobianType & out_jacobian) const override;

  bool IsInFrustum(const TPoint2D & point) const;

  /*!
   *
   */
  void ComputeImageBounds();

  /*!
   * Width of the image
   * @return unsigned
   */
  inline unsigned Width() const { return width_; }

  /*!
   * Height of the image
   * @return
   */
  inline unsigned Height() const { return height_; }

  inline void SetWidth(unsigned width) { width_ = width; }
  inline void SetHeight(unsigned height) { height_ = height; }

  inline const precision_t & ImageBoundMinX() const { return min_X_; }
  inline const precision_t & ImageBoundMinY() const { return min_Y_; }
  inline const precision_t & ImageBoundMaxX() const { return max_X_; }
  inline const precision_t & ImageBoundMaxY() const { return max_Y_; }

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

  inline const precision_t & Fx() const noexcept { return fx_; }
  inline const precision_t & Fy() const noexcept { return fy_; }
  inline const precision_t & Cx() const noexcept { return cx_; }
  inline const precision_t & Cy() const noexcept { return cy_; }
  inline const precision_t & FxInv() const noexcept { return fx_inv_; }
  inline const precision_t & FyInv() const noexcept { return fy_inv_; }

  void SetFx(precision_t fx) noexcept {
    fx_ = fx;
    fx_inv_ = fx ? 1 / fx : 0;
  }
  void SetFy(precision_t fy) noexcept {
    fy_ = fy;
    fy_inv_ = fy ? 1 / fy : 1;
  }
  void SetCx(precision_t cx) noexcept { cx_ = cx; }
  void SetCy(precision_t cy) noexcept { cy_ = cy; }
  CameraType Type() const override;
  void Serialize(std::ostream & ostream) const override;
  void Deserialize(std::istream & istream, serialization::SerializationContext & context) override;

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
  precision_t min_X_, max_X_, min_Y_, max_Y_;
  precision_t fx_inv_, fy_inv_;
  precision_t fx_, fy_, cx_, cy_;
  IDistortionModel * distortion_model_;

};

}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_CAMERA_MONOCULAR_CAMERA_H_
