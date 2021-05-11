//
// Created by vahagn on 22.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_GEOMETRY_UTILS_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_GEOMETRY_UTILS_H_

#include "../typedefs.h"
#include "pose.h"

namespace orb_slam3 {
namespace geometry {
namespace utils {

/*!
 * Creates skew-symmetric matrix from a vector by contracting it with Levi-Civita symbol
 * @param vector the input vecot
 * @return The skew-symmetric matrix
 */
TMatrix33 SkewSymmetricMatrix(const TVector3D &vector);

/*!
 * Computes the relative transformation between 2 coordinate systems that are specified by the
 * transformations R_to, T_to and R_from, T_from respectively.
 * @param R_to
 * @param T_to
 * @param R_from
 * @param T_from
 * @param out_R The output relative rotation matrix
 * @param out_T The output relative translation vector
 */
void ComputeRelativeTransformation(const Pose & pose_to,
                                   const Pose & pose_from,
                                   Pose & out_pose);

/*!
 * Triangulates the point given its 2 projections
 * @param R The rotation matrix between two camera positions the image was made
 * @param T The translation vector between two camera positions the image was made
 * @param point_from The point projection in the first coordinate system
 * @param point_to The point projection in the second coordinate system
 * @param out_trinagulated The triangulated point in the "from" coordinate system
 * @return true if triangulation successful
 */
bool Triangulate(const Pose & pose,
                 const HomogenousPoint &point_from,
                 const HomogenousPoint &point_to,
                 TPoint3D &out_trinagulated);

/*!
 * Computes the parallax of a point between 2 coordinate systems
 * @param R The rotation matrix between 2 coordinate systems
 * @param T The translation vector between 2 coordinate systems
 * @param point The point
 * @return The parallax
 */
precision_t ComputeCosParallax(const Pose & pose,
                               const TPoint3D &point);

/*!
 * Computes the error of projecting point
 * @param point The 3D-point
 * @param original_point The known projection
 * @return The error
 */
precision_t ComputeReprojectionError(const HomogenousPoint &point, const HomogenousPoint &original_point);

/*!
 * Triangulate point visible by two frames that were made in 2 coordinate systems
 * @param point_from The projection in the first coordinate system
 * @param point_to The projection of the point in the second coordinate system
 * @param R The rotation matrix between "from" and "to" coordinate systems
 * @param T The translation vector between "from" and "to" coordinate systems
 * @param reprojection_threshold_to The threshold to be validated against
 * @param parallax_threshold The parallax threshold to be validated against
 * @param out_parallax The computed parallax of the poinr
 * @param out_triangulated The output triangulated point
 * @return true on success
 */
bool TriangulateAndValidate(const HomogenousPoint &point_from,
                                   const HomogenousPoint &point_to,
                                   const Pose & pose,
                                   precision_t reprojection_threshold_to,
                                   precision_t reprojection_threshold_from,
                                   precision_t parallax_threshold,
                                   precision_t & out_parallax,
                                   TPoint3D &out_triangulated);

}
}
}

#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_GEOMETRY_UTILS_H_
