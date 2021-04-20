//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_CONSTANTS_H_
#define ORB_SLAM3_INCLUDE_CONSTANTS_H_

// == stl ===
#include <cstddef>
#include <typedefs.h>

namespace orb_slam3{
namespace constants{

const int MINIMAL_FEATURE_COUNT_PER_FRAME_MONOCULAR = 100;
const size_t FRAME_GRID_ROWS  = 48;
const size_t FRAME_GRID_COLS  = 64;

/*!
 * Delta for the robust Huber kernel used for optimization in monocular frame
 */
const precision_t HUBER_MONO_DELTA = std::sqrt(5.991);


/*!
 * The max threshold for the allowed squared error for optimization in monocular frame
 */
const precision_t MONO_CHI2 = 5.991;

/*!
 * The maxed allowed parallax for triangulation
 */
const precision_t PARALLAX_THRESHOLD = 0.9998;

}
}
#endif //ORB_SLAM3_INCLUDE_CONSTANTS_H_
