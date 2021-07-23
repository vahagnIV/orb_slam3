//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_CONSTANTS_H_
#define ORB_SLAM3_INCLUDE_CONSTANTS_H_

// == stl ===
#include <cstddef>
#include "typedefs.h"

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


/*!
 * Nearest neighbour ration for monocular frame motion model matching
 */
const precision_t NNRATIO_MONOCULAR_TWMM = 0.8;


/*!
 * Nearest neighbour orb high threshold for monocular frame motion model matching
 */
const unsigned MONO_TWMM_THRESHOLD_HIGH = 100;

/*!
 * NUmber of match candidates to search in the kf database for merge and loop detection
 */
const unsigned MAX_NUMBER_OF_MATCH_CANDIDATES = 3;

/*!
 * Minimal number of matches in the neighbourhood of the loop/merge candidate kf so that the candidate kf is
 * good
 */
const unsigned LM_MIN_NUMBER_OF_MP_MATCHES = 20;

/*!
 * Minimal number of visible map points from the neighbourhood of the candidate kf that are visible in
 * the current kf.
 */
const unsigned LM_MIN_NUMBER_OF_VISIBLES = 50;


/*!
 * Number of covisible keyframes to take as a neighbourhood for each candidate.
 */
const unsigned LM_COVISIBLE_COUNT = 5;


}
}
#endif //ORB_SLAM3_INCLUDE_CONSTANTS_H_
