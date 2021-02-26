//
// Created by vahagn on 26/02/21.
//

#ifndef ORB_SLAM3_FEATURE_UTILS_H
#define ORB_SLAM3_FEATURE_UTILS_H


// === orb-slam3 ===
#include <features/features.h>

namespace orb_slam3 {
namespace features {

int DescriptorDistance(const DescriptorType & d1,
                       const DescriptorType & d2);

}
}
#endif //ORB_SLAM3_FEATURE_UTILS_H
