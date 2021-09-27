//
// Created by vahagn on 16/08/2021.
//

#ifndef ORB_SLAM3_DRAWER_DRAWER_H_
#define ORB_SLAM3_DRAWER_DRAWER_H_

#include "drawer_impl.h"

namespace orb_slam3 {
namespace drawer {

/*!
 * Initializes static variable and links necessary libraries
 * @return true on success
 */
bool Initialize();
void Terminate();

}
}

#endif //ORB_SLAM3_DRAWER_DRAWER_H_
