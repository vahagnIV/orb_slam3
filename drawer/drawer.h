//
// Created by vahagn on 13.06.21.
//

#include <pangolin/pangolin.h>
#include <geometry/pose.h>

#ifndef ORB_SLAM3_DRAWER_DRAWER_H_
#define ORB_SLAM3_DRAWER_DRAWER_H_

class Drawer {
 public:
  Drawer(int width, int height){
    pangolin::CreateWindowAndBind("Map", width, height);
  }

  void DrawKeyFrame(orb_slam3::geometry::Pose & pose);
};

#endif //ORB_SLAM3_DRAWER_DRAWER_H_
