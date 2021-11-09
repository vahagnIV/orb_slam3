# ORB SLAM3


This is a re-implementation of  [ORB Slam 3 library]( https://github.com/UZ-SLAMLab/ORB_SLAM3) by Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel, Juan D. Tardos..

[This video](https://youtu.be/EsD2Up2jifA) shows the performance of the library on the TUM database.

## Overview
Currently we have implemented only the monocular case without IMU sensors. We imagine the future roadmap as follows:

 1. Test the library on more datasets. Discover and fix bugs.
 2. Use C++14 (or later) features to optimize multithreading efficacy. 
 3. Implement **stereo** and **rgbd** cases. (should not take much time)
 4. Add IMU sensor fusion.
 5. Test the library in real world scenarios, say, by putting it on a mobile robot or a RC UAV.
 
 
 The **main.cpp** program is for testing purposes only and is written to work only with the **TUM corridor1** database or a calibrated usb web camera. Implementing a similar application for another database is straightforward.
 
 ## Dependencies
 
 - The application depends on **Eigen3** library for matrix operations. 
 - For the time being we use **OpenCV**, however, the dependency is minimal and will be removed in future. 
 - We use [**g2o**](https://github.com/RainerKuemmerle/g2o) for error minimization problems.
 

