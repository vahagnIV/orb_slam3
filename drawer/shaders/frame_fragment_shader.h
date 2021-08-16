//
// Created by vahagn on 16.08.21.
//

#ifndef ORB_SLAM3_DRAWER_SHADERS_FRAME_FRAGMENT_SHADER_H_
#define ORB_SLAM3_DRAWER_SHADERS_FRAME_FRAGMENT_SHADER_H_
const char *FRAME_FRAGMENT_SHADER_SOURCE = "#version 330 core\n"
                                           "\n"
                                           "// Output data\n"
                                           "out vec3 color;\n"
                                           "\n"
                                           "void main()\n"
                                           "{\n"
                                           "\n"
                                           "\t// Output color = red \n"
                                           "\tcolor = vec3(1,0,0);\n"
                                           "\n"
                                           "}";
#endif //ORB_SLAM3_DRAWER_SHADERS_FRAME_FRAGMENT_SHADER_H_
