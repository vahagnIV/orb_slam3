//
// Created by vahagn on 16/08/2021.
//

#ifndef ORB_SLAM3_DRAWER_SHADERS_KEYFRAMEFRAGMENTSHADER_H_
#define ORB_SLAM3_DRAWER_SHADERS_KEYFRAMEFRAGMENTSHADER_H_

const char * KEYFRAME_FRAGMENT_SHADER_SOURCE = "#version 330 core\n"
                                        "\n"
                                        "// Output data\n"
                                        "out vec3 color;\n"
                                        "\n"
                                        "void main()\n"
                                        "{\n"
                                        "\n"
                                        "\t// Output color = red \n"
                                        "\tcolor = vec3(0,1,0);\n"
                                        "\n"
                                        "}";
#endif //ORB_SLAM3_DRAWER_SHADERS_KEYFRAMEFRAGMENTSHADER_H_
