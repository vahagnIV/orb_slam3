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
                                        "\nuniform vec3 col;\n"
                                        "void main()\n"
                                        "{\n"
                                        "\n"
                                        "\t// Output color = red \n"
                                        "\tcolor = col;\n"
                                        "\n"
                                        "}";
#endif //ORB_SLAM3_DRAWER_SHADERS_KEYFRAMEFRAGMENTSHADER_H_
