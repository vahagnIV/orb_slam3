//
// Created by vahagn on 16/08/2021.
//

#ifndef ORB_SLAM3_DRAWER_SHADERS_VERTEX_SHADER_H_
#define ORB_SLAM3_DRAWER_SHADERS_VERTEX_SHADER_H_
const char * VERTEX_SHADER_SOURCE = "#version 330 core\n"
                                    "\n"
                                    "// Input vertex data, different for all executions of this shader.\n"
                                    "layout(location = 0) in vec3 vertexPosition_modelspace;\n"
                                    "\n"
                                    "// Values that stay constant for the whole mesh.\n"
                                    "uniform mat4 MVP;\n"
                                    "\n"
                                    "void main(){\n"
                                    "\n"
                                    "\t// Output position of the vertex, in clip space : MVP * position\n"
                                    "\tgl_Position =  MVP * vec4(vertexPosition_modelspace,1);\n"
                                    "\n"
                                    "}";
#endif //ORB_SLAM3_DRAWER_SHADERS_VERTEX_SHADER_H_
