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
                                    "uniform mat4 Projection;\n"
                                    "uniform float scale;\n"
                                    "mat4 aMat4 = mat4(scale, 0.0, 0.0, 0.0,  // 1. column\n"
                                    "                  0.0, -scale, 0.0, 0.0,  // 2. column\n"
                                    "                  0.0, 0.0, -scale, 0.0,  // 3. column\n"
                                    "                  0.0, 0.0, 0.0, 1.0); // 4. column"
                                    "\n"
                                    "void main(){\n"
                                    "\n"
                                    "\t// Output position of the vertex, in clip space : MVP * position\n"
                                    "\tgl_Position = Projection * aMat4 * MVP * vec4(vertexPosition_modelspace ,1) ;\n"
                                    "\n"
                                    "}";
#endif //ORB_SLAM3_DRAWER_SHADERS_VERTEX_SHADER_H_
