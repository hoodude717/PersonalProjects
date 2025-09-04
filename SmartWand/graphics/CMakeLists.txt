cmake_minimum_required(VERSION 3.10)
project(graphics VERSION 1.0)
cmake_policy(SET CMP0072 NEW)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(graphics
    src/main.cpp
    src/glad.c
    src/triangle_mesh.cpp
    src/config.cpp
    src/material.cpp
    src/lin_algebra.cpp
)

target_include_directories(graphics PRIVATE
    dependencies  # For GLAD
)

target_link_libraries(graphics
    glfw
    OpenGL::GL
)