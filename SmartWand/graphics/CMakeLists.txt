cmake_minimum_required(VERSION 3.10)
project(graphics VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)

set(GLFW_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/include")
set(GLFW_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/lib-universal")

find_library(GLFW_LIBRARY 
    NAMES glfw3 glfw
    PATHS ${GLFW_LIB_DIR}
    NO_DEFAULT_PATH
    REQUIRED
)

message(STATUS "GLFW Include Dir: ${GLFW_INCLUDE_DIR}")
message(STATUS "GLFW Library Dir: ${GLFW_LIB_DIR}")
message(STATUS "GLFW Library: ${GLFW_LIBRARY}")

add_executable(graphics 
    src/main.cpp   
    src/config.h
    src/glad.c
    src/triangle_mesh.h
    src/triangle_mesh.cpp
)

target_include_directories(graphics PRIVATE 
    ${GLFW_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/dependencies
)
target_link_libraries(graphics 
    ${GLFW_LIBRARY}
    OpenGL::GL
)


# Platform-specific linking (macOS)
if(APPLE)
    target_link_libraries(graphics 
        "-framework Cocoa"
        "-framework IOKit" 
        "-framework CoreVideo"
    )
endif()