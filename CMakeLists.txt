#add_compile_options(-std=c++14)
cmake_minimum_required(VERSION 3.10)
project(main)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Pangolin
find_package(Pangolin REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Add include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    /usr/include/eigen3
    /usr/local/include
    /usr/local/include/sigslot
    /usr/local/include/pangolin
    /ORB_SLAM3
    /ORB_SLAM3/include
    /ORB_SLAM3/include/CameraModels
    /ORB-SLAM3/Thirdparty/Sophus
)

# Add executable
#add_executable(main main.cc)

add_executable(main
        Examples/Monocular/main.cc)
target_link_libraries(main ${PROJECT_NAME})

# Specify the folder containing your .cc files
file(GLOB_RECURSE ORB_SLAM3_SOURCES 
    "../ORB_SLAM3/src/*.cc"
    "../ORB_SLAM3/src/*.cpp"
    "../ORB_SLAM3/include/*.h"
    "../ORB_SLAM3/include/*.hpp"
)

# Add the source files to the executable
target_sources(main PRIVATE ${ORB_SLAM3_SOURCES})

# Link libraries
target_link_libraries(main ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ORB_SLAM3)