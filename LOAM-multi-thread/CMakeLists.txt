cmake_minimum_required(VERSION 2.8.3)
project(loam_velodyne)

## Compile as C++14, supported in ROS Kinetic and newer
# set_property(TARGET invz_player PROPERTY CXX_STANDARD 17)
add_compile_options(-std=c++11)

add_definitions( -march=native )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Pangolin 0.4 REQUIRED)

include_directories( 
    "/usr/local/include/eigen3"
    ${Pangolin_INCLUDE_DIRS}
)

include_directories(
  include
	${PCL_INCLUDE_DIRS}
  "./include")

add_subdirectory(src/lib)

add_executable(run_loam main.cpp)
target_link_libraries(run_loam ${PCL_LIBRARIES} loam ${Pangolin_LIBRARIES})

