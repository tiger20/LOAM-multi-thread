include_directories(../../include)
add_library(loam SHARED
            math_utils.h
            BasicScanRegistration.cpp
            MultiScanRegistration.cpp
            BasicLaserOdometry.cpp
            LaserOdometry.cpp
            BasicLaserMapping.cpp
            LaserMapping.cpp
            BasicTransformMaintenance.cpp
            TransformMaintenance.cpp)
target_link_libraries(loam ${PCL_LIBRARIES})
