cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(R0 src/resident.cpp)
rosbuild_add_executable(R1 src/cookingrobot.cpp)
rosbuild_add_executable(R2 src/visitor.cpp)
rosbuild_add_executable(R3 src/schedule.cpp)
rosbuild_add_executable(R4 src/medicalrobot.cpp)
rosbuild_add_executable(R5 src/entertainmentrobot.cpp)
rosbuild_add_executable(R6 src/companionshiprobot.cpp)
rosbuild_add_executable(R7 src/caregiver.cpp)
rosbuild_add_executable(R8 src/relative.cpp)
rosbuild_add_executable(R9 src/doctor.cpp)
rosbuild_add_executable(R10 src/nurse.cpp)
rosbuild_add_executable(R11 src/friend2.cpp)
rosbuild_add_executable(R12 src/relative2.cpp)


rosbuild_add_gtest(residentTest test/resident.test test/residentTest.cpp)
