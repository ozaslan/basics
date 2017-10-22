cmake_minimum_required(VERSION 2.8.12)

project(uniform_feat_tracker)

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS 
              roscpp 
              sensor_msgs
              cv_bridge
              utils
              uniform_feat_extractor 
              cmake_modules)

# Declare the message files to be built
#  add_message_files(FILES
#      MyMessage1.msg
#      MyMessage2.msg
#      )

find_package(OpenCV 3 REQUIRED)

# Actually generate the language-specific message and service files
# generate_messages(DEPENDENCIES std_msgs sensor_msgs)

# Declare that this catkin package's runtime dependencies
catkin_package(CATKIN_DEPENDS 
              sensor_msgs 
              cv_bridge
              uniform_feat_extractor 
              utils)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES uniform_feat_tracker
   CATKIN_DEPENDS utils uniform_feat_extractor
   DEPENDS eigen)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} include/uniform_feat_extractor.hh)
add_library(${PROJECT_NAME} src/uniform_feat_tracker.cc)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
add_executable(uniform_feat_tracker_app src/uniform_feat_tracker_app.cc)
#target_link_libraries(visual_odom ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME} uniform_feat_extractor)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_link_libraries(uniform_feat_tracker_app ${PROJECT_NAME} ${catkin_LIBRARIES})
#target_link_libraries(uniform_feat_tracker_app uniform_feat_extractor)
