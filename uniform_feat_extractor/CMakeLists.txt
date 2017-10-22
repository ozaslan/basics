cmake_minimum_required(VERSION 2.8.12)

project(uniform_feat_extractor)

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS 
              roscpp 
              sensor_msgs
              cv_bridge
              utils 
              cmake_modules)

# Declare the message files to be built
#  add_message_files(FILES
#      MyMessage1.msg
#      MyMessage2.msg
#      )

# Actually generate the language-specific message and service files
# generate_messages(DEPENDENCIES std_msgs sensor_msgs)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Declare that this catkin package's runtime dependencies
catkin_package(CATKIN_DEPENDS 
              sensor_msgs 
              cv_bridge 
              utils)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES uniform_feat_extractor
   CATKIN_DEPENDS utils
   DEPENDS eigen)


include_directories(include
                    ${OpenCV_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${INCLUDE_DIRS})

# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})  

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} include/uniform_feat_extractor.hh)
add_library(${PROJECT_NAME} src/uniform_feat_extractor.cc)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
add_executable(uniform_feat_extractor_app src/uniform_feat_extractor_app.cc)
#target_link_libraries(visual_odom ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME} uniform_feat_extractor)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_link_libraries(uniform_feat_extractor_app ${PROJECT_NAME} ${catkin_LIBRARIES})
#target_link_libraries(uniform_feat_tracker_app uniform_feat_extractor)
