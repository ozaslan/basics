cmake_minimum_required(VERSION 2.8.12)

project(utils)

# Get the information about this package's buildtime dependencies
find_package(cmake_modules REQUIRED)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

find_package(Eigen REQUIRED)



find_package(catkin REQUIRED COMPONENTS 
              roscpp 
	      geometry_msgs  
              sensor_msgs
	      visualization_msgs
              nav_msgs
	      cv_bridge
              cmake_modules)



# Declare the message files to be built
#  add_message_files(FILES
#      MyMessage1.msg
#      MyMessage2.msg
#      )

# Actually generate the language-specific message and service files
# generate_messages(DEPENDENCIES std_msgs sensor_msgs)

# Declare that this catkin package's runtime dependencies
catkin_package(CATKIN_DEPENDS 
              roscpp 
	      geometry_msgs  
              sensor_msgs
	      visualization_msgs
              nav_msgs
	      cv_bridge)

catkin_package(
   INCLUDE_DIRS include ${EIGEN_INCLUDE_DIR}
   LIBRARIES 
   CATKIN_DEPENDS 
   DEPENDS Eigen)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIR}
                    ${INCLUDE_DIRS})

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME} src/utils.cc src/trans_utils.cc src/laser_utils.cc src/color_utils.cc)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(utilities ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${ARMADILLO_LIBRARIES} ${catkin_LIBRARIES})

