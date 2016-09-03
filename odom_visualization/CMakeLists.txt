cmake_minimum_required(VERSION 2.8.12)

project(odom_visualization)

# Get the information about this package's buildtime dependencies

find_package(cmake_modules REQUIRED)

find_package(Eigen REQUIRED) 

find_package(catkin REQUIRED COMPONENTS 
              roscpp 
              tf  
              sensor_msgs
	      visualization_msgs
              nav_msgs
	      utils
	      std_msgs
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
              tf  
              sensor_msgs
	      visualization_msgs
              nav_msgs
	      utils)

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES 
   CATKIN_DEPENDS utils
   DEPENDS Eigen)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${INCLUDE_DIRS})


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(${PROJECT_NAME}_node src/odom_visualization.cc)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} utils)

