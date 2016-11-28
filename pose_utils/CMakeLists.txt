cmake_minimum_required(VERSION 2.8.12)

project(pose_utils)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS 
              roscpp 
              cmake_modules)

# Declare that this catkin package's runtime dependencies
catkin_package(CATKIN_DEPENDS 
              roscpp 
              )
catkin_package(CATKIN_DEPENDS 
              roscpp)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES 
   CATKIN_DEPENDS 
   DEPENDS armadillo)



include_directories(${catkin_INCLUDE_DIRS}
                    ${INCLUDE_DIRS}
		    include)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_library(pose_utils src/pose_utils.cpp)
target_link_libraries(pose_utils ${ARMADILLO_LIBRARIES})
