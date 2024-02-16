cmake_minimum_required(VERSION 3.0.2)
project(audio_writer)

find_package(catkin REQUIRED COMPONENTS
  audio_common_msgs
  rosbag
  roscpp
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES audio_writer
#  CATKIN_DEPENDS audio_common_msgs rosbag roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  "/usr/lib" 
  "/usr/local/lib"
)


# Find the libsndfile library
find_library(SNDFILE_LIBRARY NAMES sndfile libsndfile HINTS "/usr/lib" "/usr/local/lib")
if(NOT SNDFILE_LIBRARY)
  message(FATAL_ERROR "libsndfile not found")
endif()

# Declare ROS executable
add_executable(save_raw_audio_stream src/save_raw_audio_stream.cpp)

# Link the libraries
target_link_libraries(save_raw_audio_stream ${catkin_LIBRARIES} ${SNDFILE_LIBRARY})
