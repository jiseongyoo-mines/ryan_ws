cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(ryan_arms_plugin_l SHARED ryan_arms_plugin_l.cc)
target_link_libraries(ryan_arms_plugin_l ${GAZEBO_libraries} ${roscpp_LIBRARIES})
add_library(ryan_arms_plugin_r SHARED ryan_arms_plugin_r.cc)
target_link_libraries(ryan_arms_plugin_r ${GAZEBO_libraries} ${roscpp_LIBRARIES})


# Build the stand-alone test program
add_executable(pos pos.cc)

if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(pos ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(pos ${GAZEBO_LIBRARIES})
endif()
