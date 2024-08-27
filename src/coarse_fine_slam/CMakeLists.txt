cmake_minimum_required(VERSION 3.0.2)
project(coarse_fine_slam)

# Find required catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

find_package(fmt)
find_package(Threads REQUIRED)
find_package(TBB REQUIRED)

# Define the search paths
set(SEARCH_PATHS "" .. ../.. ../../..)

# Initialize EXT_LIBS_PATH to an empty value
set(EXT_LIBS_PATH "")

# Check each path for the map_storage directory
foreach(PATH ${SEARCH_PATHS})
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${PATH}/libs/map_storage")
    set(EXT_LIBS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${PATH}/libs/map_storage")
    break()
  endif()
endforeach()

if(EXT_LIBS_PATH)
  # Add map_storage subdirectory and force it to use Release configuration
  set(MAP_STORAGE_BINARY_DIR ${CMAKE_BINARY_DIR}/map_storage_build)
  add_subdirectory(${EXT_LIBS_PATH} ${MAP_STORAGE_BINARY_DIR} EXCLUDE_FROM_ALL)
  set_target_properties(map_storage PROPERTIES CMAKE_BUILD_TYPE Release)
else()
  message(FATAL_ERROR "map_storage library not found.")
endif()

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EXT_LIBS_PATH}/include
)

# Source files for the library
file(GLOB_RECURSE LIB_SOURCES "src/*.cpp")
file(GLOB_RECURSE LIB_HEADERS "include/*.hpp" "include/*.h")

# Catkin package
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs nav_msgs pcl_ros roscpp sensor_msgs std_msgs tf2 tf2_ros
 DEPENDS system_lib
)

# Compile options for optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)  # Default to Release if not specified
endif()

# Set compiler flags based on build type
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Og -Wall -Wextra")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Og -Wall -Wextra")
  add_definitions(-DDEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -DNDEBUG")
  add_definitions(-DRELEASE)
endif()

# Optionally, you can print the build type for verification
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")


# Create the library
add_library(${PROJECT_NAME}_lib SHARED ${LIB_SOURCES} ${LIB_HEADERS})

# Link the library with required dependencies
add_dependencies(${PROJECT_NAME}_lib map_storage)
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  map_storage
  fmt::fmt
  Threads::Threads
)

# Create the executable
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
  map_storage
  fmt::fmt
  Threads::Threads
  TBB::tbb
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
