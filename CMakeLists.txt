cmake_minimum_required(VERSION 2.8.3)
project(dwarf-calibration)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=gnu++11") FLAG PRECEDENTI
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++14 -Ofast -march=native") 

set(CMAKE_BUILD_TYPE Release)

message(STATUS "PROCESSOR_TYPE: [${CMAKE_HOST_SYSTEM_PROCESSOR}]")
if (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES armv7l)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  message("ENABLING ARM NEON OPTIMIZATIONS")
endif ()


# Find Eigen3
find_package(Eigen3 REQUIRED)


include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIRS}/src
  src
)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
