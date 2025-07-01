cmake_minimum_required(VERSION 3.10)
project(TC_GINS)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)

aux_source_directory(src SRCS)

add_library(TC_GINS ${SRCS})


add_executable(TC_GINS_example main.cpp)
target_link_libraries(TC_GINS_example TC_GINS)