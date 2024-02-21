cmake_minimum_required(VERSION 3.5)
project(constraints)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_CXX_STANDARD 20)

link_directories("/usr/lib/x86_64-linux-gnu")
link_directories("/usr/local/lib")
link_directories("/usr/local/include")

include_directories(include)
add_library(constraints SHARED src/constraints.cpp)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(constraints Eigen3::Eigen)
