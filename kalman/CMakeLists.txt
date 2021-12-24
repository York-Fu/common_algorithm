cmake_minimum_required(VERSION 3.0.0)
project(kalman VERSION 0.1.0)

option(EXAMPLE "build example" ON)

find_package(Eigen3 REQUIRED)

include_directories(./src)
aux_source_directory(./src SRCS)

add_library(${PROJECT_NAME} ${SRCS})
target_link_directories(${PROJECT_NAME} PUBLIC ./src)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

if(EXAMPLE)
  add_subdirectory(example)
endif(EXAMPLE)
