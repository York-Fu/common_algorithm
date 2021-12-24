cmake_minimum_required(VERSION 3.0.0)
project(c_filter VERSION 0.1.0)

option(EXAMPLE "build example" ON)

include_directories(./src)
aux_source_directory(./src SRCS)

add_library(${PROJECT_NAME} ${SRCS})
target_link_directories(${PROJECT_NAME} PUBLIC ./src)

if(EXAMPLE)
  add_subdirectory(example)
endif(EXAMPLE)
