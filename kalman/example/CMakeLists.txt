cmake_minimum_required(VERSION 3.0.0)
project(kalmanExample VERSION 0.1.0)

include_directories(./)
aux_source_directory(./ EXAMPLE_SRCS)

add_executable(${PROJECT_NAME} ${EXAMPLE_SRCS})
target_link_libraries(${PROJECT_NAME} kalman)
