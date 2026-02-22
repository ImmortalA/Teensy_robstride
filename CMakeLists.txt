cmake_minimum_required(VERSION 3.10)
project(spine_board)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the Asio library
find_package(Threads REQUIRED)
find_path(ASIO_INCLUDE_DIR asio.hpp)

add_library(spine_board SHARED spine_board.cpp)
target_include_directories(spine_board PRIVATE ${ASIO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(spine_board Threads::Threads)
# Add executable
add_executable(test_spine main.cpp)
target_link_libraries(test_spine spine_board)
target_include_directories(test_spine PRIVATE ${ASIO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})