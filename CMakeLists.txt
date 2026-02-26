cmake_minimum_required(VERSION 3.10)
project(spine_board)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the Asio library
find_package(Threads REQUIRED)
find_path(ASIO_INCLUDE_DIR asio.hpp)

add_library(robstride_o2 STATIC robstride_o2/robstride_o2_motor.cpp)
target_include_directories(robstride_o2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/robstride_o2)

add_library(spine_board SHARED spine_board.cpp)
target_include_directories(spine_board PRIVATE ${ASIO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(spine_board Threads::Threads)

# Main demo executable (apps/main.cpp)
add_executable(test_spine apps/main.cpp)
target_link_libraries(test_spine spine_board robstride_o2)
target_include_directories(test_spine PRIVATE ${ASIO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/robstride_o2)

# MIT-only demo (apps/demo02_MIT.cpp)
add_executable(test_spine_mit apps/demo02_MIT.cpp)
target_link_libraries(test_spine_mit spine_board robstride_o2)
target_include_directories(test_spine_mit PRIVATE ${ASIO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/robstride_o2)

# Communication test (apps/demo01_comm_test.cpp)
add_executable(comm_test apps/demo01_comm_test.cpp)
target_link_libraries(comm_test spine_board)
target_include_directories(comm_test PRIVATE ${ASIO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

# Two motors on Can0 daisy chain (apps/demo03_daisy_chain_2_motors.cpp)
add_executable(daisy_chain_2_motors apps/demo03_daisy_chain_2_motors.cpp)
target_link_libraries(daisy_chain_2_motors spine_board robstride_o2)
target_include_directories(daisy_chain_2_motors PRIVATE ${ASIO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/robstride_o2)