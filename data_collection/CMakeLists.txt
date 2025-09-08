cmake_minimum_required(VERSION 3.14)
project(data_collection LANGUAGES CXX)

# Use C++17 for libfranka compatibility
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#find libfranka (saved in /home/.local/lib/cmake/franka)
find_package(Franka 0.9.2 REQUIRED)
find_package(Eigen3 REQUIRED)

# executables
add_executable(communication_test src/communication_test.cpp)
add_executable(impedance_control src/impedance_control.cpp)
add_executable(cartesian_motion src/cartesian_motion.cpp)

# link libraries
target_link_libraries(communication_test PRIVATE Franka::Franka Eigen3::Eigen)
target_link_libraries(impedance_control PRIVATE Franka::Franka Eigen3::Eigen)
target_link_libraries(cartesian_motion PRIVATE Franka::Franka Eigen3::Eigen)

