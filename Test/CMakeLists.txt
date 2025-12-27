
cmake_minimum_required(VERSION 3.5)
project(TestProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(test test.cpp)
target_link_libraries(test PRIVATE Eigen3::Eigen)

