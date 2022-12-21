cmake_minimum_required(VERSION 3.22)
project(VehicleModel)

set(CMAKE_CXX_STANDARD 14)

add_library(TwoWheel STATIC two_wheel.cpp two_wheel.h)

add_executable(VehicleModel main.cpp)

target_link_libraries(VehicleModel TwoWheel)
