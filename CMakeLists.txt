cmake_minimum_required(VERSION 3.10)
project(BRT_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(include)

add_executable(can_test
    src/can_interface.cpp
    src/brt_sensor.cpp
    src/sensor_array.cpp
    src/force_estimator.cpp
    src/main.cpp
)

add_executable(single_sensor_test
    src/can_interface.cpp
    src/brt_sensor.cpp
    src/single_sensor_test.cpp
)
