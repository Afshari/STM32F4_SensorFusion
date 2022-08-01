cmake_minimum_required(VERSION 3.5)

project(GTests LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DGTEST=1)

include_directories(../Inc)
include_directories(../Inc/Classes)
include_directories(../Inc/Math)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

MESSAGE( STATUS CMAKE_PREFIX_PATH " = " ${CMAKE_PREFIX_PATH} )

find_package(GTest CONFIG REQUIRED)
# message( STATUS "${CMAKE_PREFIX_PATH}/include" ${CMAKE_PREFIX_PATH})
include_directories("${CMAKE_PREFIX_PATH}/include")

set(SOURCES ../Src/Classes/app_handler.cpp
	../Src/Classes/kf_tracking.cpp
	../Src/Classes/recursive_least_squares.cpp
	../Src/Classes/kf_passive_suspension.cpp
	../Src/Classes/input_parser.cpp
	../Src/Math/matrix.cpp)

add_executable(gTestSensorFusion main.cpp ${SOURCES})
target_link_libraries(gTestSensorFusion GTest::gtest GTest::gmock)

#  cmake -G "MinGW Makefiles" -DCMAKE_PREFIX_PATH="$env:CMAKE_PREFIX_PATH" ..