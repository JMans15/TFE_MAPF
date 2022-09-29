cmake_minimum_required(VERSION 3.23)
project(TFE_MAPF)

set(CMAKE_CXX_STANDARD 14)

add_library(TFE_MAPF library.cpp)

# For Qt
set(CMAKE_PREFIX_PATH "/usr/bin/qmake")
find_package(Qt5Widgets REQUIRED)
target_link_libraries(TFE_MAPF Qt5::Widgets)

# For testing
add_executable(TFE_MAPF_test test.cpp library.cpp)
target_link_libraries(TFE_MAPF_test Qt5::Widgets)