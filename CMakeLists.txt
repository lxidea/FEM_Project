cmake_minimum_required(VERSION 3.16)
project(FEM_Project LANGUAGES CXX)

# Add this line to include the FetchContent module
include(FetchContent)

enable_testing()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(src)
add_subdirectory(app)
add_subdirectory(tests)