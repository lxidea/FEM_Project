# Find the Eigen3 package installed by Conda
find_package(Eigen3 REQUIRED)

# Create a library named "fem_core" from our source files
add_library(fem_core
    Mesh.cpp
    Material.cpp
    Tet4Element.cpp
    Assembler.cpp
)

# This makes the header files (like Mesh.h and Material.h) available
target_include_directories(fem_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link our library to Eigen so it can use its features
target_link_libraries(fem_core PUBLIC Eigen3::Eigen)