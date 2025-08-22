# Add an executable target named "fem_app" from the main.cpp file
add_executable(fem_app main.cpp)

target_link_libraries(fem_app PRIVATE fem_core)