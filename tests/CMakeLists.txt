find_package(Eigen3 REQUIRED)

# --- Download and configure Google Test ---
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG    v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
# --- End of FetchContent block ---

# Test #1: Mesh Tests
add_executable(run_mesh_tests test_mesh.cpp)
target_link_libraries(run_mesh_tests PRIVATE fem_core GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(run_mesh_tests)

# Test #2: Material Tests
add_executable(run_material_tests test_material.cpp)
target_link_libraries(run_material_tests PRIVATE fem_core GTest::gtest_main Eigen3::Eigen)
gtest_discover_tests(run_material_tests)

# Test #3: Element Tests (NEW)
add_executable(run_element_tests test_element.cpp)
target_link_libraries(run_element_tests PRIVATE fem_core GTest::gtest_main)
gtest_discover_tests(run_element_tests)

# Test #4: Assembler Tests (NEW)
add_executable(run_assembler_tests test_assembler.cpp)
target_link_libraries(run_assembler_tests PRIVATE fem_core GTest::gtest_main)
gtest_discover_tests(run_assembler_tests)