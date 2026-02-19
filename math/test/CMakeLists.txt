# Integrate with CTest (auto-discovers all TEST cases)
find_package(GTest REQUIRED)

# Create test executables
add_executable(vector3d_test vector3d_test.cpp)
add_executable(matrix4x4_test matrix4x4_test.cpp)
add_executable(transform_test transform_test.cpp)

# Link to Vector3D and GTest
target_link_libraries(
    vector3d_test 
    PRIVATE math
    GTest::gtest
    GTest::gtest_main
)

# Link to Matrix4x4 and GTest
target_link_libraries(
    matrix4x4_test 
    PRIVATE math
    GTest::gtest
    GTest::gtest_main
)

# Link to Transform and GTest
target_link_libraries(
    transform_test 
    PRIVATE math
    GTest::gtest
    GTest::gtest_main
)

# Integrate with CTest (auto-discovers all TEST cases)
include(GoogleTest)
gtest_discover_tests(vector3d_test)
gtest_discover_tests(matrix4x4_test)
gtest_discover_tests(transform_test)