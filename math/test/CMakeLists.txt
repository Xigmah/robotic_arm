# Integrate with CTest (auto-discovers all TEST cases)
find_package(GTest REQUIRED)

# Create test executables
add_executable(vector3d_test vector3d_test.cpp)
add_executable(transform_test transform_test.cpp)
add_executable(math_util_test math_util_test.cpp)

# Link to Vector3D and GTest
target_link_libraries(
    vector3d_test 
    PRIVATE math
    GTest::gtest
    GTest::gtest_main
)

# Link to transform and GTest
target_link_libraries(
    transform_test 
    PRIVATE math
    GTest::gtest
    GTest::gtest_main
)

# Link to math_util and GTest
target_link_libraries(
    math_util_test 
    PRIVATE math
    GTest::gtest
    GTest::gtest_main
)

# Integrate with CTest (auto-discovers all TEST cases)
include(GoogleTest)
gtest_discover_tests(vector3d_test)
gtest_discover_tests(transform_test)
gtest_discover_tests(math_util_test)
