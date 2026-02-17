# Integrate with CTest (auto-discovers all TEST cases)
find_package(GTest REQUIRED)

# Create your test executable
add_executable(vector3d_test vector3d_test.cpp)

# Link to your library and GTest
target_link_libraries(vector3d_test 
    PRIVATE vector3d
    GTest::gtest
    GTest::gtest_main
)

# Integrate with CTest (auto-discovers all TEST cases)
include(GoogleTest)
gtest_discover_tests(vector3d_test)