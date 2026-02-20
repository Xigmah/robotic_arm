# Integrate with CTest (auto-discovers all TEST cases)
find_package(GTest REQUIRED)

# Create test executables
add_executable(joint_test joint_test.cpp)
add_executable(arm_config_test arm_config_test.cpp)

# Link to JointInterface and GTest
target_link_libraries(
    joint_test 
    PRIVATE hardware
    GTest::gtest
    GTest::gtest_main
)

# Link to JointInterface and GTest
target_link_libraries(
    arm_config_test 
    PRIVATE hardware
    GTest::gtest
    GTest::gtest_main
)

# Integrate with CTest (auto-discovers all TEST cases)
include(GoogleTest)
gtest_discover_tests(joint_test)
gtest_discover_tests(arm_config_test)