# Integrate with CTest (auto-discovers all TEST cases)
find_package(GTest REQUIRED)

# Create test executables
add_executable(kinematic_chain_test kinematic_chain_test.cpp)

# Link to kinematics and GTest
target_link_libraries(
    kinematic_chain_test 
    PRIVATE kinematics
    GTest::gtest
    GTest::gtest_main
)


# Integrate with CTest (auto-discovers all TEST cases)
include(GoogleTest)
gtest_discover_tests(kinematic_chain_test)
