add_library(hardware)

target_sources(hardware
    PRIVATE
        arm_config.cpp
        joint.cpp
        joint_factory.cpp
        simulated_joint.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
            arm_config.h
            joint.h
            joint_factory.h
            simulated_joint.h
)

target_include_directories(hardware
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(test)
