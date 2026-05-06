add_library(kinematics)

target_sources(kinematics
    PRIVATE
        kinematic_chain.cpp
        inverse_kinematics.cpp
        analytical_ik.cpp
        jacobian_ik.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
            kinematic_chain.h
            inverse_kinematics.h
            analytical_ik.h
            jacobian_ik.h

)

target_include_directories(kinematics
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link against dependencies
target_link_libraries(kinematics
    PUBLIC
        hardware
        math
)

add_subdirectory(test)
