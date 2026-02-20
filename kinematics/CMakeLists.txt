add_library(kinematics)

target_sources(kinematics
    PRIVATE
        kinematic_chain.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
            kinematic_chain.h
)

target_include_directories(kinematics
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(test)
