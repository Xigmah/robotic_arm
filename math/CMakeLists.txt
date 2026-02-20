add_library(math)

target_sources(math
    PRIVATE
        vector3d.cpp
        matrix4x4.cpp
        transform.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
            vector3d.h
            matrix4x4.h
            math_util.h
            transform.h
)

target_include_directories(math
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(test)
