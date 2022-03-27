set(TARGET_NAME "GeodesicGenerator")

if(NOT(MMMCore_FOUND AND MMMTools_FOUND))
    message(STATUS "Required libraries not found for ${TARGET_NAME}")
    return()
endif()

set(LIBRARIES
    WholeBodyMotionLib
    stdc++fs
)

set(SOURCES
GeodesicGenerator.cpp
)

set(HEADERS
)

add_executable(${TARGET_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME} PRIVATE ${LIBRARIES})

