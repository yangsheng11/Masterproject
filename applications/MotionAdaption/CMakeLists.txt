set(TARGET_NAME "MotionAdaption")

if(NOT(MMMCore_FOUND AND MMMTools_FOUND))
    message(STATUS "Required libraries not found for ${TARGET_NAME}")
    return()
endif()

set(LIBRARIES
    WholeBodyMotionLib
    ModelPoseSensor
    KinematicSensor
    stdc++fs
)

set(SOURCES
    MotionAdaption.cpp
)

set(HEADERS
    MotionAdaptionConfiguration.h
)

add_executable(${TARGET_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME} PRIVATE ${LIBRARIES})

