set(TARGET_NAME "OdeIntTest")

set(LIBRARIES
)

set(SOURCES
    OdeIntTest.cpp
)

set(HEADERS
)

add_executable(${TARGET_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME} PRIVATE ${LIBRARIES})

