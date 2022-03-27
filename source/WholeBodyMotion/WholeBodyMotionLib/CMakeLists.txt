set(TARGET_NAME "WholeBodyMotionLib")

if(NOT(MMMCore_FOUND))
    message(STATUS "Required libraries not found for ${TARGET_NAME}")
    return()
endif()

set(LIBRARIES
    SimoxUtility
    VirtualRobot
    MMMCore
    stdc++fs
    ModelPoseSensor
    KinematicSensor
)

if(drake_FOUND)
	set(LIBRARIES ${LIBRARIES} drake::drake)
        set(EXTERNAL_LIBRARY_FLAGS ${EXTERNAL_LIBRARY_FLAGS} -DDRAKE_AVAILABLE)
	message("drake found. Including.")
else()
	message("drake not found. Ignoring!")
endif()


# These headers make up the public interface of the library
# They are installed via "make install" and should not include
# private headers which are only used for implementation.
set(PUBLIC_HEADERS
    Tools.h
    RBDLExtension.h
    finite_difference.hpp
    drake_test.h
    BVP_Plant.h
    BVP_Solver.h
)

set(HEADERS
    ${PUBLIC_HEADERS}
)

set(SOURCES
    Tools.cpp
    RBDLExtension.cpp
)

add_library(${TARGET_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${TARGET_NAME} PUBLIC ${LIBRARIES})
set_target_properties(${TARGET_NAME} PROPERTIES PUBLIC_HEADER "${PUBLIC_HEADERS}")

target_compile_definitions(${TARGET_NAME} PUBLIC -D_REENTRANT)
target_include_directories(${TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>)
target_include_directories(${TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>)

install(TARGETS ${TARGET_NAME}
    EXPORT ${TARGET_NAME}Targets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
    PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/${TARGET_NAME}"
    COMPONENT dev
)


