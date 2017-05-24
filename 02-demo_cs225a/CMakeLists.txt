# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${IIWA_FORCECONTROL_BINARY_DIR}/02-demo_cs225a)
ADD_EXECUTABLE (controller2 controller_main.cpp)
ADD_EXECUTABLE (simulation2 simulation_main.cpp)
ADD_EXECUTABLE (visualizer2 visualizer_main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller2 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simulation2 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (visualizer2 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${IIWA_FORCECONTROL_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/02-demo_cs225a)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})