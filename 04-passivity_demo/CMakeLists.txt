# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${IIWA_FORCECONTROL_BINARY_DIR}/04-passivity_demo)
ADD_EXECUTABLE (controller4 controller_main.cpp ${IIWA_FORCECONTROL_COMMON_SOURCE})
ADD_EXECUTABLE (simulation4 simulation_main.cpp ${IIWA_FORCECONTROL_COMMON_SOURCE})
ADD_EXECUTABLE (visualizer4 visualizer_main.cpp ${IIWA_FORCECONTROL_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller4 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simulation4 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (visualizer4 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${IIWA_FORCECONTROL_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/04-passivity_demo)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})