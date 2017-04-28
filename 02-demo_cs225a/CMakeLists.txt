# create an executable
ADD_EXECUTABLE (controller_02 controller_main.cpp)
ADD_EXECUTABLE (simulation_02 simulation_main.cpp)
ADD_EXECUTABLE (visualizer_02 visualizer_main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller_02 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simulation_02 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (visualizer_02 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${IIWA_FORCECONTROL_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/02-demo_cs225a)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})