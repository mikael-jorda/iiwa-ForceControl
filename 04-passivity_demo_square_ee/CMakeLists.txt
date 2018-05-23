# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${IIWA_FORCECONTROL_BINARY_DIR}/04-passivity_demo_square_ee)
ADD_EXECUTABLE (controller04 controller.cpp ${IIWA_FORCECONTROL_COMMON_SOURCE})
ADD_EXECUTABLE (simviz04 simviz.cpp ${IIWA_FORCECONTROL_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller04 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz04 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${IIWA_FORCECONTROL_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/04-passivity_demo_square_ee)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf iiwa7.urdf plate.urdf DESTINATION ${APP_RESOURCE_DIR})