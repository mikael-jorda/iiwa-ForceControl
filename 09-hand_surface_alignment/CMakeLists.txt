# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${IIWA_FORCECONTROL_BINARY_DIR}/09-hand_surface_alignment)
ADD_EXECUTABLE (controller09 controller.cpp ${IIWA_FORCECONTROL_COMMON_SOURCE})
ADD_EXECUTABLE (simviz09 simviz.cpp ${IIWA_FORCECONTROL_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (controller09 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz09 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${IIWA_FORCECONTROL_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/09-hand_surface_alignment)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf iiwa7_hand_surface_alignment.urdf DESTINATION ${APP_RESOURCE_DIR})