# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${IIWA_FORCECONTROL_BINARY_DIR}/util-fsensor_calibration)
ADD_EXECUTABLE (bias_calibration bias_calibration.cpp ${IIWA_FORCECONTROL_COMMON_SOURCE})
ADD_EXECUTABLE (object_calibration object_calibration.cpp ${IIWA_FORCECONTROL_COMMON_SOURCE})
ADD_EXECUTABLE (simviz-util00 simviz.cpp ${IIWA_FORCECONTROL_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (bias_calibration ${IIWA_FORCECONTROL_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (object_calibration ${IIWA_FORCECONTROL_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz-util00 ${IIWA_FORCECONTROL_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${IIWA_FORCECONTROL_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/util-fsensor_calibration)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf iiwa7.urdf DESTINATION ${APP_RESOURCE_DIR})