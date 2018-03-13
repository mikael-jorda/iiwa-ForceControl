# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${IIWA_FORCECONTROL_BINARY_DIR}/util-fsensor_calibration)
ADD_EXECUTABLE (calibration_routine calibration_routine.cpp ${IIWA_FORCECONTROL_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (calibration_routine ${IIWA_FORCECONTROL_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${IIWA_FORCECONTROL_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/util-fsensor_calibration)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY iiwa7.urdf DESTINATION ${APP_RESOURCE_DIR})