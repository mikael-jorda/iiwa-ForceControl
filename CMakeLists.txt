cmake_minimum_required (VERSION 2.8.12)
project (iiwa-ForceControl)
SET(CMAKE_CXX_FLAGS "-std=c++11")

# set common source
set (IIWA_FORCECONTROL_COMMON_SOURCE
	# ${PROJECT_SOURCE_DIR}/src/redis/RedisClient.h
	${PROJECT_SOURCE_DIR}/src/timer/LoopTimer.cpp
	# ${PROJECT_SOURCE_DIR}/src/tasks/OrientationTask.h
	# ${PROJECT_SOURCE_DIR}/src/tasks/HybridPositionTask.h
	)
include_directories(${PROJECT_SOURCE_DIR}/src)

# set common dependencies
# - CHAI3D
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  add_definitions(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
link_directories(${CHAI3D_LIBRARY_DIRS})

# - SAI2-SIMULATION. TODO: this should not be needed. something fishy.
find_package(SAI2-SIMULATION REQUIRED)
include_directories(${SAI2-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI2-SIMULATION_DEFINITIONS})

# - SAI2-COMMON
find_package(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})
add_definitions(${SAI2-COMMON_DEFINITIONS})

# - glfw3
find_package(glfw3 QUIET)
find_library(GLFW_LIBRARY glfw)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

# - jsoncpp
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
	find_library(JSONCPP_LIBRARY jsoncpp)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(JSONCPP jsoncpp)
	include_directories(${JSONCPP_INCLUDE_DIRS})
	link_directories(${JSONCPP_LIBRARY_DIRS})
	set(JSONCPP_LIBRARY ${JSONCPP_LIBRARIES})
endif ()

# - eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(IIWA_FORCECONTROL_COMMON_LIBRARIES
	${CHAI3D_LIBARIES}
	${SAI2-COMMON_LIBRARIES}
	${HIREDIS_LIBRARY}
	${GLFW_LIBRARY}
	${JSONCPP_LIBRARY}
	)

# add apps
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY                ${PROJECT_SOURCE_DIR}/bin)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          ${PROJECT_SOURCE_DIR}/bin)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     ${PROJECT_SOURCE_DIR}/bin)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE        ${PROJECT_SOURCE_DIR}/bin)
# set (CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${PROJECT_SOURCE_DIR}/bin)
set (IIWA_FORCECONTROL_BINARY_DIR                  ${PROJECT_SOURCE_DIR}/bin)

# add_subdirectory(01-force_tracking)
# add_subdirectory(02-demo_cs225a)
# add_subdirectory(03-multi_contact_validation)
add_subdirectory(04-passivity_demo)