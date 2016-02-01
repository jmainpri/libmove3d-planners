SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/TrajectoryOptim)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

IF(MULTILOCALPATH)
BM3D_SRC_SUBDIR_PROCESS(
trajectoryOptim.cpp
eiquadprog.hpp
jointlimits.hpp
goal_set_projection.cpp
goal_set_projection.hpp
vector_trajectory.cpp
vector_trajectory.hpp)
ENDIF()

BM3D_SRC_SUBDIR_PROCESS(plannarTrajectorySmoothing.cpp)

include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/Classic/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/Chomp/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/Stomp/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/Lamp/SourceList.cmake)


SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})
