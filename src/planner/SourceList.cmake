SET(BM3D_MODULE_NAME src/planner)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(
planner.cpp 
plannerFunctions.cpp 
planEnvironment.cpp
cost_space.cpp
)

IF(MULTI_LOCALPATH AND LIGHT_PLANNER)
BM3D_SRC_SUBDIR_PROCESS(
replanning.cpp
)
ENDIF()

#BM3D_QT_GENERATE_MOC(
#planEnvironment.hpp
#)

include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/Diffusion/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/PRM/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/Greedy/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/TrajectoryOptim/SourceList.cmake)