IF(HRI_COSTSPACE)
SET(BM3D_MODULE_NAME src/hri_costspace)
BM3D_SRC_SUBDIR_PROCESS(

HRICS_costspace.cpp
HRICS_Distance.cpp
HRICS_Visibility.cpp
HRICS_Natural.cpp
HRICS_Legibility.cpp
HRICS_Workspace.cpp
HRICS_ConfigSpace.cpp
HRICS_Miscellaneous.cpp
HRICS_old.cpp
HRICS_otpmotionpl.cpp
HRICS_humanCostSpace.cpp
HRICS_Navigation.cpp
HRICS_parameters.cpp
)

IF(HRI_PLANNER)
BM3D_SRC_SUBDIR_PROCESS(

HRICS_HAMP.cpp

)
ENDIF(HRI_PLANNER)

BM3D_QT_GENERATE_MOC(
HRICS_parameters.hpp
)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/Grid/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/RRT/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/Gestures/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/human_trajectories/SourceList.cmake)

SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})
ENDIF(HRI_COSTSPACE)
