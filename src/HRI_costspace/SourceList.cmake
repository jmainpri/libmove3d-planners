IF(HRI_COSTSPACE)
SET(BM3D_MODULE_NAME src/HRI_costspace)
BM3D_SRC_SUBDIR_PROCESS(

HRICS_costspace.cpp
HRICS_Distance.cpp
HRICS_Visibility.cpp
HRICS_Natural.cpp
HRICS_Workspace.cpp
HRICS_ConfigSpace.cpp
HRICS_Miscellaneous.cpp
HRICS_old.cpp
HRICS_otpmotionpl.cpp
HRICS_humanCostSpace.cpp
HRICS_Navigation.cpp
)

IF(HRI_PLANNER)
BM3D_SRC_SUBDIR_PROCESS(

HRICS_HAMP.cpp

)
ENDIF(HRI_PLANNER)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/Grid/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/RRT/SourceList.cmake)

SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})
ENDIF(HRI_COSTSPACE)
