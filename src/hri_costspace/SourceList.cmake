IF(HRI_COSTSPACE)
SET(BM3D_MODULE_NAME src/hri_costspace)
BM3D_SRC_SUBDIR_PROCESS(

HRICS_costspace.cpp
HRICS_distance.cpp
HRICS_visibility.cpp
HRICS_natural.cpp
HRICS_legibility.cpp
HRICS_workspace.cpp
HRICS_config_space.cpp
HRICS_miscellaneous.cpp
HRICS_old.cpp
HRICS_otpmotionpl.cpp
HRICS_human_cost_space.cpp
HRICS_navigation.cpp
HRICS_parameters.cpp
)

IF(HRI_PLANNER)
BM3D_SRC_SUBDIR_PROCESS(

HRICS_hamp.cpp

)
ENDIF(HRI_PLANNER)

BM3D_QT_GENERATE_MOC(
HRICS_parameters.hpp
)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/grid/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/RRT/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/gestures/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/human_trajectories/SourceList.cmake)

SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})
ENDIF(HRI_COSTSPACE)
