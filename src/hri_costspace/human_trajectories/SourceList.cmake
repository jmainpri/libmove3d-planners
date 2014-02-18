SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/human_trajectories)
BM3D_SRC_SUBDIR_PROCESS(
HRICS_features.cpp
HRICS_ioc.cpp
HRICS_spheres.cpp
HRICS_spheres_3d.cpp
HRICS_human_features.cpp
HRICS_human_ioc.cpp
HRICS_human_cost_space.cpp
HRICS_detours.cpp
HRICS_planar_feature.cpp
HRICS_squares.cpp
HRICS_boxes.cpp
HRICS_run_multiple_planners.cpp
)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

#BM3D_QT_GENERATE_MOC(
#HRICS_HumanCostSpace.hpp
#)

include_directories (${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})
