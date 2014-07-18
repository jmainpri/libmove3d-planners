SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/human_trajectories)
BM3D_SRC_SUBDIR_PROCESS(
HRICS_ioc.cpp
HRICS_human_features.cpp
HRICS_human_ioc.cpp
HRICS_human_cost_space.cpp
HRICS_detours.cpp
HRICS_run_multiple_planners.cpp
HRICS_ioc_sequences.cpp
HRICS_human_simulator.cpp
HRICS_dynamic_time_warping.cpp
)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

#BM3D_QT_GENERATE_MOC(
#HRICS_human_cost_space.hpp
#)

include_directories (${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})
