SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/gestures)
BM3D_SRC_SUBDIR_PROCESS(

HRICS_classify_motion.cpp
HRICS_record_motion.cpp
HRICS_workspace_occupancy.cpp
HRICS_human_prediction_cost_space.cpp
HRICS_human_prediction_simulator.cpp
HRICS_gest_parameters.cpp
HRICS_play_motion.cpp
)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

BM3D_QT_GENERATE_MOC(
HRICS_gest_parameters.hpp
)

include_directories (${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})
