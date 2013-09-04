SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/Gestures)
BM3D_SRC_SUBDIR_PROCESS(

HRICS_ClassifyMotion.cpp
HRICS_RecordMotion.cpp
HRICS_WorkspaceOccupancy.cpp
HRICS_HumanPredictionCostSpace.cpp
HRICS_HumanPredictionSimulator.cpp
HRICS_GestParameters.cpp
HRICS_PlayMotion.cpp
)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

BM3D_QT_GENERATE_MOC(
HRICS_GestParameters.hpp
)

include_directories (${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})
