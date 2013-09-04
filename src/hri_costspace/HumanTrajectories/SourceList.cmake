SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/HumanTrajectories)
BM3D_SRC_SUBDIR_PROCESS(
HRICS_features.cpp
HRICS_TrajectoryEvaluator.cpp
HRICS_ioc.cpp
HRICS_spheres.cpp
HRICS_HumanIoc.cpp
)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

#BM3D_QT_GENERATE_MOC(
#HRICS_HumanCostSpace.hpp
#)

include_directories (${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})
