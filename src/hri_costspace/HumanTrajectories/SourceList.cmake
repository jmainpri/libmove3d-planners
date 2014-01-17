SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/HumanTrajectories)
BM3D_SRC_SUBDIR_PROCESS(
HRICS_features.cpp
HRICS_ioc.cpp
HRICS_spheres.cpp
HRICS_HumanFeatures.cpp
HRICS_HumanIoc.cpp
HRICS_HumanCostSpace.cpp
HRICS_detours.cpp
HRICS_planarfeature.cpp
HRICS_squares.cpp
)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

#BM3D_QT_GENERATE_MOC(
#HRICS_HumanCostSpace.hpp
#)

include_directories (${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})
