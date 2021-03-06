SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/Chomp)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(
chompOptimizer.cpp
chompParameters.cpp
chompCost.cpp
chompTrajectory.cpp
chompPlanningGroup.cpp
)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})