SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/Lamp)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(
trajOptimizer.cpp
lampTrajectory.cpp
lampComputeCost.cpp
lampStomp.cpp
lampSimpleLoop.cpp
)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})

#constraint_evaluator.cpp
