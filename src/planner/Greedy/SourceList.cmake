SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/Greedy)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(
GreedyCost.cpp
CollisionSpace.cpp
CollisionSpaceCell.cpp
CollisionPoint.cpp
BodySurfaceSampler.cpp
ThresholdPlanner.cpp
CostMapRRTs.cpp
)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})
