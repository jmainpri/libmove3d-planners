SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/Expansion)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(
BaseExpansion.cpp 
ESTExpansion.cpp 
RRTExpansion.cpp 
TransitionExpansion.cpp
ThresholdExpansion.cpp
StarExpansion.cpp
CostmapExpansion.cpp
)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})
