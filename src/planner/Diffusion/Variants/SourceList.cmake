SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/Variants)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(
ManhattanLike-RRT.cpp 
Transition-RRT.cpp
Multi-RRT.cpp
Multi-TRRT.cpp
Threshold-RRT.cpp
Star-RRT.cpp
Costmap-RRT.cpp
BaseExpansion.cpp 
ESTExpansion.cpp 
RRTExpansion.cpp 
RRG.cpp
)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})
