SET(BM3D_MODULE_NAME_TMP3 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/Dijkstra)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(dijkstra.cpp)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP3})
