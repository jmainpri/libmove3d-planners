SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/qtMainInterface)

BM3D_SRC_SUBDIR_PROCESS(
kcdpropertieswindow.cpp 
mainwindow.cpp
mainwindowTestFunctions.cpp
)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

BM3D_QT_GENERATE_MOC(
kcdpropertieswindow.hpp 
mainwindow.hpp
mainwindowGenerated.hpp
mainwindowTestFunctions.hpp
)

BM3D_QT_GENERATE_UI_HEADERS(
kcdpropertieswindow.ui 
)

include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/sideWidgets/SourceList.cmake)

SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})