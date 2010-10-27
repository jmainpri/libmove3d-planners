SET(BM3D_MODULE_NAME src/qtUI)

IF(QT_LIBRARY)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

BM3D_SRC_SUBDIR_PROCESS(
cppToQt.cpp 
main.cpp
glutWindow.cpp
)

BM3D_QT_GENERATE_MOC(main.hpp)

include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/qtBase/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/qtFormRobot/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/qtMainInterface/SourceList.cmake)
include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/qtPlot/SourceList.cmake)

ENDIF(QT_LIBRARY)

include(${CMAKE_SOURCE_DIR}/${BM3D_MODULE_NAME}/qtOpenGL/SourceList.cmake)