SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/qtBase)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

BM3D_SRC_SUBDIR_PROCESS(
qtBaseWindow.cpp 
qt_widgets.cpp 
SpinBoxSliderConnector_p.cpp)

BM3D_QT_GENERATE_MOC(
SpinBoxSliderConnector_p.hpp 
qtBaseWindow.hpp 
qt_widgets.hpp)

SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})
