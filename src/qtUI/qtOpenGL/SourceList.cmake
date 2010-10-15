SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/qtOpenGL)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

IF(QT_LIBRARY)
    BM3D_SRC_SUBDIR_PROCESS(
    g3dQtConnection.cpp 
    glwidget.cpp 
    qtGLWindow.cpp 
    qtopenglviewer.cpp
    qtMobileCamera.cpp
    )
    BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
    BM3D_QT_GENERATE_UI_HEADERS(
    qtopenglviewer.ui
    )
    BM3D_QT_GENERATE_MOC(
    g3dQtConnection.hpp 
    glwidget.hpp 
    qtGLWindow.hpp 
    qtopenglviewer.hpp
    qtMobileCamera.h
    )
    
ENDIF(QT_LIBRARY)

SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})
