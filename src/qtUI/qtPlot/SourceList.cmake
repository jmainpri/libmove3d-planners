IF(QWT)
SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/qtPlot)

BM3D_SRC_SUBDIR_PROCESS(
basicPlot.cpp 
basicPlotWindow.cpp
doublePlot.cpp
dataPlot.cpp 
tempWin.cpp 
histoWin.cpp
histogramItem.cpp
)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

BM3D_QT_GENERATE_MOC(
basicPlot.hpp
basicPlotWindow.hpp 
doublePlot.hpp
dataPlot.hpp 
tempWin.hpp 
histoWin.hpp
histogramItem.hpp
)

BM3D_QT_GENERATE_UI_HEADERS(basicPlotWindow.ui)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})
ENDIF(QWT)
