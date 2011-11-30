SET(BM3D_MODULE_NAME ${CMAKE_SOURCE_DIR}/src/utils)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

BM3D_SRC_SUBDIR_PROCESS(

testModel.cpp
#SaveContext.cpp
ConfGenerator.cpp
OtpUtils.cpp

)

IF(QT_LIBRARY)
BM3D_SRC_SUBDIR_PROCESS(

#MultiRun.cpp
#PlanningThread.cpp

)

BM3D_QT_GENERATE_MOC(
#PlanningThread.h
)


ENDIF(QT_LIBRARY)

