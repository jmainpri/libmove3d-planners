SET(BM3D_MODULE_NAME src/feature_space)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(
boxes.cpp
features.cpp
planar_feature.cpp
spheres_3d.cpp
spheres.cpp
squares.cpp
clearance.cpp
)
