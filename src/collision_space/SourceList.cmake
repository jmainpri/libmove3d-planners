SET(BM3D_MODULE_NAME src/collision_space)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(
CollisionSpace.cpp
CollisionPoint.cpp
BodySurfaceSampler.cpp
)
