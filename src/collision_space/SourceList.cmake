SET(BM3D_MODULE_NAME src/collision_space)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(
collision_space.cpp
collision_point.cpp
body_surface_sampler.cpp
collision_space_factory.cpp
)
