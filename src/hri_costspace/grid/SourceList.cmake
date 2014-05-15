SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/grid)
BM3D_SRC_SUBDIR_PROCESS(

HRICS_cell.cpp
HRICS_grid.cpp
HRICS_grid_state.cpp
HRICS_two_d_grid.cpp
HRICS_natural_grid.cpp
HRICS_natural_cell.cpp
HRICS_env_grid.cpp
HRICS_agent_grid.cpp

)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

include_directories (${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})
