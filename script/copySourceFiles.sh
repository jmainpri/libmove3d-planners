 #!/bin/bash
 #
export OOBM3D_ROOT_PATH=/Users/jmainpri/workspace/OOMove3D/src/
export BM3D_ROOT_PATH=/Users/jmainpri/workspace/BioMove3D

echo "Copy : "$BM3D_ROOT_PATH/planner_cxx;
echo

# Planner
cp $BM3D_ROOT_PATH/planner_cxx/*.cpp $OOBM3D_ROOT_PATH/planner
cp $BM3D_ROOT_PATH/planner_cxx/*.hpp $OOBM3D_ROOT_PATH/planner
cp $BM3D_ROOT_PATH/planner_cxx/Diffusion/*.cpp $OOBM3D_ROOT_PATH/planner/Diffusion
cp $BM3D_ROOT_PATH/planner_cxx/Diffusion/*.hpp $OOBM3D_ROOT_PATH/planner/Diffusion
cp $BM3D_ROOT_PATH/planner_cxx/Diffusion/Expansion/*.cpp $OOBM3D_ROOT_PATH/planner/Diffusion/Expansion
cp $BM3D_ROOT_PATH/planner_cxx/Diffusion/Expansion/*.hpp $OOBM3D_ROOT_PATH/planner/Diffusion/Expansion
cp $BM3D_ROOT_PATH/planner_cxx/Diffusion/RRT-Variants/*.cpp $OOBM3D_ROOT_PATH/planner/Diffusion/RRT-Variants
cp $BM3D_ROOT_PATH/planner_cxx/Diffusion/RRT-Variants/*.hpp $OOBM3D_ROOT_PATH/planner/Diffusion/RRT-Variants
cp $BM3D_ROOT_PATH/planner_cxx/PRM/*.cpp $OOBM3D_ROOT_PATH/planner/PRM
cp $BM3D_ROOT_PATH/planner_cxx/PRM/*.hpp $OOBM3D_ROOT_PATH/planner/PRM
cp $BM3D_ROOT_PATH/planner_cxx/Greedy/*.cpp $OOBM3D_ROOT_PATH/planner/Greedy
cp $BM3D_ROOT_PATH/planner_cxx/Greedy/*.hpp $OOBM3D_ROOT_PATH/planner/Greedy

echo "Copy : "
echo

# HRI
cp $BM3D_ROOT_PATH/HRI_costspace/*.cpp $OOBM3D_ROOT_PATH/HRI_costspace
cp $BM3D_ROOT_PATH/HRI_costspace/*.hpp $OOBM3D_ROOT_PATH/HRI_costspace
cp $BM3D_ROOT_PATH/HRI_costspace/RRT/*.cpp $OOBM3D_ROOT_PATH/HRI_costspace/RRT
cp $BM3D_ROOT_PATH/HRI_costspace/RRT/*.hpp $OOBM3D_ROOT_PATH/HRI_costspace/RRT
cp $BM3D_ROOT_PATH/HRI_costspace/Grid/*.cpp $OOBM3D_ROOT_PATH/HRI_costspace/Grid
cp $BM3D_ROOT_PATH/HRI_costspace/Grid/*.hpp $OOBM3D_ROOT_PATH/HRI_costspace/Grid

echo "Copy : "$BM3D_ROOT_PATH/planner_cxx/API;
echo

# API
cp $BM3D_ROOT_PATH/planner_cxx/API/*.cpp $OOBM3D_ROOT_PATH/API/
cp $BM3D_ROOT_PATH/planner_cxx/API/*.hpp $OOBM3D_ROOT_PATH/API/
cp $BM3D_ROOT_PATH/planner_cxx/API/ConfigSpace/*.cpp $OOBM3D_ROOT_PATH/API/ConfigSpace/
cp $BM3D_ROOT_PATH/planner_cxx/API/ConfigSpace/*.hpp $OOBM3D_ROOT_PATH/API/ConfigSpace/
cp $BM3D_ROOT_PATH/planner_cxx/API/Grids/*.cpp $OOBM3D_ROOT_PATH/API/Grids/
cp $BM3D_ROOT_PATH/planner_cxx/API/Grids/*.hpp $OOBM3D_ROOT_PATH/API/Grids/
cp $BM3D_ROOT_PATH/planner_cxx/API/Grids/GridToGraph/*.cpp $OOBM3D_ROOT_PATH/API/Grids/GridToGraph
cp $BM3D_ROOT_PATH/planner_cxx/API/Grids/GridToGraph/*.hpp $OOBM3D_ROOT_PATH/API/Grids/GridToGraph
cp $BM3D_ROOT_PATH/planner_cxx/API/Device/*.cpp $OOBM3D_ROOT_PATH/API/Device/
cp $BM3D_ROOT_PATH/planner_cxx/API/Device/*.hpp $OOBM3D_ROOT_PATH/API/Device/
cp $BM3D_ROOT_PATH/planner_cxx/API/Roadmap/*.cpp $OOBM3D_ROOT_PATH/API/Roadmap/
cp $BM3D_ROOT_PATH/planner_cxx/API/Roadmap/*.hpp $OOBM3D_ROOT_PATH/API/Roadmap/
cp $BM3D_ROOT_PATH/planner_cxx/API/Search/*.cpp $OOBM3D_ROOT_PATH/API/Search/
cp $BM3D_ROOT_PATH/planner_cxx/API/Search/*.hpp $OOBM3D_ROOT_PATH/API/Search/
cp $BM3D_ROOT_PATH/planner_cxx/API/Search/AStar/*.cpp $OOBM3D_ROOT_PATH/API/Search/AStar/
cp $BM3D_ROOT_PATH/planner_cxx/API/Search/AStar/*.hpp $OOBM3D_ROOT_PATH/API/Search/AStar/
cp $BM3D_ROOT_PATH/planner_cxx/API/Search/Dijkstra/*.cpp $OOBM3D_ROOT_PATH/API/Search/Dijkstra/
cp $BM3D_ROOT_PATH/planner_cxx/API/Search/Dijkstra/*.hpp $OOBM3D_ROOT_PATH/API/Search/Dijkstra/
cp $BM3D_ROOT_PATH/planner_cxx/API/Trajectory/*.cpp $OOBM3D_ROOT_PATH/API/Trajectory/
cp $BM3D_ROOT_PATH/planner_cxx/API/Trajectory/*.hpp $OOBM3D_ROOT_PATH/API/Trajectory/

echo "Copy : "$BM3D_ROOT_PATH/CppApi;
echo

# utils
cp $BM3D_ROOT_PATH/util/CppApi/*.cpp $OOBM3D_ROOT_PATH/utils/
cp $BM3D_ROOT_PATH/util/CppApi/*.hpp $OOBM3D_ROOT_PATH/utils/

echo "Copy : "$BM3D_ROOT_PATH/qtWindow;
echo

# qtUI
cp $BM3D_ROOT_PATH/qtWindow/*.cpp $OOBM3D_ROOT_PATH/qtUI
cp $BM3D_ROOT_PATH/qtWindow/*.hpp $OOBM3D_ROOT_PATH/qtUI
cp $BM3D_ROOT_PATH/qtWindow/qtOpenGL/*.cpp $OOBM3D_ROOT_PATH/qtUI/qtOpenGL
cp $BM3D_ROOT_PATH/qtWindow/qtOpenGL/*.hpp $OOBM3D_ROOT_PATH/qtUI/qtOpenGL
cp $BM3D_ROOT_PATH/qtWindow/qtOpenGL/*.ui $OOBM3D_ROOT_PATH/qtUI/qtOpenGL
cp $BM3D_ROOT_PATH/qtWindow/qtBase/*.cpp $OOBM3D_ROOT_PATH/qtUI/qtBase
cp $BM3D_ROOT_PATH/qtWindow/qtBase/*.hpp $OOBM3D_ROOT_PATH/qtUI/qtBase
cp $BM3D_ROOT_PATH/qtWindow/qtBase/*.ui $OOBM3D_ROOT_PATH/qtUI/qtBase
cp $BM3D_ROOT_PATH/qtWindow/qtFormRobot/*.cpp $OOBM3D_ROOT_PATH/qtUI/qtFormRobot
cp $BM3D_ROOT_PATH/qtWindow/qtFormRobot/*.hpp $OOBM3D_ROOT_PATH/qtUI/qtFormRobot
cp $BM3D_ROOT_PATH/qtWindow/qtFormRobot/*.ui $OOBM3D_ROOT_PATH/qtUI/qtFormRobot
cp $BM3D_ROOT_PATH/qtWindow/qtPlot/*.cpp $OOBM3D_ROOT_PATH/qtUI/qtPlot
cp $BM3D_ROOT_PATH/qtWindow/qtPlot/*.hpp $OOBM3D_ROOT_PATH/qtUI/qtPlot
cp $BM3D_ROOT_PATH/qtWindow/qtPlot/*.ui $OOBM3D_ROOT_PATH/qtUI/qtPlot
cp $BM3D_ROOT_PATH/qtWindow/qtMainInterface/*.cpp $OOBM3D_ROOT_PATH/qtUI/qtMainInterface
cp $BM3D_ROOT_PATH/qtWindow/qtMainInterface/*.hpp $OOBM3D_ROOT_PATH/qtUI/qtMainInterface
cp $BM3D_ROOT_PATH/qtWindow/qtMainInterface/*.ui $OOBM3D_ROOT_PATH/qtUI/qtMainInterface
cp $BM3D_ROOT_PATH/qtWindow/qtMainInterface/sideWidgets/*.cpp $OOBM3D_ROOT_PATH/qtUI/qtMainInterface/sideWidgets
cp $BM3D_ROOT_PATH/qtWindow/qtMainInterface/sideWidgets/*.hpp $OOBM3D_ROOT_PATH/qtUI/qtMainInterface/sideWidgets
cp $BM3D_ROOT_PATH/qtWindow/qtMainInterface/sideWidgets/*.ui $OOBM3D_ROOT_PATH/qtUI/qtMainInterface/sideWidgets
