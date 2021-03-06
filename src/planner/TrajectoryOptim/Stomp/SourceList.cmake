SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/Stomp)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})
BM3D_SRC_SUBDIR_PROCESS(
run_parallel_stomp.cpp
covariant_trajectory_policy.cpp
control_cost.cpp
cost_computation.cpp
policy_improvement.cpp
policy_improvement_loop.cpp
stompOptimizer.cpp
stompParameters.cpp
)
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})

#constraint_evaluator.cpp
