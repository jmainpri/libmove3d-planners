/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#ifndef POLICY_IMPROVEMENT_LOOP_H_
#define POLICY_IMPROVEMENT_LOOP_H_

//#include <ros/ros.h>
//#include <rosbag/bag.h>
#include <boost/shared_ptr.hpp>

#include "policy.hpp"

#include "task.hpp"
#include "policy_improvement.hpp"
#include "API/ConfigSpace/configuration.hpp"

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
//#include <policy_improvement_loop/PolicyImprovementStatistics.h>

namespace stomp_motion_planner
{

class PolicyImprovementLoop
{
public:
    PolicyImprovementLoop();
    virtual ~PolicyImprovementLoop();

//    bool initializeAndRunTaskByName(/*ros::NodeHandle& node_handle,*/ std::string& task_name);

    bool initialize(MOVE3D_BOOST_PTR_NAMESPACE<Task> task, bool singleRollout);
    bool runSingleIteration(int iteration_number);
  
    /**
     * Functions added by jim
     */
    void testSampler();
    bool generateSingleNoisyTrajectory();
    void getRollouts(std::vector<std::vector<Move3D::confPtr_t> >& traj);
  
    // Reset all extra rollouts
    void resetReusedRollouts();

  
private:

    bool initialized_;
//    ros::NodeHandle node_handle_;

    int num_rollouts_;
    int num_reused_rollouts_;
    int num_time_steps_;
    int num_dimensions_;

    bool write_to_file_;
    bool use_cumulative_costs_;
    bool set_parameters_in_policy_;
  
    bool use_annealing_;
    int limits_violations_;
    double K_;
  
    MOVE3D_BOOST_PTR_NAMESPACE<Task> task_;
    MOVE3D_BOOST_PTR_NAMESPACE<Policy> policy_;

    PolicyImprovement policy_improvement_;
  
    std::vector<std::vector<Eigen::VectorXd> > rollouts_; /**< [num_rollouts][num_dimensions] num_parameters */
    std::vector<std::vector<Eigen::VectorXd> > reused_rollouts_;
  
    std::vector<Eigen::MatrixXd> parameter_updates_;
    std::vector<Eigen::VectorXd> parameters_;
    Eigen::MatrixXd rollout_costs_;
    std::vector<double> noise_stddev_;
    std::vector<double> noise_decay_;

    boost::mutex mtx_end_;
    boost::mutex mtx_set_end_;
    std::vector<bool> parrallel_is_rollout_running_;
    std::vector<Eigen::VectorXd> parallel_cost_;
    std::vector<boost::thread*> threads_;

    double control_cost_weight_;
    double state_cost_weight_;

    // temporary variables
    Eigen::VectorXd tmp_rollout_cost_;

    bool readParameters();

    // added by jim
    bool readParametersSingleRollout();
    void resampleParameters();

    bool setParallelRolloutsEnd(int r);
    void parallelRollout(int i, int iteration_number);

    void executeRollout(int r, int iteration_number);

    int policy_iteration_counter_;
    bool readPolicy(const int iteration_number);
    bool writePolicy(const int iteration_number, bool is_rollout = false, int rollout_id = 0);
    //bool writePolicyImprovementStatistics(const policy_improvement_loop::PolicyImprovementStatistics& stats_msg);
  
    void addStraightLineRollout(std::vector<std::vector<Eigen::VectorXd> >& extra_rollout, std::vector<Eigen::VectorXd>& extra_rollout_cost);
    void parametersToVector(std::vector<Eigen::VectorXd>& rollout);
    void getSingleRollout(const std::vector<Eigen::VectorXd>& rollout, std::vector<Move3D::confPtr_t>& traj);
    void addSingleRolloutsToDraw(const std::vector<Eigen::VectorXd>& rollout, int color);
    void addRolloutsToDraw(bool add_reused);

    // Print rollouts
    void printSingleRollout( const std::vector<Eigen::VectorXd>& rollout, int id ) const;
    void printRollouts() const;
};

}

#endif /* POLICY_IMPROVEMENT_LOOP_H_ */
