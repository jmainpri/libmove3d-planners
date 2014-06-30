//
//  chompPlanningGroup.hpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 04/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#ifndef CHOMP_PLANNING_GROUP_HPP
#define CHOMP_PLANNING_GROUP_HPP

#include "API/Device/joint.hpp"
#include "API/Device/robot.hpp"

#include "collision_space/collision_point.hpp"

namespace Move3D {

double chomp_random_value( double max, double min );

/**
 * \brief Contains information about a single joint for CHOMP planning
 */
struct ChompJoint
{
    const Joint* move3d_joint_;                                 /**< Pointer to the Move3D joint in the tree */
    int move3d_joint_index_;                                    /**< Index for use in a Move3D joint array */
    int move3d_dof_index_;                                       /**< Index in the configuration */
    int chomp_joint_index_;                                     /**< Joint index for CHOMP */
    std::string joint_name_;                                    /**< Name of the joint */
    std::string link_name_;                                     /**< Name of the corresponding link (from planning.yaml) */
    bool is_circular_;                                          /**< Does this joint wrap-around? */
    bool has_joint_limits_;                                     /**< Are there joint limits? */
    double joint_limit_min_;                                    /**< Minimum joint angle value */
    double joint_limit_max_;                                    /**< Maximum joint angle value */
    double joint_update_limit_;                                 /**< Maximum amount the joint value can be updated in an iteration */
};

/**
 * \brief Contains information about a planning group
 */
class ChompPlanningGroup
{
public:

    // Creates the planning group
    ChompPlanningGroup(Robot* rob, const std::vector<int>& active_joints );

    Robot* robot_;                                            /** Move3D robot which is planned **/

    std::string name_;                                          /**< Name of the planning group */
    int num_joints_;                                            /**< Number of joints used in planning */
    std::vector<ChompJoint> chomp_joints_;                      /**< Joints used in planning */
    std::vector<std::string> link_names_;                       /**< Links used in planning */
    std::vector<std::string> collision_link_names_;             /**< Links used in collision checking */
    std::vector<CollisionPoint> collision_points_;              /**< Ordered list of collision checking points (from root to tip) */

    /**
   * Gets a random state vector within the joint limits
   */
    template <typename Derived>
    void getRandomState(Eigen::MatrixBase<Derived>& state_vec) const;

    /**
   * Adds the collision point to this planning group, if any of the joints in this group can
   * control the collision point in some way. Also converts the ChompCollisionPoint::parent_joints
   * vector into group joint indexes
   */
    bool addCollisionPoint( CollisionPoint& collision_point );

    /**
      * Returns the Move3d active dofs
      */
    std::vector<int> getActiveDofs() const;

    /**
   * Displays all bounding spheres
   */
    void draw() const;
    void draw(std::vector<Eigen::Transform3d>& segment) const;
    void draw(std::vector<std::vector<double> >& segment) const;
};

template <typename Derived>
void ChompPlanningGroup::ChompPlanningGroup::getRandomState(Eigen::MatrixBase<Derived>& state_vec) const
{
    for (int i=0; i<num_joints_; i++)
    {
        double min = chomp_joints_[i].joint_limit_min_;
        double max = chomp_joints_[i].joint_limit_max_;

        if (!chomp_joints_[i].has_joint_limits_)
        {
            min = -M_PI/2.0;
            max = M_PI/2.0;
        }

        state_vec(i) = chomp_random_value( max, min );
    }
}

}

#endif
