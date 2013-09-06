//
//  chompPlanningGroup.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 04/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "chompPlanningGroup.hpp"
#include "chompUtils.hpp"

#include "P3d-pkg.h"

using namespace std;

ChompPlanningGroup::ChompPlanningGroup(Robot* rob, const std::vector<int>& active_joints )
{
    robot_ = rob;
    chomp_joints_.clear();

    cout << "Creating planning group for : " << robot_->getName() << endl;

    for (unsigned int i=0; i<active_joints.size(); i++)
    {
        for (unsigned int j=0; j<robot_->getJoint( active_joints[i] )->getNumberOfDof(); j++)
        {
            Joint* move3d_joint = robot_->getJoint( active_joints[i] );

            cout << "Joint(" << j << ") : (" << move3d_joint->getName() << ")" << endl;

            if( !move3d_joint->isJointDofUser(j) )
                continue;

            double min,max;
            //robot_->getJoint( active_joints[i] )->getDofBounds(j,min,max);
            move3d_joint->getDofRandBounds(j,min,max);

            cout << "Is dof user : ";
            cout << "(min = " << min << ", max = " << max << ")" << endl;

            if (min == max)
                continue;

            ChompJoint jnt;

            jnt.move3d_joint_ = move3d_joint;
            jnt.move3d_joint_index_ = active_joints[i];
            jnt.move3d_dof_index_ = move3d_joint->getIndexOfFirstDof() + j;
            jnt.chomp_joint_index_ = i;
            jnt.joint_name_ = move3d_joint->getName();
            jnt.wrap_around_ = p3d_jnt_is_dof_circular(move3d_joint->getJointStruct(),j);
            jnt.has_joint_limits_ = true;
            jnt.joint_limit_min_ = min;
            jnt.joint_limit_max_ = max;
            jnt.joint_update_limit_ = 0.10;

            chomp_joints_.push_back( jnt );
        }
    }

    num_joints_ = chomp_joints_.size();
}

std::vector<int> ChompPlanningGroup::getActiveDofs() const
{
    std::vector<int> active_joints;
    for(int i=0;i<int(chomp_joints_.size());i++)
    {
        active_joints.push_back( chomp_joints_[i].move3d_dof_index_ );
    }

    return active_joints;
}

bool ChompPlanningGroup::addCollisionPoint(CollisionPoint& collision_point)
{
    // create the new parent joints indexing vector:
    std::vector<int> parent_joints(num_joints_, 0);

    //ROS_INFO_STREAM("Num joints is " << num_joints_ << " parent size is " << collision_point.getParentJoints().size());

    // check if this collision point is controlled by any joints which belong to the group
    bool add_this_point=false;
    for (int i=0; i<num_joints_; i++)
    {
        if (collision_point.isParentJoint(chomp_joints_[i].move3d_joint_index_))
        {
            add_this_point = true;
            break;
        }
    }

    if (!add_this_point)
        return false;

    collision_points_.push_back(CollisionPoint(collision_point, collision_point.getParentJoints()));
    return true;
}

void ChompPlanningGroup::draw() const
{
    //  cout << "Number of collision points : " << collision_points_.size() << endl;
    for (unsigned int i=0; i<collision_points_.size(); i++)
    {
        //    if (i>=38)
        //      continue;

        Eigen::Transform3d T = robot_->getJoint( collision_points_[i].getParentJoints().back() )->getMatrixPos();

        collision_points_[i].draw(T);
    }
}

void ChompPlanningGroup::draw(std::vector<Eigen::Transform3d>& segment) const
{
    for (unsigned int i=0; i<collision_points_.size(); i++)
    {
        Eigen::Transform3d T = segment[collision_points_[i].getSegmentNumber()];
        collision_points_[i].draw( T );
    }
}

void ChompPlanningGroup::draw(std::vector<std::vector<double> >& segment) const
{
    for (unsigned int i=0; i<collision_points_.size(); i++)
    {
        Eigen::Transform3d T;
        stdVectorToEigenTransform(segment[collision_points_[i].getSegmentNumber()],T);
        collision_points_[i].draw( T );
    }
}

