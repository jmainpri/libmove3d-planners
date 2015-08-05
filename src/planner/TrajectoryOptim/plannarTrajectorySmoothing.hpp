/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#ifndef PLANNARTRAJECTORYSMOOTHING_HPP
#define PLANNARTRAJECTORYSMOOTHING_HPP

#include "API/ConfigSpace/configuration.hpp"
MOVE3D_USING_SHARED_PTR_NAMESPACE

#include <Eigen/StdVector>

namespace Move3D {

class PlannarTrajectorySmoothing
{
public:
    PlannarTrajectorySmoothing( Move3D::Robot* robot );


    void initTraj( std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, Move3D::Robot* Cylinder );
    bool goToNextStep();


    // functions
    double computeDistOfTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, int begin, int end);

    double computeDistBetweenTrajAndPoint(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, Eigen::Vector2d p);

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > removeUnnecessaryPoint(
            std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > trajectory, double threshold);

    bool robotCanDoTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj,
                                                    Robot* BoundingBox, Robot* trajOfThisRobot, double dist);

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > smoothTrajectory(Robot* robot,
            std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > trajectory);

    Eigen::Vector2d getRandomPointInSegment(Eigen::Vector2d p1, Eigen::Vector2d p2, double errorT);

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > findShortCut(
            std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, int i, int j);


    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > findShortCut(
            std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, int i, Robot* cyl, Robot* robot);

    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > add3Dim(
            std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, Robot* robot, double epsilon);

    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > add3DimwithoutTrajChange(
            std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, Robot* robot, double epsilon);

    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > removeSamePoints(
            std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > traj, double epsilon);

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > get2DtrajFrom3Dtraj(
            std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > traj);



private:
    int _id;
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > _traj;
    Move3D::Robot* _cyl;
    Move3D::Robot* _Robot;
    confPtr_t  _curRobotConf;
    confPtr_t  _curCylConf;
    double _dist;


};

}

#endif // PLANNARTRAJECTORYSMOOTHING_HPP
