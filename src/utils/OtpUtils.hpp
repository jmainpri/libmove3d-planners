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
#ifndef OTPUTILS_HPP
#define OTPUTILS_HPP

#include "API/Trajectory/trajectory.hpp"

#include "planner/planner.hpp"
#include "planner/TrajectoryOptim/plannarTrajectorySmoothing.hpp"

#include "utils/OtpUtils.hpp"

#include <libmove3d/include/LightPlanner-pkg.h>

////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <Eigen/StdVector>



extern void g3d_show_tcur_both_rob(p3d_rob *robotPt, int (*fct)(p3d_rob* robot, p3d_localpath* curLp),
                                   p3d_rob *hum_robotPt, int (*hum_fct)(p3d_rob* hum_robot, p3d_localpath* hum_curLp));

extern bool detectSittingFurniture( Move3D::Robot* human, double threshold, Move3D::Robot** furniture);

namespace HRICS
{
    class OutputConf
    {
    public:
        Move3D::confPtr_t humanConf;
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > humanTraj;
        bool humanTrajExist;

        Move3D::confPtr_t robotConf;
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > robotTraj;
        bool robotTrajExist;

        double cost;
        int configNumberInList;
        int id;
        bool isStandingInThisConf;

        void clearAll();

        Move3D::confPtr_t chairConf;
//        OutputConf& operator= (const OutputConf& o);

    };

    class ConfigHR
    {
    public:

        static int index;
        ConfigHR() { id = index++; cost = 1.; }

        configPt getHumanConf() const { return q_hum; }
        void setHumanConf( Move3D::Robot* human, configPt q);

        configPt getRobotConf() const { return q_rob; }
        void setRobotConf( Move3D::Robot* robot, configPt q);

        int getId() const { return id; }
        void setId(int value) { id = value; }

        double getCost() const { return cost; }
        void setCost(double value) { cost = value; }

    private:
        configPt q_hum;
        configPt q_rob;
        int id;
        double cost;

    };

    /**
     * configuration cost sorter
     */
    class ConfigurationCost
    {
    public:

        bool operator()(ConfigHR first, ConfigHR second)
        {
            return ( first.getCost() < second.getCost() );
        }

    };

    /**
     * outputconf sorter
     */
    class OutputConfSort
    {
    public:

            bool operator()(OutputConf first, OutputConf second)
            {
                    return ( first.cost < second.cost );
            }

    };

}


#endif // OTPUTILS_HPP
