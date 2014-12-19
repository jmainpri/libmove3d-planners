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
#include "multilocalpath_utils.hpp"

#include "API/project.hpp"
#include "API/Device/robot.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "planner/planEnvironment.hpp"

#include "move3d-headless.h"
#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

using namespace Move3D;
using std::cout;
using std::endl;

static Robot* m_robot=NULL;

static int m_ArmId=0;

static int m_BaseMLP=0;
static int m_BaseSmMLP=0;
static int m_HeadMLP=0;
static int m_UpBodyMLP=0;
static int m_UpBodySmMLP=0;

//! set mlp for this robot
bool traj_optim_set_MultiLP()
{
    if (!m_robot) {
        cout << "robot not initialized in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    for (int i = 0; m_robot && i < m_robot->getP3dRobotStruct()->mlp->nblpGp; i++) {
        if (!strcmp(m_robot->getP3dRobotStruct()->mlp->mlpJoints[i]->gpName, "base")) {
            m_BaseMLP = i;
        } else if (!strcmp(m_robot->getP3dRobotStruct()->mlp->mlpJoints[i]->gpName, "baseSm")) {
            m_BaseSmMLP = i;
        } else if (!strcmp(m_robot->getP3dRobotStruct()->mlp->mlpJoints[i]->gpName, "head")) {
            m_HeadMLP = i;
        } else if (!strcmp(m_robot->getP3dRobotStruct()->mlp->mlpJoints[i]->gpName, "upBody")) {
            m_UpBodyMLP = i;
        } else if (!strcmp(m_robot->getP3dRobotStruct()->mlp->mlpJoints[i]->gpName, "upBodySm")) {
            m_UpBodySmMLP = i;
        }
    }

    return true;
}

//! invalidate all constraints
// --------------------------------------------------------
bool traj_optim_invalidate_cntrts()
{
    if (!m_robot) {
        cout << "robot not initialized in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    p3d_rob* rob = m_robot->getP3dRobotStruct();
    p3d_cntrt* ct;

    // over all constraints
    for(int i=0; i<rob->cntrt_manager->ncntrts; i++)
    {
        std::string name = rob->cntrt_manager->cntrts[i]->namecntrt;

        if ( m_robot->getName() == "JUSTIN_ROBOT" && (name == "p3d_min_max_dofs" || name == "p3d_lin_rel_dofs" || name == "p3d_fixed_jnt" ) ) {
            continue;
        }

        if( name == "p3d_fix_jnts_relpos" ){
            continue;
        }

        // cout << "deactivate : " << m_robot->getName() << " , " << name << endl;

        // get constraint from the cntrts manager
        ct = rob->cntrt_manager->cntrts[i];
        p3d_desactivateCntrt( rob, ct );
    }

    return true;
}

//! cntrts and fix
// --------------------------------------------------------
bool traj_optim_init_mlp_cntrts_and_fix_joints(Robot* rob)
{
    m_robot = rob;

    cout << "Initialize robot : " << m_robot->getName() << endl;

    if( !traj_optim_set_MultiLP() ) {
        return false;
    }

    if( true /*m_robot->getName() != "JUSTIN_ROBOT"*/ )
    {
        if( !traj_optim_invalidate_cntrts() ) {
            return false;
        }

        //    if( !traj_optim_switch_cartesian_mode(false) ) {
        //      return false;
        //    }
    }

    switch( ENV.getInt(Env::setOfActiveJoints) )
    {
    case 0 : // Navigation
        cout << "Set navigation parameters" << endl;
        p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getP3dRobotStruct() , false );
        p3d_multiLocalPath_set_groupToPlan( m_robot->getP3dRobotStruct(), m_BaseMLP, 1, false);
        fixAllJointsExceptBase( m_robot->getP3dRobotStruct() );
        break;

    case 1 : // Manipulation
    {
        cout << "Set manipulation parameters" << endl;
        p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getP3dRobotStruct() , false );
        p3d_multiLocalPath_set_groupToPlan( m_robot->getP3dRobotStruct(), m_UpBodyMLP, 1, false);

        m_ArmId = 0;
        if( m_robot->getName() == "JUSTIN_ROBOT" )
            m_ArmId = 1;

        fixAllJointsWithoutArm( m_robot->getP3dRobotStruct() , m_ArmId );

        if( m_robot->getName() == "JUSTIN_ROBOT" )
        {
            unFixJoint( m_robot->getP3dRobotStruct(), m_robot->getP3dRobotStruct()->joints[2] );
            unFixJoint( m_robot->getP3dRobotStruct(), m_robot->getP3dRobotStruct()->joints[3] );
            unFixJoint( m_robot->getP3dRobotStruct(), m_robot->getP3dRobotStruct()->joints[4] );
            unFixJoint( m_robot->getP3dRobotStruct(), m_robot->getP3dRobotStruct()->joints[5] );
            // Unfix virtual joint
            //unFixJoint( m_robot->getP3dRobotStruct(), m_robot->getP3dRobotStruct()->joints[30] );
        }
    }
        break;

    case 2 : // Mobile Manipulation
        cout << "Set mobile-manipulation parameters" << endl;
        p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getP3dRobotStruct() , false );
        p3d_multiLocalPath_set_groupToPlan( m_robot->getP3dRobotStruct(), m_UpBodyMLP, 1, false);
        fixAllJointsWithoutArm( m_robot->getP3dRobotStruct() , 0 );
        unFixJoint(m_robot->getP3dRobotStruct(), m_robot->getP3dRobotStruct()->baseJnt);
        break;
    }

    return true;
}

//! Virtual object
// --------------------------------------------------------
bool traj_optim_switch_cartesian_mode(bool cartesian) {

    if( m_robot == NULL ) {
        cout << "m_robot == NULL : traj_optim not initilized" << endl;
        return false;
    }

    cout << "Set Cartesian (" << cartesian << ")" << endl;

    p3d_rob* object = NULL;
    p3d_rob* robot = m_robot->getP3dRobotStruct();

    configPt q = p3d_get_robot_config(robot);

    // For Each Arm
    for (int i=0; i < int((*robot->armManipulationData).size()); i++)
    {
        if(  m_robot->getName() == "JUSTIN_ROBOT" && m_ArmId != i )
            continue;

        ArmManipulationData& armData  = (*robot->armManipulationData)[i];

        if( cartesian && m_ArmId==i )
            armData.setCartesian( true );
        else
            armData.setCartesian( false );

        desactivateTwoJointsFixCntrt(robot,armData.getManipulationJnt(), armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);

        if (armData.getCartesian())
        {
            cout << "Arm Data " << i <<  " is set cartesian" << endl;
            // Uptdate the Virual object for inverse kinematics
            // Be carfull the Arm will be unfixed
            //p3d_update_virtual_object_config_for_arm_ik_constraint(robot, i, q);
            activateCcCntrts(robot, i, false);
            ManipulationUtils::unfixManipulationJoints(robot, i);
            shootTheObjectArroundTheBase(robot, robot->baseJnt, armData.getManipulationJnt(), 2.0);

            if(object)
                armData.getManipulationJnt()->dist = object->joints[1]->dist;
            else
                armData.getManipulationJnt()->dist  = 0.10;

        } else {
            deactivateCcCntrts(robot, i);
            //p3d_update_virtual_object_config_for_arm_ik_constraint(robot, i, q);
            p3d_set_and_update_this_robot_conf(robot, q);
            setAndActivateTwoJointsFixCntrt(robot,armData.getManipulationJnt(), armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
            ManipulationUtils::fixManipulationJoints(robot, i, q, NULL);
            shootTheObjectInTheWorld(robot, armData.getManipulationJnt());
        }
    }
    p3d_set_and_update_this_robot_conf(robot, q);
    p3d_get_robot_config_into(robot, &q);
    p3d_destroy_config(robot, q);
    return true;
}

bool generatePointsOnTraj()
{
    cout << "Generate Points on Vias" << endl;

    Move3D::Trajectory traj = m_robot->getCurrentTraj();

    int nb_points = PlanEnv->getInt( PlanParam::nb_pointsOnTraj );
    int nb_via = traj.getNbOfViaPoints()-2;

    if( nb_via < 1 )
    {
        cout << "not enought via" << endl;
        return false;
    }

    int nb_points_per_via = ceil( nb_points/nb_via );

    Move3D::Trajectory new_traj( m_robot );

    double param=0.0;
    double delta = 0.0;
    double density = 4;
    double step = traj.getParamMax()/(nb_points*density);

    for (int i=0; i<traj.getNbOfPaths(); i++ )
    {
        param += traj.getLocalPath( i )->getParamMax();
        delta = -step*density;

        for(int j=0; j<nb_points_per_via; j++ )
        {
            new_traj.push_back( traj.configAtParam( param+delta ));
            delta += step;
        }
    }
    traj = new_traj;
    traj.replaceP3dTraj();
    return true;
}

//! Generate A Soft Motion Trajectory
// --------------------------------------------------------
bool generateSoftMotion()
{
    if( !traj_optim_set_MultiLP() )
        return false;

    Move3D::Trajectory T = m_robot->getCurrentTraj();

    if( T.isEmpty() )
    {
        cout << "The robot has no current traj!!!" << endl;
        return false;
    }

    cout << "m_UpBodyMLP = " << m_UpBodyMLP << endl;

    // T.cutTrajInSmallLP( 10 );
    // T.replaceP3dTraj();
    // T.print();

    MANPIPULATION_TRAJECTORY_CONF_STR confs;
    SM_TRAJ smTraj;

    p3d_multiLocalPath_disable_all_groupToPlan(m_robot->getP3dRobotStruct(), FALSE);
    p3d_multiLocalPath_set_groupToPlan(m_robot->getP3dRobotStruct(), m_UpBodyMLP, 1, FALSE);
    smTraj.clear();

    Move3D::Trajectory TSaved = T;

    //  p3d_traj * trajPt
    //  bool param_write_file
    //  bool approximate
    //  std::vector < int >&lp
    //  std::vector < std::vector <double > > &positions
    //  SM_TRAJ & smTraj
    p3d_convert_traj_to_softMotion(m_robot->getP3dRobotStruct()->tcur,
                                   ENV.getBool(Env::smoothSoftMotionTraj),
                                   true,
                                   false,
                                   confs.first, confs.second, smTraj);

    //smTraj.plot();

    T = m_robot->getCurrentTraj();
    double delta = T.getParamMax() / (100-1) ;

    p3d_multiLocalPath_disable_all_groupToPlan(m_robot->getP3dRobotStruct(), FALSE);
    p3d_multiLocalPath_set_groupToPlan(m_robot->getP3dRobotStruct(), m_UpBodyMLP, 1, FALSE);

    Move3D::Trajectory newT(m_robot);
    cout << "delta = " << delta << endl;

    double t = 0.0;

    for (int i=0; i<100; i++ )
    {
        newT.push_back( T.configAtParam(t) );
        t += delta;

    }
    newT.replaceP3dTraj();
    return true;
}

//! initializes localpaths and cntrts for mobile manip
// --------------------------------------------------------
void traj_optim_hrics_mobile_manip_localpath_and_cntrts()
{
    cout << "Set mobile-manipulation parameters" << endl;
    p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getP3dRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( m_robot->getP3dRobotStruct(), m_UpBodyMLP, 1, false);
    fixAllJointsWithoutArm( m_robot->getP3dRobotStruct() , 0 );
    unFixJoint( m_robot->getP3dRobotStruct(), m_robot->getP3dRobotStruct()->baseJnt );
}

//! Sets the robot and the local method to be used
//! Also sets the constraints and fixes the joints
//! which are not used durring the planning/optimization phaze
// --------------------------------------------------------
void traj_optim_navigation_set_localpath_and_cntrts()
{
#ifdef MULTILOCALPATH
    p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getP3dRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( m_robot->getP3dRobotStruct(), m_BaseMLP, 1, false);

    fixAllJointsExceptBase( m_robot->getP3dRobotStruct() );
#endif

    ENV.setInt( Env::jntToDraw, 1 );
}

//****************************************************************
//* Shelf example
//****************************************************************

//! Sets the robot and the local method to be used
//! Also sets the constraints and fixes the joints
//! which are not used durring the planning/optimization phaze
// --------------------------------------------------------
void traj_optim_shelf_set_localpath_and_cntrts()
{
    cout << "Set robot, localpath and cntrts" << endl;
#ifdef MULTILOCALPATH
    traj_optim_set_MultiLP();
    traj_optim_invalidate_cntrts();

    p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getP3dRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( m_robot->getP3dRobotStruct(), m_UpBodyMLP, 1, false);

    fixAllJointsWithoutArm( m_robot->getP3dRobotStruct() , 0 );
#endif
    //ENV.setInt( Env::jntToDraw, 28 );
}

//! Sets the robot and the local method to be used
//! Also sets the constraints and fixes the joints
//! which are not used durring the planning/optimization phaze
// --------------------------------------------------------
void traj_optim_hrics_set_localpath_and_cntrts()
{
#ifdef MULTILOCALPATH
    p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getP3dRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( m_robot->getP3dRobotStruct(), m_BaseMLP, 1, false);

    fixAllJointsExceptBase( m_robot->getP3dRobotStruct() );
#endif

    ENV.setInt( Env::jntToDraw, 1 );
}
