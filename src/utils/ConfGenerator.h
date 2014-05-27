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
#ifndef CONFGENERATOR_H
#define CONFGENERATOR_H

#include "API/Device/robot.hpp"
#include "planner/planner.hpp"
#include "planner/TrajectoryOptim/plannarTrajectorySmoothing.hpp"

#include "utils/OtpUtils.hpp"
#include "utils/ConfGenerator.h"

#include "hri_costspace/HRICS_natural.hpp"

#include <libmove3d/include/LightPlanner-pkg.h>

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace Move3D
{

class constrainedLink
{
public:
    constrainedLink();
    constrainedLink(Robot* first, std::vector<std::vector<double> > matrixRot, std::vector<double> vectTrans);
    virtual ~constrainedLink();
    void addRange(std::pair<int,std::vector<std::pair<double,double> > > range);


    std::string name;
    Robot* R;
    std::vector<std::vector<double> > matrixRotation;
    std::vector<double> vectTranslation;

    std::map<int,std::vector<std::pair<double,double> > > rangeList;
    int nbDofs;
};

}

//typedef struct linkStruct
//{
//    Robot* first;
//    Robot* second;
//    configurationConstraint CC;
//}linked;

//typedef struct configurationConstraintStruct
//{
//
//}configurationConstraint;

namespace Move3D
{

class ConfGenerator
{
public:
    ConfGenerator();
    ConfGenerator(Robot* rob,Robot* human);

    std::vector<Eigen::Vector3d> getOTPList(){return m_OTPList;}

    /**
   * Compute a configuration for hand-over
   */
    bool computeRobotIkForGrabing( configPt& q );

    /**
   * Compute a configuration for handing over an object for the arm Dofs
   * @point the point in the workspace
   * @q the configuration of the robot
   */
    bool computeRobotIkForGrabing( configPt& q, const Eigen::Vector3d& point );

    /**
   * Compute a handover configuration by iterating through the set of configurations
   * that has been pre-loaded
   */
    bool computeHandoverConfigFromList( std::pair<confPtr_t,confPtr_t>& best_handover_conf, double& best_cost );

    /**
   * Initializes the list config generator
   */
    bool initialize( std::string filename, HRICS::Natural* reachableSpace );

    /**
   * load and return configs stored in filename
   */
    bool sortConfig( std::vector<HRICS::ConfigHR>& configList, int nbNode, bool isStanding, bool isSlice, HRICS::Natural* reachableSpace);

    /**
   * Adding a 3D point to the OTP list This function is used when loading a set of OTPs in order to test them
   */
    void addToList(Eigen::Vector3d WSPoint);

    /**
   * Change the current OTP. Used when choosing new configurations
   */
    Eigen::Vector3d setCurOTP( Eigen::Vector3d WSPoint);

    /**
   * draw the OTP list
   */
    void drawOTPList(bool value);

    /**
   * Compute GIK and place robot base
   */
    bool placeRobot();

    /**
   * Adding the actual configuration to m_configList
   */
    std::vector<HRICS::ConfigHR> addConfToList();

    /**
   * removing the last configuration of m_configList
   */
    void removeLastConf();

    /**
   * Clear m_configList;
   */
    void clearConfList();

    /**
   * save what's in m_configList to the filename file.
   */
    void saveToXml(std::string filename);

    /**
   * load and return configs stored in filename
   */
    std::vector<HRICS::ConfigHR> loadFromXml(std::string filename);

    //########################################
    //new version :
    //########################################

    bool generateConfiguration(int nbConfs);
    bool addConstraintToList(constrainedLink CL);


private:

    /**
   * Set the human into the configuration at the ihe index given as parametter
   * @param ith index if vector
   * @param vectConfs vector of human robot config
   */
    std::pair<confPtr_t,confPtr_t> setRobotsToConf(const HRICS::ConfigHR& handover_conf);

    /**
   * the list of OTPs used to create a new config list
   */
    std::vector<Eigen::Vector3d> m_OTPList;

    /**
   * store configs (for standing human when loading)
   */
    std::vector<HRICS::ConfigHR> m_configList;

    Robot* _robot;
    Robot* _human;

    //########################################
    //new version :
    //########################################
    std::vector<constrainedLink> linkList;
    std::map<std::string,constrainedLink> linkListFiltred;
};

}

#endif // CONFGENERATOR_H
