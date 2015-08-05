#ifndef HRICS_OTPMOTIONPL_HPP
#define HRICS_OTPMOTIONPL_HPP

/*
 *  HRICS_CostSpace.h
 *  BioMove3D
 *
 *  Created by Mamoun Gharbi on 20/04/11.
 *  Copyright 2011 magharbi@laas.fr All rights reserved.
 *
 */

#include "HRICS_workspace.hpp"

#include "API/Trajectory/trajectory.hpp"

#include "planner/planner.hpp"
#include "planner/TrajectoryOptim/plannarTrajectorySmoothing.hpp"

#include "grid/HRICS_env_grid.hpp"
#include "grid/HRICS_two_d_grid.hpp"

#include "utils/OtpUtils.hpp"
#include "utils/ConfGenerator.h"

#include <libmove3d/include/LightPlanner-pkg.h>

////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <Eigen/StdVector>

extern std::string global_ActiveRobotName;
extern Move3D::TwoDGrid* API_activeRobotGrid;

extern Eigen::Vector3d current_WSPoint;
extern std::pair<double,Eigen::Vector3d > current_cost;

void hrics_otp_fct();
/**
 @defgroup HRICS Hri Cost space
 */

/**
 @ingroup HRICS
 */
namespace HRICS
{

class Trajectory;

/**
      * Motion Planing for an object transfert point
      */
class OTPMotionPl : public HumanAwareMotionPlanner
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;


    OTPMotionPl();
    OTPMotionPl( Move3D::Robot* R, Move3D::Robot* H);

    ~OTPMotionPl();

    //getters and setters
    Move3D::Robot* getHuman() { return m_Human; }
    Move3D::Robot* getRobot() { return _Robot; }

    EnvGrid* getPlanGrid() { return m_2DGrid; }
    std::vector<Eigen::Vector3d> getOTPList() { return m_OTPList; }
    int getNbConf(){ return m_confList.size(); }

    Eigen::Vector3d getHumanActualPos();
    Eigen::Vector3d getRobotActualPos();

    void clearOTPList() { m_OTPList.clear(); }

    Move3D::ConfGenerator* getConfGenerator();

    std::vector<double> getHumanPos(){return m_humanPos;}
    Eigen::Vector3d getRobotPos(){return m_robotPos;}

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > getRobotTraj(){return m_2DPath;}
    void setRobotTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj){m_2DPath = traj;}

    std::vector<double> getMultipleData(){return m_multipleData;}

    double getInitTime() {return initTime;}
    void setInitTime(double d) {initTime = d;}


    /**
          * to compute traj or not
          */
    void compteTraj(bool compute);

    /**
          * Draws the 3D path as a yellow line for robot and green one for human
          */
    void draw2dPath();

    /**
          * Change Human (in case of multiple human in the scenne)
          */
    bool changeHumanByName(std::string humanName);

    //##########################################
    //functions for changing config list #######
    //##########################################

    //about the robot
    /**
          * Compute only PR2 GIK
          */
    bool ComputePR2Gik();

    /**
          * set Pr2 to a giving configuration
          */
    void initPR2GiveConf();

    /**
          * Compute human GIK
          */
    void placeHuman();


    // about the OTP list (of point to compute GIK)


    /**
          * load configs from filename and put them, either in sitting or standing list in the correct vector.
          */
    int loadConfsFromXML(std::string filename, bool isStanding, bool isSlice);

    /**
          * return the size of the selected config list
          */
    int getConfListSize();

    /**
          * return the config at i in the selected config list
          */
    configPt getRobotConfigAt(int i);

    /**
          * return the selected config list
          */
    std::vector<ConfigHR> getConfList();

    // after loading confs, computing distances
    /**
          * return the minimal and maximal dist between human and robot in the loaded configuration
          */
    std::pair<double,double> computeHumanRobotDist();

    /**
          * return  distance between human and robot
          */
    double getHumanRobotDist();

    //##########################################
    //Tools for Computing OTP ##################
    //##########################################

    //initialisation
    /**
          * init the plan grid with the costs and the distances
          */
    void initGrid();

    /**
          * called for initialisation
          */
    void initAll();

    //Place from a stored config
    /**
          * chamge robot configuration to the one with the specified id in the correct list
          * return a pair of the resulting human and robot configuration
          */
    std::pair< Move3D::confPtr_t, Move3D::confPtr_t > setRobotsToConf(int id, bool isStanding);

    /**
          * chamge robot configuration to the one with the specified id in the correct list and placing human according to x, y and Rz
          * return a pair of the resulting human and robot configuration
          */
    std::pair< Move3D::confPtr_t, Move3D::confPtr_t > setRobotsToConf(int id, bool isStanding, double x, double y, double Rz);

    // saving and reloading a configuration
    /**
          * save the actual robot and human conf to reload it afterwards
          */
    void saveInitConf();

    /**
          * reload the saved human and robot conf
          */
    void loadInitConf(bool reloadHuman, bool relaodRobot);

    //actual algorithm
    /**
          * compute a distance in a planar grid between a point (p) and a segment [p1,p2]
          */
    double ComputePlanarDistancesLineToSegment(Eigen::Vector2d p, Eigen::Vector2d p1, Eigen::Vector2d p2);

    /**
          * return the randomly choosed x, y and Rz. the random can be biased or not. the choice is done in the GUI
          */
    bool getRandomPoints(double id, Eigen::Vector3d& vect);

    /**
          * test if the vector has already been computed
          */
    bool isAlreadyTested(Eigen::Vector3d p);

    /**
          * Compute the OTP using the last algorithme
          */
    bool newComputeOTP();

    /**
          * Used in newComputeOTP(), this function compoute a conf for human an robot
          * from a position (x,y,Rz) of the human.
          */
    OutputConf lookForBestLocalConf(double x, double y, double Rz, double objectNecessity);

    /**
          * if the human is stitting, special features should be than.
          */
    OutputConf findBestPosForHumanSitConf(double objectNecessity);

    // redundant call of OTP Computing
    /**
          * getting the inputs from a unique source. this function permit using redundant call to the computing OTP function.
          * this function put the input in global variable that can be used by others.
          */
    void getInputs();

    /**
          * setting the inputs !!
          */
    void setInputs( Eigen::Vector3d humanPos, Eigen::Vector3d robotPos,bool isStanding, double mobility);

    /**
          * adding path to real trajectory
          */
    void addVectorToRealTreajectory(Eigen::Vector2d vect);

    /**
          * set robotPos to actual pos
          */
    void setRobotPos();

    /**
          * clear real trajectory
          */
    void clearRealTreajectory(){m_2DHumanRealPath.clear();}

    /**
          * compare the real trajectory with the predicted trajectory
          */
    bool isTheRealNearThePredicted(double threshold);

    //other
    /**
          * sort configuration stored by confort cost
          */
    void sortConfigList(double nbNode, bool isStanding, bool isSlice);

    /**
          * testing collision for human and robot
          */
    bool testCol(bool isHuman, bool useConf);

    /**
          * Put human to a standing position
          * the standing postition is right in from of him if there is no obstacle
          * if obstacles is detected, the human push bach the chair and stand up without moving.
          */
    bool standUp();


    // Showing the configurations
    /**
          * show a computed conf from it's number in the list
          */
    double showConf(unsigned int i);

    /**
        * show the best computed conf
        */
    OutputConf showBestConf();

    /**
        * get best conf
        */
    Move3D::confPtr_t getBestConf();

    /**
        * show the best computed conf for the robot only
        */
    void showBestConfRobOnly();


    //creating trajectories
    /**
          * create a trajectory from a configuration
          */
    bool createTrajectoryFromOutputConf(OutputConf conf);

    /**
          * generate a smoothed 2D traj
          */
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > smoothTrajectory(
            std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > trajectory);

    /**
          * test if current traj is feasable
          */
    bool testCurrentTraj();

    /**
          * init the class to begin testing
          */
    void initTrajTest();

    /**
          * planning using planNavigation
          */
    void navigate();

    /**
          * go to the next position of the cylinder
          */
    void goToNextStep();

    // results functions
    /**
          * save the costs to a file
          */
    void saveCostsTofile(double cost, double randomCost);

    /**
          * save all the costs computed in multipli conpute OTP in a file
          */
    void saveAllCostsToFile();

    /**
          * save all the times computed in multipli conpute OTP in a file
          */
    void saveAlltimesToFile();

    /**
          * save the costs to a file (take string as entry)
          */
    void saveCostsTofile(std::string cost);

    /**
          * clear the file before saving costs
          */
    void clearCostsfile();

    /**
          * compute OTP using newComputeOTP() n time, the returning the average
          */
    double multipliComputeOtp(int n);

    //##########################################
    //Tools for Using from as lib ##############
    //##########################################

    //about cells
    /**
          * get distance from human or robot of a cell.
          */
    double getDistFromCell(int x, int y, bool isHuman);

    /**
          * get rotation from human or robot of a cell.
          */
    double getrotFromCell(int x, int y);

    //about values
    /**
          * write cost of current configList
          */
    void dumpCosts();

    /**
          * writing variable
          */
    void dumpVar();

    // using from mhp
    /**
          * setting variables
          */
    void setVar();

    /**
          * function to be called from codels for softmotion usage
          */
    bool getOtp(std::string humanName, Eigen::Vector3d &dockPos, std::vector<SM_TRAJ>& smTraj, configPt& handConf,bool isStanding, double objectNessecity);


    /**
          * function to be called from codels
          */
    bool getOtp(std::string humanName, Eigen::Vector3d &dockPos, std::vector<std::pair<double,double> >& traj, configPt& handConf,bool isStanding, double objectNessecity);

    /**
          * function to be called from codels for ros usage
          */
    bool getOtp(std::string humanName, std::vector<std::vector<double> >& traj, configPt& handConf,bool isStanding, double objectNessecity);


    /**
          * initialisation of OTP computing for usage in mhp
          */
    bool InitMhpObjectTransfert(std::string humanName);

    /**
          * test pr2Softmotion
          */
    bool testTrajectories(bool fullbody);


    /**
          * get a simple path
          */
    bool getSimplePath(double x, double y, double theta, std::vector<std::vector<double> >& path);

    /**
          * test if the human has moved according to the plan (3d and sitting)
          */
    bool hasHumanMovedAccordingToPlan(double error);

private:

    /**
          * the human robot
          */
    Move3D::Robot* m_Human;

    /**
          * The grid used for all the computations
          */
    EnvGrid* m_2DGrid;

    /**
          * robot path to be draw
          */
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m_2DPath;

    /**
          * robot 3d path for Ros
          */
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > m_robotTraj3D;

    /**
          * human path to be draw
          */
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m_2DHumanPath;

    /**
          * human real path
          */
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m_2DHumanRealPath;

    /**
          * Manipulation planner for robot usage
          */
    ManipulationPlanner* m_ManipPl;

    /**
          * Manipulation planner for human usage
          */
    ManipulationPlanner* m_ManipPlHum;

    /**
          * if the robot path exist
          */
    bool m_PathExist;

    /**
          * if the human path exist
          */
    bool m_HumanPathExist;

    /**
          * the list of OTPs used to create a new config list
          */
    std::vector<Eigen::Vector3d> m_OTPList;

    /**
          * store configs (for standing human when loading)
          */
    std::vector<ConfigHR> m_configList;

    /**
          * store configs (for sitting human when loading)
          */
    std::vector<ConfigHR> m_sittingConfigList;

    /**
          * store configs in a slice (for standing human when loading)
          */
    std::vector<ConfigHR> m_configListSlice;

    /**
          * store configs in a slice (for sitting human when loading)
          */
    std::vector<ConfigHR> m_sittingConfigListSlice;

    /**
          * to save and reload configuration
          */
    OutputConf m_savedConf;

    /**
          * save different configuration in order to show them afterwards
          */
    std::vector<OutputConf> m_confList;

    /**
          * is the human is initially sitting
          */
    bool m_isInitSiting;

    /**
          * vector used for choosing randoms when using slices
          */
    Eigen::Vector3d m_sliceVect;

    /**
          * is at false if the human cannot stand
          */
    bool m_humanCanStand;

    /**
          * time to compute OTP
          */
    double m_time;

    /**
          * time to compute sitting OTP
          */
    double m_sittingTime;

    /**
          * the data used in multiplyCompute OTP
          * 1: totalTime, 2: initGrid time,3: first config time, 4: loop time, 5: cost, 6:nb Iterations, 7:nb Found solutions
           */
    std::vector<double> m_multipleData;

    /**
          * the chair where the human is sitting
          */
    Move3D::Robot* m_simpleChair;

    /**
          * a vector of all the costs computed when computing a bunch of OTPs
          */
    std::vector<std::vector<double> > m_multipliComputeCostVector;

    /**
          * a vector of all the time coresponding to costs computed when computing a bunch of OTPs
          */
    std::vector<std::vector<double> > m_multipliComputeTimeVector;

    /**
          * The cost vector
          */
    std::vector<double> m_costVector;

    /**
          * The time cost vector
          */
    std::vector<double> m_timeVector;

    /**
          * a vector for softmotions trajectories
          */
    std::vector<SM_TRAJ> m_smTrajs;

    /**
          * an object to compute 2d trajectory
          */
    Move3D::PlannarTrajectorySmoothing* m_pts;

    /**
          * a list of all the tested vectors
          */
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> >   m_testedVectors;

    /**
          * inputs for computing OTP
          */
    //        Eigen::Vector3d m_humanPos;
    std::vector<double> m_humanPos;
    Eigen::Vector3d m_robotPos;
    bool m_isStanding;
    double m_mobility;

    /**
          *
          */
    Move3D::ConfGenerator* m_ConfGen;

    /**
          * initialisation time
          */
    double initTime;

    bool m_OTPsuceed;



};



}

#endif // HRICS_OTPMOTIONPL_HPP
