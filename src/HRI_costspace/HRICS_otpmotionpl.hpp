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
#include "API/planningAPI.hpp"

#include "API/Trajectory/trajectory.hpp"
#include "planner/planner.hpp"

#include "HRICS_Workspace.hpp"

#include "Grid/HRICS_EnvGrid.hpp"
#include "Grid/HRICS_TwoDGrid.hpp"

#include "LightPlanner-pkg.h"
#include "planner/TrajectoryOptim/plannarTrajectorySmoothing.hpp"


#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <Eigen/StdVector>

extern void g3d_show_tcur_both_rob(p3d_rob *robotPt, int (*fct)(p3d_rob* robot, p3d_localpath* curLp),
                                   p3d_rob *hum_robotPt, int (*hum_fct)(p3d_rob* hum_robot, p3d_localpath* hum_curLp));

/**
 @defgroup HRICS Hri Cost space
 */

/**
 @ingroup HRICS
 */
namespace HRICS
{

    class OutputConf
    {
    public:
        std::tr1::shared_ptr<Configuration> humanConf;
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > humanTraj;
        bool humanTrajExist;

        std::tr1::shared_ptr<Configuration> robotConf;
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > robotTraj;
        bool robotTrajExist;

        double cost;
        int configNumberInList;
        int id;
        bool isStandingInThisConf;

        void clearAll();

        std::tr1::shared_ptr<Configuration> chairConf;
//        OutputConf& operator= (const OutputConf& o);

    };


    class ConfigHR
    {
    public:

        static int index;
        ConfigHR() { id = index++; cost = 1.; }

        configPt getHumanConf(){ return q_hum; }
        void setHumanConf(Robot* human, configPt q);

        configPt getRobotConf(){ return q_rob; }
        void setRobotConf(Robot* robot, configPt q);

        int getId(){ return id; }
        void setId(int value) { id = value; }

        double getCost() { return cost; }
        void setCost(double value) { cost = value; }

    private:
        configPt q_hum;
        configPt q_rob;
        int id;
        double cost;

    };

    class Trajectory;

    /**
      * Motion Planing for an object transfert point
      */
    class OTPMotionPl : public HumanAwareMotionPlanner
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;


        OTPMotionPl();
        OTPMotionPl(Robot* R, Robot* H);

        ~OTPMotionPl();

        //getters and setters
        Robot* getHuman() { return m_Human; }
        Robot* getRobot() { return _Robot; }

        EnvGrid* getPlanGrid() { return m_2DGrid; }
        std::vector<Eigen::Vector3d> getOTPList() { return m_OTPList; }
        int getNbConf(){ return m_confList.size(); }

        Eigen::Vector3d getHumanActualPos();
        Eigen::Vector3d getRobotActualPos();

        void clearOTPList() { m_OTPList.clear(); }

        std::vector<double> getHumanPos(){return m_humanPos;}
        Eigen::Vector3d getRobotPos(){return m_robotPos;}

        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > getRobotTraj(){return m_2DPath;}
        void setRobotTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj){m_2DPath = traj;}

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
          * Compute GIK and place robot base
          */
        bool placeRobot();

        // about the human
        /**
          * Compute human GIK
          */
        void placeHuman();


        // about the OTP list (of point to compute GIK)
        /**
          * Adding a 3D point to the OTP list This function is used when loading a set of OTPs in order to test them
          */
        void addToList(Eigen::Vector3d WSPoint);

        /**
          * Change the current OTP. Used when choosing new configurations
          */
        void setCurrentOTP(Eigen::Vector3d WSPoint);

        /**
          * draw the OTP list
          */
        void drawOTPList(bool value);

        //about the configuration to store/stored
        /**
          * Adding the actual configuration to m_configList
          */
        std::vector<ConfigHR> addConfToList();

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
        std::vector<ConfigHR> loadFromXml(std::string filename);

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
        std::pair<std::tr1::shared_ptr<Configuration>, std::tr1::shared_ptr<Configuration> > setRobotsToConf(int id, bool isStanding);

        /**
          * chamge robot configuration to the one with the specified id in the correct list and placing human according to x, y and Rz
          * return a pair of the resulting human and robot configuration
          */
        std::pair<std::tr1::shared_ptr<Configuration>, std::tr1::shared_ptr<Configuration> > setRobotsToConf(int id, bool isStanding, double x, double y, double Rz);

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
        Eigen::Vector3d getRandomPoints(double id);

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
        * show the best computed conf for the robot only
        */
        void showBestConfRobOnly();


        //creating trajectories
        /**
          * create a trajectory from a configuration
          */
        bool createTrajectoryFromOutputConf(OutputConf conf);

        /**
          * generate a trajectory for the Arm
          */
        bool computeArmMotion(double* qInit, double* qGoal, API::Trajectory& traj);

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
          * function to be called from codels
          */
        bool getOtp(std::string humanName, Eigen::Vector3d &dockPos, std::vector<SM_TRAJ>& smTraj, configPt& handConf,bool isStanding, double objectNessecity);


        /**
          * function to be called from codels
          */
        bool getOtp(std::string humanName, Eigen::Vector3d &dockPos, std::vector<std::pair<double,double> >& traj, configPt& handConf,bool isStanding, double objectNessecity);

        /**
          * initialisation of OTP computing for usage in mhp
          */
        bool InitMhpObjectTransfert(std::string humanName);

    private:

        /**
          * the human robot
          */
        Robot* m_Human;

        /**
          * The grid used for all the computations
          */
        EnvGrid* m_2DGrid;

        /**
          * robot path to be draw
          */
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m_2DPath;

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
        Robot* m_simpleChair;

        /**
          * a vector of all the costs computed when computing a bunch of OTPs
          */
        std::vector<std::vector<double> > m_multipliComputeCostVector;

        /**
          * The cost vector
          */
        std::vector<double> m_costVector;

        /**
          * a vector for softmotions trajectories
          */
        std::vector<SM_TRAJ> m_smTrajs;

        /**
          * an object to compute 2d trajectory
          */
        PlannarTrajectorySmoothing* m_pts;

        /**
          * inputs for computing OTP
          */
//        Eigen::Vector3d m_humanPos;
        std::vector<double> m_humanPos;
        Eigen::Vector3d m_robotPos;
        bool m_isStanding;
        double m_mobility;



    };

}

#endif // HRICS_OTPMOTIONPL_HPP
