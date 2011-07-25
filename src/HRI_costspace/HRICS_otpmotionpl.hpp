#ifndef HRICS_OTPMOTIONPL_HPP
#define HRICS_OTPMOTIONPL_HPP

/*
 *  HRICS_CostSpace.h
 *  BioMove3D
 *
 *  Created by Mamoun Gharbi on 20/04/11.
 *  Copyright 2009 magharbi@laas.fr All rights reserved.
 *
 */
#include "API/planningAPI.hpp"
#include "API/Trajectory/trajectory.hpp"
#include "planner/planner.hpp"

#include "HRICS_Workspace.hpp"

#include "Grid/HRICS_EnvGrid.hpp"
#include "Grid/HRICS_TwoDGrid.hpp"

#include "LightPlanner-pkg.h"

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
        OTPMotionPl();
        OTPMotionPl(Robot* R, Robot* H);

        ~OTPMotionPl();

        EnvGrid* getPlanGrid() { return m_2DGrid; }


        /**
          * use solveAStar to find a trajectory between initial and final conf of each robot
          */
        bool computeAStarIn2DGrid();

        /**
          * find a path in m_2DGrid between start and goal position
          * start : the position for robot start
          * goal : the final position of the robot
          * isHuman : true if the robot is the Human false otherwise.
          */
        void solveAStar(EnvState* start, EnvState* goal, bool isHuman);

        /**
          * Draw the computed path
          */
        void draw2dPath();

        /**
          * compute base placement and OTP only
          */
        bool simpleComputeBaseAndOTP();

        /**
          * Compute only the OTP for the human. th is a thershold for changing the OTP
          */
        bool OTPonly(int th);

        /**
          * a complete function to compute : robot base placement, PR2 GIK and human OTP
          */
        bool computeObjectTransfertPoint();

        /**
          * Compute only PR2 GIK
          */
        bool ComputePR2Gik();

        /**
          * Find trajectory for either human or robot, and co;puting the distance of this trajectory
          */
        double FindTraj(Eigen::Vector2d startPos, Eigen::Vector2d goalPos, bool isHuman);
        bool computeUpBodyOpt();

        bool moveToNextPos();
        void SetPathIndexNull(){ m_pathIndex = -1; }

        void initPR2GiveConf();

        bool computeProx(Eigen::Vector3d WSPoint, int proximiti);
        bool computeHight(Eigen::Vector3d WSPoint, int hight);
        bool computeAngle(Eigen::Vector3d WSPoint, int angle);

        void addToList();
        void addToList(Eigen::Vector3d WSPoint);
        void removeLastFromOTPList() {m_OTPList.pop_back(); }
        std::vector<Eigen::Vector3d> getOTPList() { return m_OTPList; }
        void showOTPList();

        void setCurrentOTP(Eigen::Vector3d WSPoint);
        void clearOTPList() { m_OTPList.clear(); }
        bool placeRobot();
        void placeHuman();

        void drawOTPList(bool value);

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
<<<<<<< Updated upstream

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
        double computeConfigCost(configPt q_rob_initial,
                                 configPt q_rob_final,
                                 configPt q_hum_initial,
                                 configPt q_hum_final);

        double testComputeConfigCost();

        /**
          * sort configuration stored by confort cost
          */
        void sortConfigList(double nbNode, bool isStanding, bool isSlice);

        void initHumanCenteredGrid(double cellsize);

        /**
          * compute OTP using newComputeOTP() n time, the returning the average
          */
        double multipliComputeOtp(int n);

        /**
          * Compute the OTP using the last algorithme
          */
        void newComputeOTP();

        /**
          * Used in newComputeOTP(), this function compoute a conf for human an robot
          * from a position (x,y,Rz) of the human.
          */
        OutputConf lookForBestLocalConf(double x, double y, double Rz, double objectNecessity);

        /**
          * if the human is stitting, special features should be than.
          */
        OutputConf findBestPosForHumanSitConf(double objectNecessity);

        /**
          * testing collision for human and robot
          */
        bool testCol(bool isHuman, bool useConf);

        /**
          * save the actual robot and human conf to reload it afterwards
          */
        void saveInitConf();

        /**
          * reload the saved human and robot conf
          */
        void loadInitConf(bool reloadHuman, bool relaodRobot);

        /**
          * show a computed conf from it's number in the list
          */
        double showConf(unsigned int i);

        /**
        * get the number of conf computed
        */
        int getNbConf(){ return confList.size(); }

        /**
        * show the best co;puted conf
        */
        OutputConf showBestConf();

        /**
          * init the plan grid with the costs and the distances
          */
        void initGrid();

        /**
          * get distance from human or robot of a cell.
          */
        double getDistFromCell(int x, int y, bool isHuman);

        /**
          * get rotation from human or robot of a cell.
          */
        double getrotFromCell(int x, int y);

        /**
          * save the costs to a file
          */
        void saveCostsTofile(double cost, double randomCost);

        /**
          * save the costs to a file (take string as entry)
          */
        void saveCostsTofile(std::string cost);

        /**
          * clear the file before saving costs
          */
        void clearCostsfile();


        /**
          * create a trajectory from a configuration
          */
        void createTrajectoryFromOutputConf(OutputConf conf);

        /**
          * generate a trajectory for the Arm
          */
        bool computeArmMotion(double* qInit, double* qGoal, API::Trajectory& traj);

        /**
          * return the minimal and maximal dist between human and robot in the loaded configuration
          */
        std::pair<double,double> computeHumanRobotDist();

        /**
          * return  distance between human and robot
          */
        double getHumanRobotDist();

        /**
          * return the randomly choosed x, y and Rz
          */
        Eigen::Vector3d getRandomPoints(double id);

        /**
          * write cost of current configList
          */
        void dumpCosts();

        /**
          * Put human to a standing position
          * the standing postition is right in from of him if there is no obstacle
          * if obstacles is detected, the human push bach the chair and stand up without moving.
          */
        bool standUp();

=======
        int loadFromXml(std::string filename);
        int getConfListSize() {return m_configList.size();}
        configPt getRobotConfigAt(int i) {return m_configList.at(i).getHumanConf(); }
        std::vector<ConfigHR> getConfList() {return m_configList;}
        void setRobotsToConf(int id);
        double computeConfigCost(configPt q_rob_initial,
                                 configPt q_rob_final,
                                 configPt q_hum_initial,
                                 configPt q_hum_final);

        double testComputeConfigCost();

        void sortConfigList(double nbNode);

        void initHumanCenteredGrid(double cellsize);
>>>>>>> Stashed changes
    private:


        void initAll();

        Robot* m_Human;
        EnvGrid* m_2DGrid;
        EnvGrid* m_2DHumanCenteredGrid;

        std::vector<double> m_EnvSize;
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m_2DPath;
        std::vector<API::TwoDCell*> m_2DCellPath;

        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m_2DHumanPath;
        std::vector<API::TwoDCell*> m_2DHumanCellPath;

        ManipulationPlanner* m_ManipPl;
        ManipulationPlanner* m_ManipPlHum;


        bool m_PathExist;
        bool m_HumanPathExist;
        bool m_noPath;

        unsigned int m_pathIndex;

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
        OutputConf savedConf;

        /**
          * save different configuration in order to show them afterwards
          */
        std::vector<OutputConf> confList;

        /**
          * is the human is initially sitting
          */
        bool isInitSiting;

        /**
          * vector used for choosing randoms when using slices
          */
        Eigen::Vector3d sliceVect;

        /**
          * is at false if the human cannot stand
          */
        bool m_humanCanStand;

        /**
          * time to compute OTP
          */
        double m_time;

        /**
          * to show or hide text when computing OTP
          */
        bool m_showText;

        /**
          * the chair where the human is sitting
          */
        Robot* m_simpleChair;

    };

}

#endif // HRICS_OTPMOTIONPL_HPP
