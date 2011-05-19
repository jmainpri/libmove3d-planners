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
#include "planner/planner.hpp"

#include "HRICS_Workspace.hpp"

#include "Grid/HRICS_EnvGrid.hpp"
#include "Grid/HRICS_TwoDGrid.hpp"

#include <Eigen/StdVector>

/**
 @defgroup HRICS Hri Cost space
 */

/**
 @ingroup HRICS
 */
namespace HRICS
{
    class ConfigHR
    {
    public:

        static int index;
        ConfigHR() { id = index++; }

        configPt getHumanConf(){ return q_hum; }
        void setHumanConf(Robot* human, configPt q);

        configPt getRobotConf(){ return q_rob; }
        void setRobotConf(Robot* robot, configPt q);

        int getId(){ return id; }

    private:
        configPt q_hum;
        configPt q_rob;
        int id;

    };


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


        bool computeAStarIn2DGrid();

        void solveAStar(EnvState* start, EnvState* goal, bool isHuman);
        void draw2dPath();

        bool simpleComputeBaseAndOTP();
        bool OTPonly(int th);
        bool computeObjectTransfertPoint();

        bool ComputePR2Gik();

        bool FindTraj(Eigen::Vector2d startPos, Eigen::Vector2d goalPos, bool isHuman);
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

        std::vector<ConfigHR> addConfToList();
        void removeLastConf();
        void saveToXml(std::string filename);
        int loadFromXml(std::string filename);
        void setRobotsToConf(int id);

//        bool saveToXml


    private:
        void initHumanCenteredGrid(double cellsize);

        void initAll();

        Robot* m_Human;
        EnvGrid* m_2DGrid;
        EnvGrid* m_2DHumanCenteredGrid;

        std::vector<double> m_EnvSize;
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m_2DPath;
        std::vector<API::TwoDCell*> m_2DCellPath;

        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m_2DHumanPath;
        std::vector<API::TwoDCell*> m_2DHumanCellPath;



        bool m_PathExist;
        bool m_HumanPathExist;

        unsigned int m_pathIndex;

        std::vector<Eigen::Vector3d> m_OTPList;
        std::vector<ConfigHR> m_configList;

    };

}

#endif // HRICS_OTPMOTIONPL_HPP
