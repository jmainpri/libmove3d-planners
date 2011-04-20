/*
 *  HRICS_CostSpace.h
 *  BioMove3D
 *
 *  Created by Mamoun Gharbi on 20/04/11.
 *  Copyright 2009 magharbi@laas.fr All rights reserved.
 *
 */

#include "HRICS_otpmotionpl.hpp"
#include "Grid/HRICS_Grid.hpp"

#include "P3d-pkg.h"
#include "move3d-headless.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

extern HRICS::HumanAwareMotionPlanner*		HRICS_MotionPLConfig;
extern string global_ActiveRobotName;

OTPMotionPl::OTPMotionPl() : HumanAwareMotionPlanner()
{
    cout << "New OTPMotionPl HRI planner" << endl;

    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);

        if(name.find(global_ActiveRobotName) != string::npos )
        {
            _Robot = new Robot(XYZ_ENV->robot[i]);
            cout << "Robot is " << name << endl;
        }

        if(name.find("HUMAN") != string::npos )
        {
            mHuman = new Robot(XYZ_ENV->robot[i]);
            cout << "Humans is " << name << endl;
        }
    }
    initCostSpace();
}


OTPMotionPl::OTPMotionPl(Robot* R, Robot* H) : HumanAwareMotionPlanner() , mHuman(H)
{
    this->setRobot(R);
    initCostSpace();
}

OTPMotionPl::~OTPMotionPl()
{
    delete m2DGrid;
}


void OTPMotionPl::initCostSpace()
{
    mEnvSize.resize(4);
    mEnvSize[0] = XYZ_ENV->box.x1; mEnvSize[1] = XYZ_ENV->box.x2;
    mEnvSize[2] = XYZ_ENV->box.y1; mEnvSize[3] = XYZ_ENV->box.y2;

    m2DGrid = new EnvGrid(ENV.getDouble(Env::PlanCellSize),mEnvSize);
    m2DGrid->setRobot(_Robot);
}
