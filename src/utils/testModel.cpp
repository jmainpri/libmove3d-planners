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
/*
 * testModel.cpp
 *
 *  Created on: Jul 8, 2009
 *      Author: jmainpri
 */
#include "testModel.hpp"

#include "API/project.hpp"
#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/localpath.hpp"

#include <iostream>

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"

//#include "Greedy/CollisionSpace.h"

using namespace Move3D;
using namespace std;

MOVE3D_USING_SHARED_PTR_NAMESPACE

TestModel::TestModel() :
        nbColisionTest(10000000), nbLocalPathTest(1000000)
{
    modelRobot = new Robot(XYZ_ROBOT);
    cout << modelRobot->getName() << endl;
}

int TestModel::nbOfVoxelCCPerSeconds()
{
    double tu, ts;
    int nbTested(0);
    int nbInCol(0);
    ChronoOn();
	
	Robot* robotPt = global_Project->getActiveScene()->getActiveRobot();

    for (int i = 0;; i++)
    {
			robotPt->setAndUpdate(*robotPt->shoot());
       // global_collisionSpace->updateRobotOccupationCells(robotPt);
			cout << "Warning not imlemented" << endl;

        ChronoTimes(&tu, &ts);
        if (tu > 5)
        {
            nbTested = i + 1;
            cout << "Voxel in collision = " << ((double) nbInCol
												   / (double) nbTested) << endl;
            break;
        }
    }

    ChronoPrint("");
    ChronoTimes(&tu, &ts);
    ChronoOff();
    double val = (double) nbTested / tu;
#ifdef QT_LIBRARY
    QString str = QString("%1 Voxel Collision per second").arg(val);
    ENV.setString(Env::numberOfCollisionPerSec,str.toStdString());
#endif
    return (int) val;
}

int TestModel::nbOfColisionsPerSeconds()
{
    double tu, ts;
    int nbTested(0);
    int nbInCol(0);
    ChronoOn();
	
    for (unsigned int i = 0;; i++)
    {
        if (modelRobot->shoot()->isInCollision())
        {
            nbInCol++;
        }
		
        ChronoTimes(&tu, &ts);
        if (tu > 5)
        {
            nbTested = i + 1;
            cout << "Percenatge in collision = " << ((double) nbInCol
                                                     / (double) nbTested) << endl;
            break;
        }
    }
	
    ChronoPrint("");
    ChronoTimes(&tu, &ts);
    ChronoOff();
    double val = (double) nbTested / tu;
#ifdef QT_LIBRARY
    QString str = QString("%1 Collision Test per second").arg(val);
    ENV.setString(Env::numberOfCollisionPerSec,str.toStdString());
#endif
    return (int) val;
}

int TestModel::nbOfCostPerSeconds()
{

    double tu, ts;
//    int nbTested(0);
    int nbCost(0);
    ChronoOn();

    cout << "Model is "<< modelRobot->getName() << endl;

    for (int i = 0;; i++)
    {
        modelRobot->shoot()->cost();
        nbCost++;


        ChronoTimes(&tu, &ts);
        if (tu > 3)
        {
            break;
        }
    }

    ChronoPrint("");
    ChronoTimes(&tu, &ts);
    ChronoOff();
    double val = (double) nbCost / tu;
#ifdef QT_LIBRARY
    QString str = QString("%1 Cost per second").arg(val);
    ENV.setString(Env::numberOfCostPerSec,str.toStdString());
#endif
    return (int) val;
}

void TestModel::distEnv()
{

    confPtr_t q;

    for (int i = 0; i < 100; i++)
    {
        q = modelRobot->shoot(true);
        //			q->print();
        //			cout << "Conf is In Colision = " << (int)q->isInCollision() << endl;
        cout << "Distance from obst = " << (double) q->distEnv() << endl;
    }

    return;
}

int TestModel::nbOfLocalPathsPerSeconds()
{

    confPtr_t current = modelRobot->getCurrentPos();

    confPtr_t q1;
    confPtr_t q2;

    double tu, ts;
    ChronoOn();
    int nbLP = 0;
    int nbLPtot = 0;
    int nbColTest = 0;
    int nbLPValid = 0;
    int nbMaxTest = 0;
    int nbTest = 0;

    vector<double> dist;

    double x = ENV.getDouble(Env::extensionStep) * ENV.getDouble(Env::dmax);

    for (int i = 0;/*i<100*/; i++)
    {

        ChronoTimes(&tu, &ts);
        if (tu > 10)
        {
            nbLPtot = i;
            break;
        }

        q1 = modelRobot->shoot();

        if ( (!q1->setConstraints()) || q1->isInCollision() )
        {
            continue;
        }

        q2 = modelRobot->shoot();
        LocalPath LP1(q1, q2);


        q2 = LP1.configAtParam(x);
        q2->setConstraints();

        LocalPath LP2(q1, q2);

        nbLP++;
        if (LP2.isValid())
        {
            nbLPValid++;
        }

        nbTest = LP2.getNbColTest();
        nbColTest += nbTest;

        dist.resize(aveBBDist.size());
        for (unsigned int i = 0; i < dist.size(); i++)
        {
            dist[i] += aveBBDist[i];
        }

        if (nbMaxTest < nbTest)
        {
            nbMaxTest = nbTest;
        }
    }

    modelRobot->setAndUpdate(*current);

    cout << "Nb Tested = " << nbLP << endl;
    cout << "Nb Valid = " << nbLPValid << endl;
    cout << "Ratio of Valid/Total = " << (double) nbLPValid / (double) nbLP
            << endl;
    cout << "----------------------------------" << endl;
    cout << "nbColTest/sec = " << (double) nbColTest / 10 << endl;
    cout << "nbColTest/LP = " << (double) nbColTest / (double) nbLP << endl;
    cout << "nbColTestMax/LP = " << nbMaxTest << endl;

    ChronoPrint("");
    ChronoTimes(&tu, &ts);
    ChronoOff();
    double val = (double) nbLPtot / tu;
#ifdef QT_LIBRARY
    QString str = QString("%1 LocalPaths per second").arg(val);
    ENV.setString(Env::numberOfLocalPathPerSec,str.toStdString());
#endif
    return (int) val;
}

void TestModel::runAllTests()
{

    cout << "StarTingTests -----------------------------" << endl;
    //	cout << "nbColisionTest = " << nbColisionTest << endl;
    //	cout << "nbLocalPathTest = " << nbLocalPathTest << endl;
    int costPerSec(0);

    int colPerSec = nbOfColisionsPerSeconds();
    int lpPerSec = nbOfLocalPathsPerSeconds();

    if(ENV.getBool(Env::isCostSpace))
    {
        costPerSec = nbOfCostPerSeconds();
    }

    cout << colPerSec << " Collisions per second and ";
    cout << lpPerSec << " Local Paths per second" << endl;

#ifdef QT_LIBRARY
    QString str1 = QString("%1 Collisions per second").arg(colPerSec);
    ENV.setString(Env::numberOfCollisionPerSec,str1.toStdString());
    QString str2 = QString("%1 LocalPaths per second").arg(lpPerSec);
    ENV.setString(Env::numberOfLocalPathPerSec,str2.toStdString());
#endif

    if(ENV.getBool(Env::isCostSpace))
    {
        cout << costPerSec << " Cost computation per second " << endl;
#ifdef QT_LIBRARY
        QString str3 = QString("%1 Cost per second").arg(costPerSec);
        ENV.setString(Env::numberOfCostPerSec,str3.toStdString());
#endif
    }

    cout << " -- End tests--" << endl;

}
