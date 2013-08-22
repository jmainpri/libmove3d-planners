/*
 *  localPathValidTest.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 12/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "API/ConfigSpace/localPathValidTest.hpp"
#include "API/Device/robot.hpp"

#include "P3d-pkg.h"
#include "Collision-pkg.h"

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

const bool forward_s = true;
const bool backward_s = false;

LocalPathValidTest::LocalPathValidTest(const LocalPath& LP) : 
    LocalPath(LP),
    mIsClassicVsDicho(true),
    mDoSelf(true),
    mDoStatics(true),
    mDoOthers(true),
    mMicrocollisionAvoidance(false),
    mNbTest(0)
{

}

LocalPathValidTest::~LocalPathValidTest()
{

}

void LocalPathValidTest::setClassicTest(bool isClassic)
{
    mIsClassicVsDicho = isClassic;
}

unsigned int LocalPathValidTest::getNbCollisionTest()
{
    return mNbTest;
}

shared_ptr<Configuration> LocalPathValidTest::getLastValidConfiguration()
{
    return mLastValidConfiguration;
}

bool LocalPathValidTest::testIsValid()
{
    bool valid = false;

    if (mIsClassicVsDicho)
    {
        valid =! LocalPathValidTest::testClassic();

        if (valid)
        {
            //cout << "is Valid "<< endl;
        }
        else {
            //cout << "is Not Valid!!!!"<< endl;
        }

        return valid;
    }
    else
    {
        return testDichotomic();
    }

}

bool LocalPathValidTest::testClassic()
{	
    double u = 0., du, umax; /* parameters along the local path */
    //	configPt qp;
    int njnt = _Robot->getRobotStruct()->njoints;
    double* distances;
    double tolerance, newtol, dmax, dist0;
    int end_localpath = 0;

    //p3d_reset_current_q_inv(robotPt);

    //Carefull
    //double Kpath = 0;
    double* q_atKpath = NULL;
    //shared_ptr<Configuration> q_atKpath;

    p3d_col_get_dmax(&dmax);

    if (this->getLocalpathStruct() == NULL)
    {
        return false;
    }

    //cout << "Classic Test " << endl;

    /* Some curves can be decided unvalid by the user */
    if (this->getLocalpathStruct()->valid == false)
    {
        return true;
    }

    umax = this->getParamMax();

    distances = new double[njnt+1];
    tolerance = 0.0;
    if (/*p3d_col_get_tolerance(&tolerance) &&*/ mMicrocollisionAvoidance)
    {
        p3d_col_set_tolerance(tolerance + dmax);
        newtol = tolerance + dmax;
        dist0 = dmax;
    }
    else
    {
        newtol = 0;
        dist0 = 2 * dmax;
    }

    /* current position of robot is saved */
    shared_ptr<Configuration> qp = this->configAtParam(0.0);

    /* We suppose that the fisrt and the last configurations are valid so
     we could test the configuration at dist0 from the bounds */
    /* Remove the end of interval */
    u = umax;

    if (this->changePositionRobot(u)) {   // modif Juan
        /* The initial position of the robot is recovered */
        _Robot->setAndUpdateWithoutConstraints(*qp);
        p3d_col_set_tolerance(tolerance);
        return true;
    }

    for (int j = 0; j <= njnt; j++)
    {
        distances[j] = dist0;
    }
    /* Compute the lenght of left interval */
    /* Compute the lenght of left interval */
    /* Modif EF: the end of the path is not removed for special use of the RRT algorithm */
    /*
     du = localpathPt->stay_within_dist(robotPt, localpathPt, u,
     BACKWARD, distances);
     umax = umax - du;
     */
    if (umax < EPS6)
    {
        /* The initial position of the robot is recovered */
        _Robot->setAndUpdateWithoutConstraints(*qp);
        p3d_col_set_tolerance(tolerance);
        return false;
    }

    /* Remove the beginning of interval */
    u = 0;

    if (this->changePositionRobot(u)) {
        /* The initial position of the robot is recovered */
        _Robot->setAndUpdateWithoutConstraints(*qp);
        p3d_col_set_tolerance(tolerance);
        return true;
    }

    for (int j = 0; j <= njnt; j++)
    {
        distances[j] = dist0;
    }

    du = this->stayWithInDistance(u,forward_s,distances);

    u = du;

    if (u > umax - EPS6) {
        u = umax;
        end_localpath++;
    }

    dist0 = 2 * dmax - newtol; /* Minimal distance we could cross at each step */

    while (end_localpath < 2)
    {
        /* position of the robot corresponding to parameter u */
        if ( this->changePositionRobot(u) )
        {
            /* The initial position of the robot is recovered */
            _Robot->setAndUpdateWithoutConstraints(*qp);
            p3d_col_set_tolerance(tolerance);
            return true;
        }

        qp = _Robot->getCurrentPos();

        // TEMP MODIF : PROBLEM WITH SELF COLLISION : CONSTANT STEP
        p3d_BB_dist_robot(_Robot->getRobotStruct(), distances);

        int test = false;
        for (int j=0; j<=njnt; j++)
        {
            if (distances[j] < newtol + EPS6)
            {
                test = true;
            }
            distances[j] += dist0;
            //cout << "distance["<<j<<"]=" << distances[j] << endl;
        }

        if (test)
        {
            /////////////////////////
            /* collision checking */
            //cout << "Test" << endl;
            mNbTest ++;
            if (p3d_col_test())
            {
                /* The initial position of the robot is recovered */
                p3d_set_current_q_inv(_Robot->getRobotStruct(), getLocalpathStruct(), qp->getConfigStruct());
                _Robot->setAndUpdateWithoutConstraints(*qp);
                p3d_col_set_tolerance(tolerance);
                return true;
            }
        }

        // TEMP MODIF ///////
        /* Modif. Etienne: if collision detector computed distances*/

        //double Kpath = u / this->getParamMax();

        //		if (q_atKpath)
        //		{
        //			q_atKpath = _Robot->getCurrentPos();
        //		}
        du = this->stayWithInDistance(u,forward_s,distances);

        u += du;

        if (u > umax - EPS6)
        {
            u = umax;
            end_localpath++;
        }

    }

    //Kpath = 1.0;

    // WARNING : NEXT LINES ARE VALID IN THE CASE OF CONSTRAINTS ???
    if( q_atKpath != NULL )
    {
        qp = this->configAtParam(u);
        //p3d_copy_config_into(robotPt, qp, &q_atKpath);
    }

    delete[] distances;
    p3d_col_set_tolerance(tolerance);
    return false;
}

bool LocalPathValidTest::testDichotomic()
{
    return false;
}

bool LocalPathValidTest::changePositionRobot(double l) 
{
    shared_ptr<Configuration> q = configAtParam(l);
    if (!_Robot->setAndUpdate(*q))
    {
        return true;
    }
    return invalidJointLimits();
}

bool LocalPathValidTest::changePositionRobotWithoutCntrt(double l) 
{
    shared_ptr<Configuration> q = configAtParam(l);
    _Robot->setAndUpdateWithoutConstraints(*q);
    return invalidJointLimits();
}

bool LocalPathValidTest::invalidJointLimits()
{
    p3d_jnt* jntPt;
    double v, v_min, v_max;

    /* test that joints keep in their bounds */
    for (int i = 0;  i <= _Robot->getRobotStruct()->njoints; i++)
    {
        jntPt = _Robot->getRobotStruct()->joints[i];
        for (int j = 0; j < jntPt->dof_equiv_nbr; j++)
        {
            p3d_jnt_get_dof_bounds(jntPt, j, &v_min, &v_max);
            v = p3d_jnt_get_dof(jntPt, j);
            if (((v > v_max + EPS6) || (v < v_min - EPS6)) &&
                    !p3d_jnt_is_dof_circular(jntPt, j))
            {
                return true;
            }
        }
    }
    return false;
}
