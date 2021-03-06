/*
 *  HRICS_CostSpace.h
 *  BioMove3D
 *
 *  Created by Mamoun Gharbi on 20/04/11.
 *  Copyright 2009 magharbi@laas.fr All rights reserved.
 *
 */

#include "HRICS_otpmotionpl.hpp"
#include "HRICS_costspace.hpp"
#include "grid/HRICS_grid.hpp"

#include "API/project.hpp"

#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"

#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/move3d-headless.h>
#include <libmove3d/include/Planner-pkg.h>
#include <libmove3d/include/Collision-pkg.h>
#include <libmove3d/hri/HRI_tasks.h>

#include "time.h"

#include <fstream>

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#include "lightPlanner/proto/ManipulationViaConfPlanner.hpp"
#endif

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE
using namespace HRICS;
using namespace Move3D;

// import most common Eigen types
//using namespace Eigen;
using namespace Eigen;

//extern HRICS::HumanAwareMotionPlanner*		HRICS_MotionPLConfig;



//extern std::vector<Eigen::Vector3d> OTPList;



bool hrics_init_otp(std::string humanName)
{
    if (HRICS_MotionPLConfig == NULL)
    {
        cout << "No pointer to HRICS_MotionPLConfig" <<endl;
        return false;
    }
    else
    {
        return  dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->InitMhpObjectTransfert(humanName);
    }
}

bool hrics_compute_otp(std::string humanName, std::vector<std::vector<double> >& traj, configPt& handConf,bool isStanding, double objectNessecity)
{
    if (HRICS_MotionPLConfig == NULL)
    {
        cout << "No pointer to HRICS_MotionPLConfig" <<endl;
        return false;
    }
    else
    {
        return  dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getOtp(humanName,traj,handConf,isStanding,objectNessecity);
    }
}

void hrics_otp_fct()
{
#ifdef MIGHTABILITY_MAPS
    ext_hrics_init_otp= hrics_init_otp;
    ext_hrics_compute_otp= hrics_compute_otp;
#else
    cout<< "Flag MIGHTABILITY_MAPS off" << endl;
#endif    
}



OTPMotionPl::OTPMotionPl() : HumanAwareMotionPlanner() , m_PathExist(false) , m_HumanPathExist(false) , initTime(10)
{
#ifdef DEBUG_STATUS
    cout << "New OTPMotionPl HRI planner" << endl;
#endif
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
            m_Human = new Robot(XYZ_ENV->robot[i]);
            cout << "Humans is " << name << endl;
        }
    }

    initAll();
    m_OTPsuceed =false;
}

OTPMotionPl::OTPMotionPl(Robot* R, Robot* H) : HumanAwareMotionPlanner() , m_Human(H) , m_PathExist(false) , m_HumanPathExist(false), initTime(10)
{
    this->setRobot(R);
    initAll();
}

OTPMotionPl::~OTPMotionPl()
{
    delete m_2DGrid;
    delete m_pts;
    delete m_ConfGen;
}

void OTPMotionPl::initAll()
{

    std::vector<double> m_EnvSize;
    // initCostSpace
    m_EnvSize.resize(4);
    m_EnvSize[0] = XYZ_ENV->box.x1; m_EnvSize[1] = XYZ_ENV->box.x2;
    m_EnvSize[2] = XYZ_ENV->box.y1; m_EnvSize[3] = XYZ_ENV->box.y2;

    m_2DGrid = new EnvGrid(PlanEnv->getDouble(PlanParam::env_Cellsize),m_EnvSize,false,_Robot,m_Human);

    // initDistance
    vector<Robot*> m_Humans;
    m_Humans.push_back(m_Human);
    m_DistanceSpace = new Distance(_Robot,m_Humans);

    if (_Robot)
    {
        cout << "Robot " << _Robot->getName() << endl;
        cout << "Robot get struct " << _Robot->getP3dRobotStruct() << endl;
    }

    m_DistanceSpace->parseHumans();

    // initVisibility
    m_VisibilitySpace = new Visibility(m_Human);

    //initReachability
    setReachability(new Natural(m_Human));

    // init pos
    confPtr_t q_human_cur = m_Human->getCurrentPos();
    m_Human->setInitPos(*q_human_cur);

    // Init Manipulation Planner
    m_ManipPl = new ManipulationPlanner( _Robot->getP3dRobotStruct() );
    m_ManipPlHum = new ManipulationPlanner( m_Human->getP3dRobotStruct() );

    // Init plannar smoothing
    m_pts = new PlannarTrajectorySmoothing(_Robot);
    m_ConfGen = new ConfGenerator(_Robot,m_Human);

    // Init Vector for using Slices
    m_sliceVect[0] = 0;
    m_sliceVect[1] = 0;
    m_sliceVect[2] = numeric_limits<double>::max( );

    //Init the chair
    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);
        if(name.find("SIMPLECHAIR") != string::npos )
        {
            m_simpleChair = new Robot(XYZ_ENV->robot[i]);
        }
    }
    if (!m_simpleChair)
    {
        cout << "No simpleChair, human should not be sitting" << endl;
    }


}

void OTPMotionPl::draw2dPath()
{
    if( m_PathExist && m_2DPath.size() > 1)
    {
        //        cout << "Drawing 2D path" << endl;
        for(unsigned int i=0;i<m_2DPath.size()-1;i++)
        {
            glLineWidth(3.);
            g3d_drawOneLine(m_2DPath[i][0],      m_2DPath[i][1],      0.4,
                            m_2DPath[i+1][0],    m_2DPath[i+1][1],    0.4,
                            Blue, NULL);
            glLineWidth(1.);

            double colorvector[4];
            colorvector[0] = 0.0;       //red
            colorvector[1] = 0.0;       //green
            colorvector[2] = 1.0;       //blue
            colorvector[3] = 1;       //transparency

            //        glEnable(GL_BLEND);
            g3d_set_color(Any,colorvector);

            g3d_draw_solid_sphere(m_2DPath[i+1][0],
                                  m_2DPath[i+1][1],
                                  0.4,
                                  0.05, 20);
            //            cout << "\n\n cell: \n" << m_2DPath[i] << endl;
        }
    }

    if( m_HumanPathExist )
    {
        //        cout << "Drawing 2D path" << endl;
        for(unsigned int i=0;i<m_2DHumanPath.size()-1;i++)
        {
            glLineWidth(3.);
            g3d_drawOneLine(m_2DHumanPath[i][0],      m_2DHumanPath[i][1],      0.4,
                            m_2DHumanPath[i+1][0],    m_2DHumanPath[i+1][1],    0.4,
                            Green, NULL);
            glLineWidth(1.);

            double colorvector[4];
            colorvector[0] = 0.0;       //red
            colorvector[1] = 1.0;       //green
            colorvector[2] = 0.0;       //blue
            colorvector[3] = 1;       //transparency

            //        glEnable(GL_BLEND);
            g3d_set_color(Any,colorvector);

            g3d_draw_solid_sphere(m_2DHumanPath[i+1][0],
                                  m_2DHumanPath[i+1][1],
                                  0.4,
                                  0.05, 20);
        }
    }

    if (m_2DHumanRealPath.size()> 0)
    {
        for(unsigned int i=0;i<m_2DHumanRealPath.size()-1;i++)
        {

            if (m_2DHumanRealPath[i][0] != m_2DHumanRealPath[i+1][0] || m_2DHumanRealPath[i][1] != m_2DHumanRealPath[i+1][1])
            {
                //                cout << "to draw \n" << m_2DHumanRealPath[i] << endl;
                glLineWidth(3.);
                g3d_drawOneLine(m_2DHumanRealPath[i][0],      m_2DHumanRealPath[i][1],      0.4,
                                m_2DHumanRealPath[i+1][0],    m_2DHumanRealPath[i+1][1],    0.4,
                                Yellow, NULL);
                glLineWidth(1.);
            }
        }
    }
}

bool OTPMotionPl::ComputePR2Gik()
{
    cout << "OTPMotionPl::computePR2GIK()" << endl;

    configPt q;
    bool res = m_ConfGen->computeRobotIkForGrabing(q);
    if (res)
    {
        confPtr_t m_q = confPtr_t(
                new Configuration(_Robot,p3d_copy_config(_Robot->getP3dRobotStruct(),q)));
        _Robot->setAndUpdate( *m_q );
        cout << "gik computing: success" << endl;
    }
    return res;
}

void OTPMotionPl::initPR2GiveConf()
{
    if (PlanEnv->getBool(PlanParam::env_showText))
    {
        cout << "OTPMotionPl::initPR2GiveConf()" << endl;
    }


    confPtr_t q_cur = _Robot->getCurrentPos();

    configPt q;
    q = p3d_alloc_config(_Robot->getP3dRobotStruct());


    for (unsigned int i = 0; i < 48; i++)
    {
        q[i] = (*q_cur)[i];
    }

    //   q[12] = 0;
    q[13] = 0;
    q[14] = 0;
    q[15] = 0;
    q[17] = 37.00*M_PI/180;
    q[18] = -29.00*M_PI/180;
    q[19] = -133.00*M_PI/180;
    q[20] = 173.00*M_PI/180;
    q[21] = -75.00*M_PI/180;
    q[22] = 180.00*M_PI/180;
    q[23] = 0;
    q[24] = 0;
    q[25] = 0;
    q[26] = 80.00*M_PI/180;
    q[27] = 85.00*M_PI/180;
    q[28] = -90.00*M_PI/180;
    q[29] = 0;
    q[30] = 0;
    q[31] = 90.00*M_PI/180;
    q[32] = 0;
    q[33] = 0;
    q[34] = 0;
    q[35] = 0;
    q[36] = 0;



    confPtr_t m_q = confPtr_t(
            new Configuration(_Robot,p3d_copy_config(_Robot->getP3dRobotStruct(),q)));
    _Robot->setAndUpdate( *m_q );
    _Robot->setInitPos(*m_q );
}

ConfGenerator* OTPMotionPl::getConfGenerator()
{
    if (!m_ConfGen)
    {
        m_ConfGen = new ConfGenerator(_Robot,m_Human);
    }
    return m_ConfGen;
}

void OTPMotionPl::placeHuman()
{
    confPtr_t q_robot_cur = _Robot->getCurrentPos();
    _Robot->setAndUpdate(*_Robot->getInitPos());

    m_ReachableSpace->computeIsReachableAndMove(current_WSPoint,false);
    _Robot->setAndUpdate(*q_robot_cur);
}


int OTPMotionPl::loadConfsFromXML(string filename, bool isStanding, bool isSlice)
{
    if (isSlice)
    {
        if (isStanding)
        {
            m_configListSlice = getConfGenerator()->loadFromXml(filename);
            sortConfigList(m_configListSlice.size(),isStanding,isSlice);
            return m_configListSlice.size();
        }
        else
        {
            m_sittingConfigListSlice = getConfGenerator()->loadFromXml(filename);
            sortConfigList(m_sittingConfigListSlice.size(),isStanding,isSlice);
            return m_sittingConfigListSlice.size();
        }
    }
    else
    {
        if (isStanding)
        {
            m_configList = getConfGenerator()->loadFromXml(filename);
            sortConfigList(m_configList.size(),isStanding,isSlice);
            return m_configList.size();
        }
        else
        {
            m_sittingConfigList = getConfGenerator()->loadFromXml(filename);
            sortConfigList(m_sittingConfigList.size(),isStanding,isSlice);
            return m_sittingConfigList.size();
        }
    }
}

pair<confPtr_t,confPtr_t > OTPMotionPl::setRobotsToConf(int id, bool isStanding)
{
    pair<confPtr_t,confPtr_t > resConf;
    vector<ConfigHR> vectConfs;
    if (PlanEnv->getBool(PlanParam::env_useSlice) || PlanEnv->getBool(PlanParam::env_useOrientedSlice))
    {
        if (isStanding)
        {
            vectConfs = m_configListSlice;
        }
        else
        {
            vectConfs = m_sittingConfigListSlice;
        }
    }
    else
    {
        if (isStanding)
        {
            vectConfs = m_configList;
        }
        else
        {
            vectConfs = m_sittingConfigList;
        }
    }
    for (unsigned int i = 0; i< vectConfs.size(); i++)
    {
        if (vectConfs.at(i).getId() == id)
        {

            int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
            int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getP3dRobotStruct()->baseJnt)->user_dof_equiv_nbr;

            //			confPtr_t q_robot_cur = _Robot->getCurrentPos();
            confPtr_t q_human_cur = m_Human->getCurrentPos();

            confPtr_t ptrQ_human(new Configuration(m_Human,vectConfs.at(i).getHumanConf()));
            (*ptrQ_human)[firstIndexOfHumanDof + 0] = (*q_human_cur)[firstIndexOfHumanDof + 0];
            (*ptrQ_human)[firstIndexOfHumanDof + 1] = (*q_human_cur)[firstIndexOfHumanDof + 1];
            (*ptrQ_human)[firstIndexOfHumanDof + 5] = angle_limit_PI((*q_human_cur)[firstIndexOfHumanDof + 5]);
            m_Human->setAndUpdate(*ptrQ_human);
            resConf.first = ptrQ_human;

            confPtr_t ptrQ_robot(new Configuration(_Robot,vectConfs.at(i).getRobotConf()));
            (*ptrQ_robot)[firstIndexOfRobotDof + 5] = angle_limit_PI((*ptrQ_robot)[firstIndexOfRobotDof + 5] + (*q_human_cur)[firstIndexOfHumanDof + 5]);
            double dist = sqrt(pow( (*ptrQ_robot)[firstIndexOfRobotDof + 0], 2) + pow( (*ptrQ_robot)[firstIndexOfRobotDof + 1], 2));
            (*ptrQ_robot)[firstIndexOfRobotDof + 0] = -cos((*ptrQ_robot)[firstIndexOfRobotDof + 5])*dist + (*ptrQ_human)[firstIndexOfHumanDof + 0];
            (*ptrQ_robot)[firstIndexOfRobotDof + 1] = -sin((*ptrQ_robot)[firstIndexOfRobotDof + 5])*dist + (*ptrQ_human)[firstIndexOfHumanDof + 1];

            _Robot->setAndUpdate(*ptrQ_robot);
            resConf.second = ptrQ_robot;


            return resConf;
        }
    }
    return resConf;
}

pair<confPtr_t,confPtr_t > OTPMotionPl::setRobotsToConf(int id, bool isStanding ,double x, double y, double Rz)
{
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
    confPtr_t ptrQ_human = m_Human->getCurrentPos();
    (*ptrQ_human)[firstIndexOfHumanDof + 0] = x;
    (*ptrQ_human)[firstIndexOfHumanDof + 1] = y;
    (*ptrQ_human)[firstIndexOfHumanDof + 5] = angle_limit_PI(Rz);
    m_Human->setAndUpdate(*ptrQ_human);

    return setRobotsToConf(id, isStanding);
}

void OTPMotionPl::sortConfigList(double nbNode, bool isStanding, bool isSlice)
{
    if (isSlice)
    {
        if (isStanding)
        {
            getConfGenerator()->sortConfig(m_configListSlice,nbNode,isStanding,isSlice,m_ReachableSpace);
        }
        else
        {
            getConfGenerator()->sortConfig(m_sittingConfigListSlice,nbNode,isStanding,isSlice,m_ReachableSpace);
        }
    }
    else
    {
        if (isStanding)
        {
            getConfGenerator()->sortConfig(m_configList,nbNode,isStanding,isSlice,m_ReachableSpace);
        }
        else
        {
            getConfGenerator()->sortConfig(m_sittingConfigList,nbNode,isStanding,isSlice,m_ReachableSpace);
        }
    }

    /*
    confPtr_t q_robot_cur = _Robot->getCurrentPos();
    confPtr_t q_human_cur = m_Human->getCurrentPos();

    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getP3dRobotStruct()->baseJnt)->user_dof_equiv_nbr;
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();

    double reachFactor = ENV.getDouble(Env::Kreachable);
    double distFactor = ENV.getDouble(Env::Kdistance);
    double visFactor = ENV.getDouble(Env::Kvisibility);

    vector<ConfigHR> vectConfs;
    if (isSlice)
    {
        if (isStanding)
        {
            vectConfs = m_configListSlice;
        }
        else
        {
            vectConfs = m_sittingConfigListSlice;
        }
    }
    else
    {
        if (isStanding)
        {
            vectConfs = m_configList;
        }
        else
        {
            vectConfs = m_sittingConfigList;
        }
    }

    double maxDist = 0;
    for (int i = 0; i < nbNode; i++)
    {
        confPtr_t ptrQ_robot(new Configuration(_Robot,vectConfs.at(i).getRobotConf()));
        confPtr_t ptrQ_human(new Configuration(m_Human,vectConfs.at(i).getHumanConf()));
        double tmpDist = sqrt(pow((*ptrQ_robot)[firstIndexOfRobotDof + 0]-(*ptrQ_human)[firstIndexOfHumanDof + 0],2) +
                              pow((*ptrQ_robot)[firstIndexOfRobotDof + 1]-(*ptrQ_human)[firstIndexOfHumanDof + 1],2));
        if (tmpDist > maxDist)
        {
            maxDist = tmpDist;
        }
    }


    for (int i = 0; i < nbNode; i++)
    {
        confPtr_t ptrQ_robot(new Configuration(_Robot,vectConfs.at(i).getRobotConf()));
        _Robot->setAndUpdate(*ptrQ_robot);
        double xRob = (*ptrQ_robot)[firstIndexOfRobotDof + 0];
        double yRob = (*ptrQ_robot)[firstIndexOfRobotDof + 1];

        confPtr_t ptrQ_human(new Configuration(m_Human,vectConfs.at(i).getHumanConf()));
        m_Human->setAndUpdate(*ptrQ_human);
        double xHum = (*ptrQ_human)[firstIndexOfHumanDof + 0];
        double yHum = (*ptrQ_human)[firstIndexOfHumanDof + 1];
        double rzHum = angle_limit_PI((*ptrQ_human)[firstIndexOfHumanDof + 5]);

        double reachCost = m_ReachableSpace->getConfigCost() * reachFactor;
        double distCost = (1 - sqrt(pow(xRob-xHum,2) + pow(yRob-yHum,2))/maxDist) * distFactor;
        double visCost = rzHum - atan2(yRob-yHum,xRob-xHum);
        if (visCost < -M_PI)
        {
            visCost += 2 * M_PI;
        }
        visCost = fabs(visCost/M_PI) * visFactor;
        double cost = (reachCost + distCost + visCost) / (reachFactor + distFactor + visFactor);
        vectConfs.at(i).setCost(cost);
    }
    _Robot->setAndUpdate(*q_robot_cur);
    m_Human->setAndUpdate(*q_human_cur);

    ConfigurationCost ConfigurationCostCompObject;
    sort(vectConfs.begin(),vectConfs.end(),ConfigurationCostCompObject);

    for (int i = 0; i < nbNode; i++)
    {
        vectConfs.at(i).setId(i);
    }

    if (isSlice)
    {
        if (isStanding)
        {
            m_configListSlice = vectConfs;
        }
        else
        {
            m_sittingConfigListSlice = vectConfs;
        }
    }
    else
    {
        if (isStanding)
        {
            m_configList = vectConfs;
        }
        else
        {
            m_sittingConfigList = vectConfs;
        }
    }

*/

}


double OTPMotionPl::multipliComputeOtp(int n)
{
    cout << "-----------------------------" << endl;
    cout << "start computing" << endl;
    cout << "-----------------------------" << endl;
    PlanEnv->setBool(PlanParam::env_showText,false);
    double totalTimeSum = 0;
    double initGridTimeSum = 0;
    double firstTimeSum = 0;
    double loopTimeSum = 0;
    double costSum = 0;
    int nbIteration = 0;
    int nbSolution = 0;

    int nb = 0;
    int id = 0;
    m_time = -1;
    saveInitConf();
    std::vector<OutputConf> confVect;
    vector<double> timeVect;

    ofstream myfile;
    string fileName = "/statFiles/OtpComputing/configCosts.lst";
    string home = getenv("HOME_MOVE3D") + fileName;
    myfile.open (home.c_str());
    myfile << "";
    myfile.close();

    m_multipliComputeCostVector.clear();
    m_multipliComputeTimeVector.clear();
    for (int i = 0; i < n; i++)
    {
        m_multipleData.clear();
        newComputeOTP();
        string inf = "with";
        if(m_time <= 0)
        {
            inf = "without";
        }
        cout << "Computing of " << id++ << " done " << inf << " success" << endl;

        if (m_confList.size()> 0)
        {
            confVect.push_back(m_confList.at(0));
            if (m_time > 0)
            {
                timeVect.push_back(m_time);
                totalTimeSum += m_multipleData.at(0);
                initGridTimeSum += m_multipleData.at(1);
                firstTimeSum += m_multipleData.at(2);
                loopTimeSum += m_multipleData.at(3);
                costSum += m_multipleData.at(4);
                nbIteration += m_multipleData.at(5);
                nbSolution += m_multipleData.at(6);
                nb++;
            }

        }
        loadInitConf(true,true);
    }

    m_multipleData.clear();
    m_multipleData.push_back((double)totalTimeSum/ nb);
    m_multipleData.push_back((double)initGridTimeSum/ nb);
    m_multipleData.push_back((double)firstTimeSum/ nb);
    m_multipleData.push_back((double)loopTimeSum/ nb);
    m_multipleData.push_back((double)costSum/ nb);
    m_multipleData.push_back((double)nbIteration/ nb);
    m_multipleData.push_back((double)nbSolution/ nb);
    m_multipleData.push_back((double)nbSolution / nbIteration);
    m_multipleData.push_back((double)nb);


    saveAllCostsToFile();
    saveAlltimesToFile();
    m_multipliComputeCostVector.clear();
    m_multipliComputeTimeVector.clear();
    PlanEnv->setBool(PlanParam::env_showText,true);

    m_confList.clear();
    m_confList = confVect;
    cout << "-----------------------------" << endl;
    cout << "end computing" << endl;
    cout << "-----------------------------" << endl;
    for (unsigned int i = 0; i < timeVect.size(); i++)
    {
        cout << "Computing time for test nb " << i << " is : " << timeVect.at(i) << endl ;
    }
    double sumCost = 0;
    for (unsigned int i = 0; i < confVect.size(); i++)
    {
        OutputConf conf = confVect.at(i);
        sumCost += conf.cost;
    }
    cout << "-----------------------------" << endl;
    cout << "Nb of OTP computed = " << nb << endl;
    cout << "Average First conf computing time = " << firstTimeSum / nb << endl << endl;
    cout << "Average time = " << totalTimeSum / nb << endl;
    cout << "Average init Grid time = " << initGridTimeSum / nb << endl;
    cout << "Average loop time = " << loopTimeSum / nb << endl;
    cout << "Average cost = " << costSum / nb << endl;
    cout << "Average nb Iteration = " << (double)nbIteration / nb << endl;
    cout << "Average nb solutions = " << (double)nbSolution / nb << endl;
    cout << "Average rate = " << (double)nbSolution / nbIteration << endl;
    cout << "rate of successful OTP = " << (double)nb/id << endl;
    cout << "Formated :" << endl << totalTimeSum / nb << endl << initGridTimeSum / nb << endl << loopTimeSum / nb << endl
            << costSum / nb << endl << (double)nbIteration / nb << endl << (double)nbSolution / nb << endl << (double)nbSolution / nbIteration <<
            endl << (double)nb/id << endl;
    cout << "-----------------------------" << endl;

    return totalTimeSum / nb;


}




void OTPMotionPl::setInputs( Eigen::Vector3d humanPos, Eigen::Vector3d robotPos,bool isStanding, double mobility)
{
    std::vector<double> humPos;
    humPos.push_back(humanPos[0]);
    humPos.push_back(humanPos[1]);
    humPos.push_back(humanPos[2]);
    m_humanPos = humPos;
    m_robotPos = robotPos;
    isStanding = m_isStanding;
    mobility = m_mobility;
}



void OTPMotionPl::getInputs()
{
    confPtr_t q_human_cur = m_Human->getCurrentPos();
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
    if (PlanEnv->getBool(PlanParam::env_realTime))
    {
        Robot* visball = global_Project->getActiveScene()->getRobotByNameContaining("VISBALL");
        (*q_human_cur)[firstIndexOfHumanDof + 0] = (*visball->getCurrentPos())[6];
        (*q_human_cur)[firstIndexOfHumanDof + 1] = (*visball->getCurrentPos())[7];
        (*q_human_cur)[firstIndexOfHumanDof + 5] = angle_limit_PI((*visball->getCurrentPos())[11]);
    }


    m_humanPos.clear();
    m_humanPos.push_back((*q_human_cur)[firstIndexOfHumanDof + 0]);
    m_humanPos.push_back((*q_human_cur)[firstIndexOfHumanDof + 1]);
    m_humanPos.push_back(angle_limit_PI((*q_human_cur)[firstIndexOfHumanDof + 5]));

    confPtr_t q_robot_cur = _Robot->getCurrentPos();
    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getP3dRobotStruct()->baseJnt)->user_dof_equiv_nbr;
    //    cout << " in getInputs: firstIndexOfRobotDof = " << firstIndexOfRobotDof << endl;
    //    q_robot_cur->print();
    m_robotPos[0] = (*q_robot_cur)[firstIndexOfRobotDof + 0];
    m_robotPos[1] = (*q_robot_cur)[firstIndexOfRobotDof + 1];
    double trt = angle_limit_PI((*q_robot_cur)[firstIndexOfRobotDof + 5]);

    m_robotPos[2] = trt;


    m_isStanding = PlanEnv->getBool(PlanParam::env_isStanding);
    m_mobility = PlanEnv->getDouble(PlanParam::env_objectNessecity);
}

void OTPMotionPl::setRobotPos()
{
    confPtr_t q_robot_cur = _Robot->getCurrentPos();
    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getP3dRobotStruct()->baseJnt)->user_dof_equiv_nbr;
    m_robotPos[0] = (*q_robot_cur)[firstIndexOfRobotDof + 0];
    m_robotPos[1] = (*q_robot_cur)[firstIndexOfRobotDof + 1];
    m_robotPos[2] = angle_limit_PI((*q_robot_cur)[firstIndexOfRobotDof + 5]);
}

bool OTPMotionPl::isTheRealNearThePredicted(double threshold)
{

    if (m_2DHumanRealPath.size()> 0)
    {
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > tmp;
        tmp.push_back(m_2DHumanRealPath.at(0));

        for(unsigned int i=0;i<m_2DHumanRealPath.size()-1;i++)
        {
            if (m_2DHumanRealPath[i][0] != m_2DHumanRealPath[i+1][0] || m_2DHumanRealPath[i][1] != m_2DHumanRealPath[i+1][1])
            {
                tmp.push_back(m_2DHumanRealPath.at(i+1));
                //                cout << m_2DHumanRealPath.at(i+1)[0] << "  " << m_2DHumanRealPath.at(i+1)[1]<< endl;
            }
        }
        m_2DHumanRealPath = tmp;
    }

    //    if (m_2DHumanPath.size()> 0)
    //    {
    //        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > tmp;
    //        tmp.push_back(m_2DHumanPath.at(0));
    //
    //        for(unsigned int i=0;i<m_2DHumanPath.size()-1;i++)
    //        {
    //            if (m_2DHumanPath[i][0] != m_2DHumanPath[i+1][0] || m_2DHumanPath[i][1] != m_2DHumanPath[i+1][1])
    //            {
    //                tmp.push_back(m_2DHumanPath.at(i+1));
    ////                cout << m_2DHumanRealPath.at(i+1)[0] << "  " << m_2DHumanRealPath.at(i+1)[1]<< endl;
    //            }
    //        }
    //        m_2DHumanPath = tmp;
    //    }

    //    if (m_2DHumanRealPath.size()> 0 && m_HumanPathExist )
    //    {
    //        cout <<  "real path" << endl;
    //
    //        for (unsigned int i=0;i<m_2DHumanRealPath.size();i++)
    //        {
    //            cout << m_2DHumanRealPath.at(i)[0] << "  " << m_2DHumanRealPath.at(i)[1]<< endl;
    //        }
    //
    //        cout <<  "predicted path" << endl;
    //        for (unsigned int i=0;i<m_2DHumanPath.size();i++)
    //        {
    //            cout << m_2DHumanPath.at(i)[0] << "  " << m_2DHumanPath.at(i)[1]<< endl;
    //        }
    //    }

    if( !m_HumanPathExist )
    {
        Eigen::Vector2d tmp(m_humanPos[0],m_humanPos[1]);
        m_2DHumanPath.clear();
        m_2DHumanPath.push_back(tmp);
    }
    if (m_2DHumanRealPath.size() == 0)
    {
        Eigen::Vector2d tmp(m_humanPos[0],m_humanPos[1]);
        m_2DHumanRealPath.clear();
        m_2DHumanRealPath.push_back(tmp);
    }

    if(m_2DHumanRealPath.size() > 1)
    {
        bool tmp = false;
        for(unsigned int j=0;j<m_2DHumanRealPath.size();j++)
        {
            tmp=false;
            for(unsigned int i=0;i<m_2DHumanPath.size()-1;i++)
            {
                double h = ComputePlanarDistancesLineToSegment(m_2DHumanRealPath.at(j), m_2DHumanPath.at(i), m_2DHumanPath.at(i+1));

                if (h <= threshold )
                {
                    tmp=true;
                    break;
                }
            }
            if (!tmp)
            {
                return false;
            }
        }
        return true;
    }
    else
    {
        for(unsigned int i=0;i<m_2DHumanPath.size();i++)
        {
            if (sqrt(pow(m_2DHumanRealPath.at(0)[0] - m_2DHumanPath.at(i)[0],2) + pow(m_2DHumanRealPath.at(0)[1] - m_2DHumanPath.at(i)[1],2)) < threshold)
            {
                return true;
            }
        }
        return false;
    }

}

double OTPMotionPl::ComputePlanarDistancesLineToSegment(Eigen::Vector2d p, Eigen::Vector2d p1, Eigen::Vector2d p2)
{

    //    cout << "\n\np \n" << p << "\np1 \n" << p1<< "\np2 \n" <<p2 << endl;

    double a = sqrt(pow(p[0] - p1[0],2) + pow(p[1] - p1[1],2));
    double b = sqrt(pow(p[0] - p2[0],2) + pow(p[1] - p2[1],2));
    double dist = sqrt(pow(p1[0] - p2[0],2) + pow(p1[1] - p2[1],2));

    //    cout << "a = " << a << endl;
    //    cout << "b = " << b << endl;
    //    cout << "dist = " << dist << endl;

    double h = numeric_limits<double>::max( );
    if (p1[0] == p2[0] && p1[1] == p2[1])
    {
        return a;
    }
    else if (p1[0] == p2[0])
    {
        h = fabs(p1[0]-p[0]);
    }
    else if (p1[1] == p2[1])
    {
        h = fabs(p1[1]-p[1]);
    }
    else
    {
        double a = (p2[1] - p1[1])/(p2[0] - p1[0]);
        double b = p1[1] - p1[0]* a;
        h = fabs(a*p[0] - p[1] + b)/sqrt(1 + a*a);
    }


    double hypo = sqrt(h*h + dist*dist);
    if (a <= hypo && b <= hypo)
    {
        return h;
    }
    else if (a < b)
    {
        return a;
    }
    else if (a >= b)
    {
        return b;
    }
    return numeric_limits<double>::max( );
}


bool OTPMotionPl::getRandomPoints(double id, Vector3d& vect)
{
    //    Vector3d vect;

    if (PlanEnv->getBool(PlanParam::env_normalRand))
    {
        confPtr_t q_human_cur = m_Human->getCurrentPos();
        int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();

        double x = (*q_human_cur)[firstIndexOfHumanDof + 0];
        double y = (*q_human_cur)[firstIndexOfHumanDof + 1];
        //        double Rz = (*q_human_cur)[firstIndexOfHumanDof + 5];

        double randomXMinLimit = PlanEnv->getDouble(PlanParam::env_randomXMinLimit);//-3.0
        double randomXMaxLimit = PlanEnv->getDouble(PlanParam::env_randomXMaxLimit);//3.0
        double randomYMinLimit = PlanEnv->getDouble(PlanParam::env_randomYMinLimit);//-3.0
        double randomYMaxLimit = PlanEnv->getDouble(PlanParam::env_randomYMaxLimit);//3.0
        int nbRandomRotOnly = PlanEnv->getInt(PlanParam::env_nbRandomRotOnly);//10

        vect[0] = p3d_random(x + randomXMinLimit, x + randomXMaxLimit);
        vect[1] = p3d_random(y + randomYMinLimit, y + randomYMaxLimit);
        if (id < nbRandomRotOnly)
        {
            vect[0] = x;
            vect[1] = y;
        }
        vect[2] = p3d_random(-M_PI,M_PI);
    }
    else if (PlanEnv->getBool(PlanParam::env_useAllGrid))
    {
        confPtr_t q_human_cur = m_Human->getCurrentPos();
        int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
        int nbRandomRotOnly = PlanEnv->getInt(PlanParam::env_nbRandomRotOnly);//10
        double x = (*q_human_cur)[firstIndexOfHumanDof + 0];
        double y = (*q_human_cur)[firstIndexOfHumanDof + 1];

        std::vector<EnvCell*> sortedCell = m_2DGrid->getHumanAccessibleCells();
        double rand = p3d_random(0,1);
        //        rand = pow(rand,PlanEnv->getInt(PlanParam::env_pow)); // giving a higher probability of getting small numbers
        int cellNb = floor(rand*(sortedCell.size()-1));
        Vector2d center = sortedCell.at(cellNb)->getCenter();
        Vector2d cellSize = sortedCell.at(cellNb)->getCellSize();
        vect[0] = p3d_random(center[0] - cellSize[0]/2, center[0] + cellSize[0]/2);
        vect[1] = p3d_random(center[1] - cellSize[1]/2, center[1] + cellSize[1]/2);
        if (id < nbRandomRotOnly)
        {
            vect[0] = x;
            vect[1] = y;
        }
        vect[2] = p3d_random(-M_PI,M_PI);

    }
    else if (PlanEnv->getBool(PlanParam::env_fusedGridRand))
    {
        std::vector<std::pair<double,EnvCell*> > sortedCell = m_2DGrid->getSortedGrid();
        if (sortedCell.empty())
        {
            cout << "the biased grid is empty." << endl ;
            vect[0] = 0;
            vect[1] = 0;
            vect[2] = 0;
            return false;
        }
        else
        {
            double rand = p3d_random(0,1);
            rand = pow(rand,PlanEnv->getInt(PlanParam::env_pow)); // giving a higher probability of getting small numbers
            int cellNb = floor(rand*(sortedCell.size()-1));
            Vector2d center = sortedCell.at(cellNb).second->getCenter();
            Vector2d cellSize = sortedCell.at(cellNb).second->getCellSize();
            vect[0] = p3d_random(center[0] - cellSize[0]/2, center[0] + cellSize[0]/2);
            vect[1] = p3d_random(center[1] - cellSize[1]/2, center[1] + cellSize[1]/2);
            vect[2] = p3d_random(-M_PI,M_PI);
        }

    }
    else if (PlanEnv->getBool(PlanParam::env_fusedGridAndRotRand))
    {
        std::vector<std::pair<double,EnvCell*> > sortedCell = m_2DGrid->getSortedGrid();
        if (sortedCell.empty())
        {
            cout << "the biased grid is empty."<< endl ;
            vect[0] = 0;
            vect[1] = 0;
            vect[2] = 0;
            return false;
        }
        else
        {

            double rand = p3d_random(0,1);
            rand = pow(rand,PlanEnv->getInt(PlanParam::env_pow)); // giving a higher probability of getting small numbers
            int cellNb = floor(rand*(sortedCell.size()-1));
            EnvCell* cell = sortedCell.at(cellNb).second;
            Vector2d center = cell->getCenter();
            EnvCell* robotCell = cell->getRobotBestPos().second;
            Vector2d robotCellCenter = robotCell->getCenter();
            double angle = atan2(robotCellCenter[1] - center[1],robotCellCenter[0] - center[0]);
            int sign = p3d_random_integer(0,1);
            if (sign == 0){sign = -1;}
            double randAngle = pow(p3d_random(0,1),PlanEnv->getInt(PlanParam::env_pow))*sign*M_PI;
            //            double randAngle = pow(p3d_random(-1,1),PlanEnv->getInt(PlanParam::env_pow)*2+1)*M_PI;
            randAngle=  angle_limit_PI(randAngle + angle);


            Vector2d cellSize = sortedCell.at(cellNb).second->getCellSize();
            vect[0] = p3d_random(center[0] - cellSize[0]/2, center[0] + cellSize[0]/2);
            vect[1] = p3d_random(center[1] - cellSize[1]/2, center[1] + cellSize[1]/2);
            vect[2] = randAngle;
        }

    }
    else if (PlanEnv->getBool(PlanParam::env_useOldDude))
    {
        std::vector<std::pair<double,EnvCell*> > sortedCell = m_2DGrid->getSortedGrid();
        if (sortedCell.empty())
        {
            cout << "the biased grid is empty." << endl ;
            vect[0] = 0;
            vect[1] = 0;
            vect[2] = 0;

            return false;
        }
        else
        {
            double rand = p3d_random(0,1);
            //            rand = pow(rand,PlanEnv->getInt(PlanParam::env_pow)); // giving a higher probability of getting small numbers
            int cellNb = floor(rand*(sortedCell.size()-1));
            Vector2d center = sortedCell.at(cellNb).second->getCenter();
            Vector2d cellSize = sortedCell.at(cellNb).second->getCellSize();
            vect[0] = p3d_random(center[0] - cellSize[0]/2, center[0] + cellSize[0]/2);
            vect[1] = p3d_random(center[1] - cellSize[1]/2, center[1] + cellSize[1]/2);
            vect[2] = p3d_random(-M_PI,M_PI);
        }

    }
    else if (PlanEnv->getBool(PlanParam::env_useSlice))
    {
        int nbRandomRotOnly = PlanEnv->getInt(PlanParam::env_nbRandomRotOnly);
        if (m_sliceVect[2] < nbRandomRotOnly)
        {
            vect[0] = m_sliceVect[0];
            vect[1] = m_sliceVect[1];
            m_sliceVect[2] = m_sliceVect[2] + 1;
        }
        else
        {
            std::vector<std::pair<double,EnvCell*> > sortedCell = m_2DGrid->getSortedGrid();
            double rand = p3d_random(0,1);
            rand = pow(rand,PlanEnv->getInt(PlanParam::env_pow)); // giving a higher probability of getting small numbers
            int cellNb = floor(rand*(sortedCell.size()-1));
            Vector2d center = sortedCell.at(cellNb).second->getCenter();
            Vector2d cellSize = sortedCell.at(cellNb).second->getCellSize();
            vect[0] = p3d_random(center[0] - cellSize[0]/2, center[0] + cellSize[0]/2);
            vect[1] = p3d_random(center[1] - cellSize[1]/2, center[1] + cellSize[1]/2);
            m_sliceVect[0] = vect[0];
            m_sliceVect[1] = vect[1];
            m_sliceVect[2] = 0;
        }
        vect[2] = p3d_random(-M_PI,M_PI);
    }
    else if (PlanEnv->getBool(PlanParam::env_useOrientedSlice))
    {
        int nbRandomRotOnly = PlanEnv->getInt(PlanParam::env_nbRandomRotOnly);
        EnvCell* cell;
        if (m_sliceVect[2] < nbRandomRotOnly)
        {
            vect[0] = m_sliceVect[0];
            vect[1] = m_sliceVect[1];
            Vector2d pos;
            pos[0]  = vect[0];
            pos[1]  = vect[1];
            cell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(pos));
            m_sliceVect[2] = m_sliceVect[2] + 1;
        }
        else
        {
            std::vector<std::pair<double,EnvCell*> > sortedCell = m_2DGrid->getSortedGrid();
            double rand = p3d_random(0,1);
            rand = pow(rand,PlanEnv->getInt(PlanParam::env_pow)); // giving a higher probability of getting small numbers
            int cellNb = floor(rand*(sortedCell.size()-1));
            cell = sortedCell.at(cellNb).second;
            Vector2d center = cell->getCenter();
            Vector2d cellSize = cell->getCellSize();
            vect[0] = p3d_random(center[0] - cellSize[0]/2, center[0] + cellSize[0]/2);
            vect[1] = p3d_random(center[1] - cellSize[1]/2, center[1] + cellSize[1]/2);
            m_sliceVect[0] = vect[0];
            m_sliceVect[1] = vect[1];
            m_sliceVect[2] = 0;
        }
        //        cout << "----------------" << endl;
        if (cell->getAngleForHumanComming() < numeric_limits<double>::max())
        {
            double angle = PlanEnv->getDouble(PlanParam::env_limitRot);
            vect[2] = p3d_random(cell->getAngleForHumanComming() - angle ,cell->getAngleForHumanComming() + angle);
            //            cout << "comming angle : " << cell->getAngleForHumanComming()*180/M_PI << endl;
        }
        else
        {
            vect[2] = p3d_random(- M_PI , M_PI);
            //            cout << "comming angle : None " << endl;
        }
        if (vect[2] > M_PI)
        {
            vect[2] = vect[2] - M_PI;
        }
        else if (vect[2] < -M_PI)
        {
            vect[2] = vect[2] + M_PI;
        }
        //        cout << "random point : x = " << cell->getCoord()[0] << " y = " << cell->getCoord()[1] << " Rz = " << vect[2]*180/M_PI<< endl;
        //        cout << "----------------" << endl;
    }
    else
    {
        cout << "ERROR in random" << endl ;
        vect[0] = 0;
        vect[1] = 0;
        vect[2] = 0;
        return false;
    }

    return true;
}

bool OTPMotionPl::newComputeOTP()
{
    m_OTPsuceed = false;
    _Robot->setInitPos(*_Robot->getCurrentPos());
    clock_t start = clock();
    bool isStanding = PlanEnv->getBool(PlanParam::env_isStanding);
    if (!PlanEnv->getBool(PlanParam::env_realTime))
    {
        getInputs();
    }
    Eigen::Vector3d humPos;
    if (!PlanEnv->getBool(PlanParam::env_normalRand) && ! PlanEnv->getBool(PlanParam::env_useAllGrid))
    {
        //        Eigen::Vector3d humPos;
        humPos[0] = m_humanPos[0];
        humPos[1] = m_humanPos[1];
        humPos[2] = m_humanPos[2];
        if (!PlanEnv->getBool(PlanParam::env_realTime) && isStanding)
        {
            m_2DGrid->initGrid(humPos);
        }
        m_2DGrid->recomputeGridWhenHumanMove(humPos);
    }
    //    m_2DGrid->setCellsToblankCost();
    m_costVector.clear();
    m_timeVector.clear();
    clock_t gridInit = clock();

    //bool isStanding = m_isStanding;
    //double objectNecessity = m_mobility;

    double objectNecessity = PlanEnv->getDouble(PlanParam::env_objectNessecity);
    bool m_showText = PlanEnv->getBool(PlanParam::env_showText);
    double timeLimitation = PlanEnv->getDouble(PlanParam::env_timeLimitation);
    double dumpTime = PlanEnv->getDouble(PlanParam::env_timeToDump);
    double currntDumpTime = 0;
    double epsilon = EPS3;
//    bool noRepetition = PlanEnv->getBool(PlanParam::env_noRepetition);
    bool oldCriteria = PlanEnv->getBool(PlanParam::env_oldCriteria);

    int maxIter = PlanEnv->getInt(PlanParam::env_maxIter); //300

    clearCostsfile();
    if (m_configList.empty() || m_sittingConfigList.empty())
    {
        cout << "No configuration lists" << endl;
        return false;
    }
    confPtr_t q_human_cur = m_Human->getCurrentPos();
    confPtr_t q_robot_cur = _Robot->getCurrentPos();
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();


    double x = m_humanPos[0];
    double y = m_humanPos[1];
    double Rz = m_humanPos[2];

    if (m_showText)
    {
        cout << "---------------------------------------------------" << endl;
        cout << "start OTP search" << endl <<"---------------------------------------------------" << endl;
    }
    OutputConf bestConf;


    m_confList.clear();

    clock_t varInit = clock();
    if (isStanding)
    {
        //        if (!PlanEnv->getBool(PlanParam::env_realTime))
        //        {
        //            m_2DGrid->initGrid(humPos);
        //        }

        bestConf = lookForBestLocalConf(x,y,Rz,objectNecessity);
        if (m_simpleChair)
        {
            bestConf.chairConf = m_simpleChair->getCurrentPos();
        }

        Vector2d v;
        v[0] = x;
        v[1] = y;

        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > vect;
        vect.clear();
        vect.push_back(v);
        vect.push_back(v);
        bestConf.humanTraj = vect;
        bestConf.humanTrajExist = true;

        m_confList.push_back(bestConf);
        saveCostsTofile(bestConf.cost,bestConf.cost);
        m_isInitSiting = false;
    }
    else
    {
        bestConf = findBestPosForHumanSitConf(objectNecessity);
        m_confList.push_back(bestConf);
        saveCostsTofile(bestConf.cost,bestConf.cost);
        m_isInitSiting = true;
    }


    int i = 0;
    int beginId = m_confList.size();
    int id = beginId;
    bestConf.id = id;

    m_Human->setAndUpdate(*q_human_cur);
    _Robot->setAndUpdate(*q_robot_cur);

    bool finished = true;
    if (!m_humanCanStand && m_isInitSiting)
    {
        finished = false;
    }

    m_testedVectors.clear();

    clock_t firstConfs = clock();
    while (finished)
    {

        //        cout << "---------------------------------------------------" << endl;
        //        cout << "new section, init pos : x = " << x << " y = " << y << " Rz = " << Rz << endl;
        Vector3d vect;
        bool gridIsEmpty = getRandomPoints(id,vect);
        if (!gridIsEmpty)
        {
            break;
        }

        // STOPING CRITERIA
        clock_t curTime1 = clock();
        if (oldCriteria)
        {
            if (i > maxIter + beginId || id > PlanEnv->getInt(PlanParam::env_totMaxIter) + beginId) { break; }
            if (((double)curTime1 - firstConfs) / CLOCKS_PER_SEC > timeLimitation /20) { break;}
            //        cout << "i = " << i << " rate = " <<  i/(double)id << " time = " << ((double)curTime1 - firstConfs) / CLOCKS_PER_SEC << endl;
        }
        else
        {
            if ( i > maxIter + beginId  || // number of results are sufisiant
                 ((double)curTime1 - firstConfs) / CLOCKS_PER_SEC > timeLimitation ||  // taking too much time
                 id > PlanEnv->getInt(PlanParam::env_totMaxIter) ||
                 (i > 0 && (((double)curTime1 - firstConfs) / CLOCKS_PER_SEC) > (timeLimitation /20)) // taking too much time after finding some solution
                 ){
                if (m_showText){
                    if (i > maxIter + beginId) {cout << "number of results are sufisiant" <<endl; }
                    if (((double)curTime1 - firstConfs) / CLOCKS_PER_SEC > timeLimitation) {cout << "taking too much time" <<endl; }
                    if (i > 0 && ((double)curTime1 - firstConfs) / CLOCKS_PER_SEC > timeLimitation /20)
                    {cout << "taking too much time after finding some solution" <<endl; }
                }
                break;
            }
        }

//        if (isAlreadyTested(vect) && noRepetition)
//        {
//            continue;
//        }
        double randomX = vect[0];
        double randomY = vect[1];
        double randomRz = vect[2];

        id++;
        //        cout << "random : x = " << randomX << " y = " << randomY << " Rz = " << randomRz << endl;

        OutputConf tmpConf = lookForBestLocalConf(randomX,randomY,randomRz,objectNecessity);
        tmpConf.id = id;
        //        cout << "Cost : "<< tmpConf.cost << endl;
        if (m_simpleChair)
        {
            tmpConf.chairConf = m_simpleChair->getCurrentPos();
        }

        Vector2d v;
        v[0] = x;
        v[1] = y;

        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > vectPath = tmpConf.humanTraj;
        vectPath.insert(vectPath.begin(),v);
        //        vectPath.insert(vectPath.begin(),v);
        //        vectPath.insert(vectPath.begin(),v);
        tmpConf.humanTraj = vectPath;
        tmpConf.humanTrajExist = true;

        if (tmpConf.cost < numeric_limits<double>::max( ))
        {
            m_confList.push_back(tmpConf);
            //            saveCostsTofile(bestConf.cost,tmpConf.cost);

        }
        //        saveCostsTofile(bestConf.cost,tmpConf.cost);

        if (tmpConf.cost < bestConf.cost - epsilon)
        {
            bestConf = tmpConf;
            i = 0;

        }
        else if (tmpConf.cost < bestConf.cost && tmpConf.cost > bestConf.cost - epsilon )
        {
            bestConf = tmpConf;
            i++;
        }
        else if (tmpConf.cost < numeric_limits<double>::max( )){ i++; }

        //        cout << "current id = "<< id << endl;
        //        cout << "current iteration worst than the best one : " << i << endl;


        int time = PlanEnv->getInt(PlanParam::env_timeShow)*1000;
        if (PlanEnv->getBool(PlanParam::env_drawOnlyBest) && (time > 0))
        {
            if (bestConf.cost < numeric_limits<double>::max( ))\
            {
                double xf = (*bestConf.humanConf)[firstIndexOfHumanDof + 0];
                double yf = (*bestConf.humanConf)[firstIndexOfHumanDof + 1];
                double Rzf = angle_limit_PI((*bestConf.humanConf)[firstIndexOfHumanDof + 5]);
                setRobotsToConf(bestConf.configNumberInList,bestConf.isStandingInThisConf,xf,yf,Rzf);
                g3d_draw_allwin_active();
                usleep(time);
            }
        }


        m_Human->setAndUpdate(*q_human_cur);
        _Robot->setAndUpdate(*q_robot_cur);
        cout.clear(ios_base::goodbit);
        clock_t curTime = clock();



        if (((double)curTime - firstConfs) / CLOCKS_PER_SEC > dumpTime+currntDumpTime)
        {
            m_costVector.push_back(bestConf.cost);
            currntDumpTime += dumpTime;
            m_timeVector.push_back(((double)curTime - firstConfs) / (CLOCKS_PER_SEC /1000));
        }
    }

    clock_t endLoop = clock();
    if (bestConf.cost >= 10)
    {
        if(m_showText)
        {
            cout << "No transfer configuration for this initials positions" << endl;
            cout << "Total time = " << ((double)endLoop - start) / CLOCKS_PER_SEC << " s"<< endl;
        }

        m_time = -1;
        _Robot->setAndUpdate(*q_robot_cur);
        m_Human->setAndUpdate(*q_human_cur);
        return false;
    }
    // this can be reused in order to get more results
    //    for (unsigned int i = 0; i < m_costVector.size(); i++)
    //    {
    //        saveCostsTofile(m_costVector.at(i),0);
    //    }

    if (!m_showText)
    {
        m_multipliComputeCostVector.push_back(m_costVector);
        m_multipliComputeTimeVector.push_back(m_timeVector);
    }
    else
    {
        cout << "--------------------------------------" << endl << "End of OTP search" << endl << "--------------------------------------" << endl;

    }

    OutputConfSort OutputConfSortObj;
    sort(m_confList.begin(),m_confList.end(),OutputConfSortObj);

    bool trajFound = false;
    while (!trajFound)
    {

        OutputConf o = m_confList.at(0);
//        m_confList.erase(m_confList.begin());
//        o.humanTraj = m_pts->smoothTrajectory(m_Human,o.humanTraj);
//        o.robotTraj = m_pts->smoothTrajectory(_Robot,o.robotTraj);
//        m_confList.insert(m_confList.begin(),o);

        // trajectory

        m_2DHumanPath.clear();
        m_2DPath.clear();

        m_2DHumanPath = o.humanTraj;
        m_HumanPathExist = o.humanTrajExist;

        m_2DPath = o.robotTraj;
        m_PathExist = o.robotTrajExist;


        //    double xf = (*bestConf.humanConf)[firstIndexOfHumanDof + 0];
        //    double yf = (*bestConf.humanConf)[firstIndexOfHumanDof + 1];
        //    double Rzf = (*bestConf.humanConf)[firstIndexOfHumanDof + 5];
        //    setRobotsToConf(bestConf.configNumberInList,bestConf.isStandingInThisConf,xf,yf,Rzf);



//        cout << "time between and loop and before createTraj " << ((double)clock() - endLoop) / CLOCKS_PER_SEC << endl;
        if (PlanEnv->getBool(PlanParam::env_createTrajs))
        {
            if (!createTrajectoryFromOutputConf(o))
            {
                cout << "Computing of trajectory has failed, triing next issue" << endl;
                m_confList.erase(m_confList.begin());

                if (m_confList.size() <= 0)
                {
                    cout << "No trajectory found" << endl;
                    return false;
                }

            }
            else
            {
                trajFound = true;
            }
        }
        else
        {
            break;
        }
    }
//    PlanEnv->setBool(PlanParam::env_drawFinalConf,true);

    clock_t end = clock();

    m_time = ((double)end - start) / CLOCKS_PER_SEC;
    double tInitGrid = ((double)gridInit - start) / CLOCKS_PER_SEC;
    double tFirst = ((double)varInit - gridInit) / CLOCKS_PER_SEC;
    double tLoop = ((double)endLoop - firstConfs) / CLOCKS_PER_SEC;
    m_multipleData.clear();
    m_multipleData.push_back(m_time);
    m_multipleData.push_back(tInitGrid);
    m_multipleData.push_back(tFirst);
    if (isStanding)
    {
        m_multipleData.push_back(tLoop);
    }
    else
    {
        m_multipleData.push_back(tLoop+m_sittingTime);
    }
    m_multipleData.push_back(bestConf.cost);
    m_multipleData.push_back(id);
    m_multipleData.push_back(m_confList.size());


    if (m_showText)
    {
        cout.clear(ios_base::goodbit);
        cout.flush();
        cout << "==========" << endl;
        cout << "Number of tested configuration = " << id << endl;
        cout << "Number of solutions found = " << m_confList.size() << endl;
        cout << "Success rate = " << (double)m_confList.size() / (double)id << endl;
        cout << "the choosed configuration cost is : " << bestConf.cost << endl;
        cout << "----------" << endl <<"time elapsed to : " << endl;
        cout << "init grid = " << tInitGrid << " s"<< endl;
        cout << "init variable = " << tFirst << " s"<< endl;
        cout << "look for the first conf = " << tFirst << " s"<< endl;
        cout << "pass the loop = " << ((double)endLoop - firstConfs) / CLOCKS_PER_SEC << " s"<< endl;
        cout << "computing trajectory = " << ((double)end - endLoop) / CLOCKS_PER_SEC << " s"<< endl;
        cout << "----------" << endl << "Total time = " << m_time << " s"<< endl;
        cout << "==========" << endl;
    }
    //    showBestConf();
    m_costVector.clear();
    m_OTPsuceed = true;
    return m_OTPsuceed;
}

OutputConf OTPMotionPl::lookForBestLocalConf(double x, double y, double Rz, double objectNecessity)
{
    OutputConf bestLocalConf;
    bestLocalConf.isStandingInThisConf = true;

    if (x < XYZ_ENV->box.x1 ||  x > XYZ_ENV->box.x2 || y < XYZ_ENV->box.y1 || y > XYZ_ENV->box.y2)
    {
        //        cout << "human out of bound : x = " << x << " y = " << y << " Rz = " << Rz << endl;
        bestLocalConf.clearAll();
        return bestLocalConf;
    }

    ENV.getDouble(Env::Kdistance);
    double robotSpeed = PlanEnv->getDouble(PlanParam::env_robotSpeed);//1;
    double humanSpeed = PlanEnv->getDouble(PlanParam::env_humanSpeed);//1;
    double timeStamp = PlanEnv->getDouble(PlanParam::env_timeStamp);//0.1;
    double psi = PlanEnv->getDouble(PlanParam::env_psi);//0.99;
    double delta = PlanEnv->getDouble(PlanParam::env_delta);//0.01;

    double ksi = PlanEnv->getDouble(PlanParam::env_ksi);//0.5;
    double rho = PlanEnv->getDouble(PlanParam::env_rho);//0.5;

    double sittingOffset = PlanEnv->getDouble(PlanParam::env_sittingOffset);//0.2;

    confPtr_t q_robot_cur = _Robot->getCurrentPos();
    confPtr_t q_human_cur = m_Human->getCurrentPos();

    Vector2d robCurrentPos;
    robCurrentPos[0] = (*m_savedConf.robotConf)[6];
    robCurrentPos[1] = (*m_savedConf.robotConf)[7];


    double configCost = -1;
    vector<ConfigHR> vectConfs;
    if (PlanEnv->getBool(PlanParam::env_useSlice) || PlanEnv->getBool(PlanParam::env_useOrientedSlice))
    {
        vectConfs = m_configListSlice;
    }
    else
    {
        vectConfs = m_configList;
    }

    if (vectConfs.empty())
    {
        cout << "ERROR: no configuration loaded" << endl;
        return bestLocalConf;
    }

    Vector2d goalPos;
    goalPos[0] = x;
    goalPos[1] = y;

    EnvCell* cell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(goalPos));
    if (!cell)
    {
        //      cout << "No cell for this human position : x = " << x << " y = " << y << " Rz = " << Rz << endl;
        bestLocalConf.clearAll();
        return bestLocalConf;
    }
    if (!cell->isHumAccessible())
    {
        //        cout << "No human traj found for that position : x = " << x << " y = " << y << " Rz = " << Rz << endl;
        bestLocalConf.clearAll();
        return bestLocalConf;
    }
    cell->addPoint(Rz);

    for(unsigned int i = 0; i < vectConfs.size(); i++)
    {
        pair<confPtr_t,confPtr_t > conf = setRobotsToConf(i,true,x,y,Rz);

        conf.first->setAsNotTested();
        conf.second->setAsNotTested();
        //        string humanStr = testCol(true,false)?"is in collision":"is NOT in collision";
        //        string robotStr = testCol(false,false)?"is in collision":"is NOT in collision";
        //            cout << "test colision : human " << humanStr << " and robot " << robotStr << endl;
        if (!testCol(true,false) && !testCol(false,true))
        {
            bestLocalConf.humanConf = conf.first;
            bestLocalConf.robotConf = conf.second;
            bestLocalConf.configNumberInList = i;
            configCost = vectConfs.at(i).getCost();
            //                cout << "the choosed configuration in the configuration list is : " << i << endl;
            break;
        }
    }

    if (configCost < 0)
    {
        //        cout << "No conf found for that position : x = " << x << " y = " << y << " Rz = " << Rz << endl;
        bestLocalConf.clearAll();
        return bestLocalConf;
    }

    //    cout << "Config cost = " << configCost << endl;
    //    cout << "human start coord : x " << scell->getCoord()[0] << " y " << scell->getCoord()[1] << endl;
    //    cout << "human goal coord : x " << cell->getCoord()[0] << " y " << cell->getCoord()[1] << endl;

    double hDist = cell->getHumanDist() / m_2DGrid->getHumanMaxDist();

    if (m_isInitSiting)
    {
        hDist += sittingOffset;
    }

    bestLocalConf.humanTraj = cell->getHumanVectorTraj();
    bestLocalConf.humanTrajExist = false;
    if (bestLocalConf.humanTraj.size()> 0)
    {
        bestLocalConf.humanTrajExist = true;
    }

    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
    double hRot = angle_limit_PI((*bestLocalConf.humanConf)[firstIndexOfHumanDof + 5] - m_humanPos[2]);

    double mvCost = (psi*hDist + delta*fabs(hRot) )/(psi+delta);

    //    cout << "moving cost = " << mvCost << endl;

    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getP3dRobotStruct()->baseJnt)->user_dof_equiv_nbr;

    goalPos[0] = (*bestLocalConf.robotConf)[firstIndexOfRobotDof + 0];
    goalPos[1] = (*bestLocalConf.robotConf)[firstIndexOfRobotDof + 1];

    if (goalPos[0] < XYZ_ENV->box.x1 ||  goalPos[0] > XYZ_ENV->box.x2 || goalPos[1] < XYZ_ENV->box.y1 || goalPos[1] > XYZ_ENV->box.y2)
    {
        //        cout << "Robot out of bound : x = " << goalPos[0] << " y = " << goalPos[1] << " Rz = " << endl;
        bestLocalConf.clearAll();
        return bestLocalConf;
    }

    EnvCell* rCell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(goalPos));
    if (!rCell)
    {
        //cout << "No cell for this robot position : x = " << goalPos[0] << " y = " << goalPos[1] << endl;
        bestLocalConf.clearAll();
        return bestLocalConf;
    }
    if (!rCell->isRobAccessible())
    {
        //        cout << "No robot traj found for that position : x = " << x << " y = " << y << " Rz = " << Rz << endl;
        //        cout << "coords : x = " << cell->getCoord()[0] << " y = " << cell->getCoord()[1] << endl;
        bestLocalConf.clearAll();
        return bestLocalConf;
    }

    //    cout << "robot goal coord : x " << cell->getCoord()[0] << " y " << cell->getCoord()[1] << endl;

    double rDist = rCell->getRobotDist() / m_2DGrid->getRobotMaxDist();

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > tmp = rCell->getRobotVectorTraj();
    tmp.insert(tmp.begin(),robCurrentPos);

    bestLocalConf.robotTraj = tmp;
    bestLocalConf.robotTrajExist = false;
    if (bestLocalConf.robotTraj.size()> 0)
    {
        bestLocalConf.robotTrajExist = true;
    }

    double hTime = hDist/humanSpeed;

    double rTime = rDist/robotSpeed;

    double tempCost = hTime * timeStamp;
    if (hTime < rTime)
    {
        tempCost = rTime * timeStamp;
    }

    //    cout << "temporal cost = " << tempCost << endl;

    bestLocalConf.cost = (ksi * mvCost + rho * configCost) * (1 - objectNecessity)/(rho + ksi) + tempCost * objectNecessity;

    int time = PlanEnv->getInt(PlanParam::env_timeShow)*1000;
    if (ENV.getBool(Env::drawGraph) && !PlanEnv->getBool(PlanParam::env_drawOnlyBest) && (time > 0))
    {
        g3d_draw_allwin_active();
        usleep(time);
    }



    _Robot->setAndUpdate(*q_robot_cur);
    m_Human->setAndUpdate(*q_human_cur);

    return bestLocalConf;
}

OutputConf OTPMotionPl::findBestPosForHumanSitConf(double objectNecessity)
{

    confPtr_t q_robot_cur = _Robot->getCurrentPos();
    confPtr_t q_human_cur = m_Human->getCurrentPos();

    OutputConf bestLocalConf;
    bestLocalConf.cost = numeric_limits<double>::max( );
    OutputConf bestConf;
    bestConf.cost = numeric_limits<double>::max( );
    double ksi = PlanEnv->getDouble(PlanParam::env_ksi);//0.5;
    double rho = PlanEnv->getDouble(PlanParam::env_rho);//0.5;

    double robotSpeed = PlanEnv->getDouble(PlanParam::env_robotSpeed);//1;
    double timeStamp = PlanEnv->getDouble(PlanParam::env_timeStamp);//0.1;
    double sittingTime = PlanEnv->getDouble(PlanParam::env_sitTimeLimitation);
    double dumpTime = PlanEnv->getDouble(PlanParam::env_timeToDump);
    double currntDumpTime = 0;


    int nbSittingRot = PlanEnv->getInt(PlanParam::env_nbSittingRotation);//100
    int nbRandomRotOnly = PlanEnv->getInt(PlanParam::env_nbRandomRotOnly);//10


//    Robot* furniture = NULL;
//    detectSittingFurniture(m_Human,1,&furniture);

    vector<p3d_rob*> furnitures;
    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);
        if (p3d_col_test_robot_other(m_Human->getP3dRobotStruct(),XYZ_ENV->robot[i],1) == 1)
        {
            cout << "robot : " << name << " is in : "<< p3d_col_test_robot_other(m_Human->getP3dRobotStruct(),XYZ_ENV->robot[i],1) << " with the human " << endl;

            furnitures.push_back(XYZ_ENV->robot[i]);
            p3d_col_deactivate_robot_robot(m_Human->getP3dRobotStruct(), XYZ_ENV->robot[i]);
        }


    }

//    if (furniture!= NULL)
//    {
//        p3d_col_deactivate_robot_robot(m_Human->getP3dRobotStruct(), furniture->getP3dRobotStruct());
//    }


    //    cout << "sitting test" << endl;
    double configCost = 0;
    int id = 0;
    vector<ConfigHR> vectConfs;
    if (PlanEnv->getBool(PlanParam::env_useSlice) || PlanEnv->getBool(PlanParam::env_useOrientedSlice))
    {
        vectConfs = m_sittingConfigListSlice;
    }
    else
    {
        vectConfs = m_sittingConfigList;
    }

    if (vectConfs.empty())
    {
        return bestLocalConf;
    }
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
    confPtr_t q_hum = m_Human->getCurrentPos();

    m_2DGrid->setAsNotSorted();

    double elapsedTime = 0.0;
    clock_t beginLoop = clock();
    for (int j = 0; j < nbSittingRot; j ++)
    {
        if (j!= 0)
        {
            (*q_hum)[firstIndexOfHumanDof + 5] =  p3d_random(- M_PI , M_PI);
        }

        m_Human->setAndUpdate(*q_hum);
        for(unsigned int i = 0; i < vectConfs.size(); i++)
        {
            //        cout << "---------------------------" << endl;
            pair<confPtr_t,confPtr_t > conf = setRobotsToConf(i,false);

            conf.first->setAsNotTested();
            conf.second->setAsNotTested();


            //            string humanStr = testCol(true,false)?"is in collision":"is NOT in collision";
            //            string robotStr = testCol(false,false)?"is in collision":"is NOT in collision";
            //        cout << "test colision : human " << humanStr << " and robot " << robotStr << endl;

            if (!testCol(true,false) && !testCol(false,true))
            {
                int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getP3dRobotStruct()->baseJnt)->user_dof_equiv_nbr;
                Vector2d goalPos;
                goalPos[0] = (*conf.second)[firstIndexOfRobotDof + 0];
                goalPos[1] = (*conf.second)[firstIndexOfRobotDof + 1];

                if (goalPos[0] < XYZ_ENV->box.x1 ||  goalPos[0] > XYZ_ENV->box.x2 || goalPos[1] < XYZ_ENV->box.y1 || goalPos[1] > XYZ_ENV->box.y2)
                {
                    //                cout << "Robot out of bound : x = " << goalPos[0] << " y = " << goalPos[1] << " Rz = " << endl;
                    continue;
                }

                EnvCell* cell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(goalPos));

                if (!cell)
                {
                    //                cout << "No cell for this robot position : x = " << goalPos[0] << " y = " << goalPos[1] << endl;
                    continue;
                }
                if (cell->isRobAccessible())
                {

                    OutputConf localConf;
                    localConf.humanConf = conf.first;
                    localConf.robotConf = conf.second;
                    localConf.configNumberInList = i;
                    configCost = vectConfs.at(i).getCost();
                    double tempCost = (cell->getRobotDist() / m_2DGrid->getRobotMaxDist())/robotSpeed * timeStamp;
                    double mvCost = 0 ;
                    localConf.cost = (ksi * mvCost + rho * configCost) * (1 - objectNecessity)/(rho + ksi) + tempCost * objectNecessity;
                    localConf.robotTraj = cell->getRobotVectorTraj();
                    localConf.humanTrajExist = false;
                    localConf.robotTrajExist = false;
                    if (localConf.robotTraj.size()> 0)
                    {
                        localConf.robotTrajExist = true;
                    }

                    localConf.isStandingInThisConf = false;
                    //                cout << "global cost = " << localConf.cost<< endl;
                    localConf.id = id++;
                    if (m_simpleChair)
                    {
                        localConf.chairConf = m_simpleChair->getCurrentPos();
                    }

                    Vector2d v;
                    v[0] = (*q_hum)[firstIndexOfHumanDof + 0];
                    v[1] = (*q_hum)[firstIndexOfHumanDof + 1];

                    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > vect;
                    vect.clear();
                    vect.push_back(v);
//                    vect.push_back(v);
                    localConf.humanTraj = vect;
                    localConf.humanTrajExist = true;

                    if (localConf.cost < bestConf.cost)
                    {
                        bestConf = localConf;
                    }
                    m_confList.push_back(localConf);

                    //saveCostsTofile(bestConf.cost,localConf.cost);

                    int time = PlanEnv->getInt(PlanParam::env_timeShow)*1000;
                    if (ENV.getBool(Env::drawGraph) && !PlanEnv->getBool(PlanParam::env_drawOnlyBest) && (time > 0))
                    {
                        g3d_draw_allwin_active();
                        usleep(time);
                    }
                    break;

                    //                cout << "the choosed configuration in the sitting configuration list is : " << i << endl;
                }
                
            }
            clock_t tmpTime = clock();
            elapsedTime = ((double)tmpTime - beginLoop) / CLOCKS_PER_SEC;
            if (elapsedTime > dumpTime+currntDumpTime)
            {
                m_costVector.push_back(bestConf.cost);
                currntDumpTime +=dumpTime;
            }
            if (elapsedTime > sittingTime)
            {
                break;
            }
            
        }
        if (elapsedTime > dumpTime+currntDumpTime)
        {
            m_costVector.push_back(bestConf.cost);
            currntDumpTime +=dumpTime;
        }
        if (elapsedTime > sittingTime)
        {
            break;
        }

        if (id > nbRandomRotOnly )
        {
            break;
        }

    }

    m_sittingTime = elapsedTime;

    _Robot->setAndUpdate(*q_robot_cur);
    m_Human->setAndUpdate(*q_human_cur);

    standUp();

    getInputs();
    Eigen::Vector3d humPos;
    humPos[0] = m_humanPos[0];
    humPos[1] = m_humanPos[1];
    humPos[2] = m_humanPos[2];
    PlanEnv->setBool(PlanParam::env_isStanding, true);
    m_2DGrid->initGrid(humPos);
    PlanEnv->setBool(PlanParam::env_isStanding, false);
//
//    if (furniture!= NULL)
//    {
//        p3d_col_activate_robot_robot(m_Human->getP3dRobotStruct(), furniture->getP3dRobotStruct());
//    }
    for (unsigned int j =0; j < furnitures.size(); j++)
    {
        p3d_col_activate_robot_robot(m_Human->getP3dRobotStruct(), furnitures.at(j));
    }

    return bestConf;

}

bool  OTPMotionPl::testCol(bool isHuman, bool useConf)
{
//    int* is_human = new int();
//    int* agent_idx = new int();
//    hri_is_robot_an_agent(m_Human->getP3dRobotStruct(), GLOBAL_AGENTS,  is_human, agent_idx);
//    cout << hri_agent_get_posture(GLOBAL_AGENTS->all_agents[*agent_idx]) << endl;

    bool ret = true;
    if (isHuman)
    {
        if (useConf)
        {
            confPtr_t q(m_Human->getCurrentPos());
            q->setAsNotTested();
            ret = q->isInCollision();
        }
        else
        {
            ret = m_Human->isInCollisionWithOthersAndEnv();
        }
    }
    else
    {
        if (useConf)
        {
            confPtr_t q(_Robot->getCurrentPos());
            q->setAsNotTested();
            ret = q->isInCollision();
        }
        else
        {

            ret = _Robot->isInCollision();
        }
    }

    return ret;
}

void OTPMotionPl::saveInitConf()
{
    m_savedConf.humanConf = m_Human->getCurrentPos();
    m_savedConf.robotConf = _Robot->getCurrentPos();
    if (m_simpleChair)
    {
        m_savedConf.chairConf = m_simpleChair->getCurrentPos();
    }

}

void OTPMotionPl::loadInitConf(bool reloadHuman, bool relaodRobot)
{
    if (m_savedConf.humanConf && m_savedConf.robotConf)
    {
        if (reloadHuman)
        {
            m_Human->setAndUpdate(*m_savedConf.humanConf);
            if (m_simpleChair)
            {
                m_simpleChair->setAndUpdate(*m_savedConf.chairConf);
            }
        }
        if (relaodRobot)
        {
            _Robot->setAndUpdate(*m_savedConf.robotConf);
        }

    }
}

double OTPMotionPl::showConf(unsigned int i)
{
    if (m_confList.size() > i && m_confList.size() >1)
    {
        int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
        OutputConf conf = m_confList.at(i);
        m_2DPath = conf.robotTraj;
        m_PathExist = conf.robotTrajExist;
        double xf = (*conf.humanConf)[firstIndexOfHumanDof + 0];
        double yf = (*conf.humanConf)[firstIndexOfHumanDof + 1];
        double Rzf = angle_limit_PI((*conf.humanConf)[firstIndexOfHumanDof + 5]);
        m_2DHumanPath = conf.humanTraj;
        m_HumanPathExist = conf.humanTrajExist;
        setRobotsToConf(conf.configNumberInList,conf.isStandingInThisConf,xf,yf,Rzf);
        if (m_simpleChair)
        {
            m_simpleChair->setAndUpdate(*conf.chairConf);
        }

        return conf.cost;
    }
    return -1;
}

OutputConf OTPMotionPl::showBestConf()
{
    int id = 0;
    for (unsigned int i = 1; i < m_confList.size(); i++)
    {
        if (m_confList.at(i).cost < m_confList.at(id).cost)
        {
            id = i;
        }
    }
    showConf(id);
    return m_confList.at(id);
}

confPtr_t OTPMotionPl::getBestConf()
{
    int id = 0;
    for (unsigned int i = 1; i < m_confList.size(); i++)
    {
        if (m_confList.at(i).cost < m_confList.at(id).cost)
        {
            id = i;
        }
    }
    return m_confList.at(id).robotConf;
}

void OTPMotionPl::showBestConfRobOnly()
{
    confPtr_t ptrQ_human = m_Human->getCurrentPos();
    showBestConf();
    m_Human->setAndUpdate(*ptrQ_human);
}

void OTPMotionPl::initGrid()
{
    cout << "---------------------------" << endl;
    cout << "Begin initialising the grid\n" << endl;


    if(!PlanEnv->getBool(PlanParam::env_isStanding))
    {
        cout << "case: human is sitting\n" << endl;
        Robot* chair = NULL;
        for (int i=0; i<XYZ_ENV->nr; i++)
        {
            string name(XYZ_ENV->robot[i]->name);
            if(name.find("SIMPLECHAIR") != string::npos )
            {
                chair = new Robot(XYZ_ENV->robot[i]);
                break;
            }
        }
        confPtr_t q_chair_cur = chair->getCurrentPos();
        confPtr_t q_human = m_Human->getCurrentPos();

        cout << "make human stand" << endl;
        if(standUp())
        {
            cout << "human is standing\n" << endl;
        }
        clock_t start = clock();
        m_2DGrid->init(computeHumanRobotDist());
        clock_t second = clock();
        getInputs();
        Eigen::Vector3d humPos;
        humPos[0] = m_humanPos[0];
        humPos[1] = m_humanPos[1];
        humPos[2] = m_humanPos[2];
        m_2DGrid->initGrid(humPos);
        clock_t stop = clock();

        m_Human->setAndUpdate(*q_human);
        chair->setAndUpdate(*q_chair_cur);
        cout << "Human is sitting" << endl;

        cout << "initializing the grid took : " << ((double)second - start) / CLOCKS_PER_SEC << " s"<< endl;
        cout << "initializing the content of the grid take : " << ((double)stop - second) / CLOCKS_PER_SEC << " s"<< endl;


    }
    else
    {
        cout << "case: human is standing\n" << endl;
        clock_t start = clock();
        m_2DGrid->init(computeHumanRobotDist());
        clock_t second = clock();
        getInputs();
        Eigen::Vector3d humPos;
        humPos[0] = m_humanPos[0];
        humPos[1] = m_humanPos[1];
        humPos[2] = m_humanPos[2];
        m_2DGrid->initGrid(humPos);
        clock_t stop = clock();

        cout << "initializing the grid take : " << ((double)second - start) / CLOCKS_PER_SEC << " s"<< endl;
        cout << "initializing the content of the grid take : " << ((double)stop - second) / CLOCKS_PER_SEC << " s"<< endl;
        initTime = ((double)second - start) / CLOCKS_PER_SEC;
    }

    cout << "Grid initialized with sucess" << endl;
    cout << "---------------------------" << endl;

}

double OTPMotionPl::getDistFromCell(int x, int y, bool isHuman)
{
    EnvCell* cell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(x,y));
    if (isHuman)
    {
        if (cell->isHumanDistComputed())    { return cell->getHumanDist();}
        else                                { return -1.0; }
    }
    else
    {
        if (cell->isRobotDistComputed())    { return cell->getRobotDist();}
        else                                { return -1.0; }
    }
}

double OTPMotionPl::getrotFromCell(int x, int y)
{
    EnvCell* cell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(x,y));
    if (cell->isHumAccessible())
    {
        return cell->getAngleForHumanComming();
    }
    else
        return numeric_limits<double>::max();
}

void OTPMotionPl::saveAllCostsToFile()
{
    unsigned int maxSize = 0;
    for (unsigned int costVectorId=0; costVectorId < m_multipliComputeCostVector.size(); costVectorId++)
    {
        if (m_multipliComputeCostVector.at(costVectorId).size() > maxSize)
        {
            maxSize = m_multipliComputeCostVector.at(costVectorId).size();
        }
    }

    for (unsigned int costVectorId=0; costVectorId < m_multipliComputeCostVector.size(); costVectorId++)
    {
        vector<double> tmpCostVctor = m_multipliComputeCostVector.at(costVectorId);
        if (tmpCostVctor.size() < maxSize)
        {
            double lastCost = tmpCostVctor.at(tmpCostVctor.size() - 1);
            int tmpIter = maxSize - tmpCostVctor.size();
            for (int i = 0; i < tmpIter; i++)
            {
                tmpCostVctor.push_back(lastCost);
            }
            m_multipliComputeCostVector[costVectorId] = tmpCostVctor;
        }
    }

    ofstream myfile;
    string fileName = "/statFiles/OtpComputing/configCosts.lst";
    string home = getenv("HOME_MOVE3D") + fileName;
    myfile.open(home.c_str());

    ofstream fileVariance;
    fileName = "/statFiles/OtpComputing/costsVariance.lst";
    home = getenv("HOME_MOVE3D") + fileName;
    fileVariance.open(home.c_str());

    ofstream fileMin;
    fileName = "/statFiles/OtpComputing/costsMin.lst";
    home = getenv("HOME_MOVE3D") + fileName;
    fileMin.open(home.c_str());

    ofstream fileMax;
    fileName = "/statFiles/OtpComputing/costsMax.lst";
    home = getenv("HOME_MOVE3D") + fileName;
    fileMax.open(home.c_str());


    double meanCost = 0;
    for (unsigned int i = 0; i < maxSize; i++)
    {
        meanCost = 0;
        double minCost = numeric_limits<double>::max( );;
        double maxCost = 0;
        for (unsigned int costVectorId=0; costVectorId < m_multipliComputeCostVector.size(); costVectorId++)
        {
            double cost = m_multipliComputeCostVector.at(costVectorId).at(i);
            if (cost > 10)
            {
                cost = 2;
            }
            meanCost += cost;
            if (cost > maxCost)
            {
                maxCost = cost;
            }
            if (cost < minCost)
            {
                minCost = cost;
            }
        }
        meanCost = meanCost / m_multipliComputeCostVector.size();
        myfile << meanCost << endl;
        fileMin << minCost <<endl;
        fileMax << maxCost <<endl;

        double variance = 0;
        for (unsigned int costVectorId=0; costVectorId < m_multipliComputeCostVector.size(); costVectorId++)
        {
            double cost = m_multipliComputeCostVector.at(costVectorId).at(i);
            if (cost > 10)
            {
                cost = 2;
            }
            variance += pow(cost - meanCost,2);
        }
        variance = variance/ m_multipliComputeCostVector.size();
        fileVariance << variance << endl;
    }
    myfile.close();
    fileVariance.close();
    fileMin.close();
    fileMax.close();
}

void OTPMotionPl::saveAlltimesToFile()
{
//    double dumpTime = PlanEnv->getDouble(PlanParam::env_timeToDump) *1000;
    unsigned int maxSize = 0;
    for (unsigned int timeVectorId=0; timeVectorId < m_multipliComputeTimeVector.size(); timeVectorId++)
    {
        if (m_multipliComputeTimeVector.at(timeVectorId).size() > maxSize)
        {
            maxSize = m_multipliComputeTimeVector.at(timeVectorId).size();
        }
    }

    for (unsigned int timeVectorId=0; timeVectorId < m_multipliComputeTimeVector.size(); timeVectorId++)
    {
        vector<double> tmpCostVctor = m_multipliComputeTimeVector.at(timeVectorId);
        if (tmpCostVctor.size() < maxSize)
        {
            double lastCost = tmpCostVctor.at(tmpCostVctor.size() - 1);
            int tmpIter = maxSize - tmpCostVctor.size();
            for (int i = 0; i < tmpIter; i++)
            {
                lastCost+=10;
                tmpCostVctor.push_back(lastCost);
            }
            m_multipliComputeTimeVector[timeVectorId] = tmpCostVctor;
        }
    }

    ofstream myfile;
    string fileName = "/statFiles/OtpComputing/configTime.lst";
    string home = getenv("HOME_MOVE3D") + fileName;
    myfile.open(home.c_str());


    for (unsigned int i = 0; i < maxSize; i++)
    {
        for (unsigned int timeVectorId=0; timeVectorId < m_multipliComputeTimeVector.size(); timeVectorId++)
        {
            double time = m_multipliComputeTimeVector.at(timeVectorId).at(i);
            myfile << time << " ";
        }
        myfile << endl;
    }
    myfile.close();
}

void OTPMotionPl::saveCostsTofile(double cost, double randomCost)
{
    ofstream myfile;
    string fileName = "/statFiles/OtpComputing/configurationsCosts.lst";
    string home = getenv("HOME_MOVE3D") + fileName;
    myfile.open (home.c_str(),ios::app);
    //myfile << cost << "\t" << randomCost << endl;
    if (cost > 10)
    {
        cost = 2;
    }
    myfile << cost << endl;
    myfile.close();
}

void OTPMotionPl::saveCostsTofile(string cost)
{
    ofstream myfile;
    string fileName = "/statFiles/OtpComputing/configurationsCosts.lst";
    string home = getenv("HOME_MOVE3D") + fileName;
    myfile.open (home.c_str(),ios::app);
    myfile << cost << endl;
    myfile.close();
}

void OTPMotionPl::clearCostsfile()
{
    ofstream myfile;
    string fileName = "/statFiles/OtpComputing/configurationsCosts.lst";
    string home = getenv("HOME_MOVE3D") + fileName;
    myfile.open (home.c_str());
    myfile << "";
    myfile.close();
}

int OTPMotionPl::getConfListSize()
{
    if (PlanEnv->getBool(PlanParam::env_isStanding))
    {
        return m_configList.size();
    }
    else
    {
        return m_sittingConfigList.size();
    }
}

configPt OTPMotionPl::getRobotConfigAt(int i)
{
    if (PlanEnv->getBool(PlanParam::env_isStanding))
    {
        return m_configList.at(i).getRobotConf();
    }
    else
    {
        return m_sittingConfigList.at(i).getRobotConf();
    }
}

std::vector<ConfigHR> OTPMotionPl::getConfList()
{
    if (PlanEnv->getBool(PlanParam::env_drawSlice) || PlanEnv->getBool(PlanParam::env_useOrientedSlice))
    {
        if (PlanEnv->getBool(PlanParam::env_isStanding))
        {
            return m_configListSlice;
        }
        else
        {
            return m_sittingConfigListSlice;
        }
    }
    else
    {
        if (PlanEnv->getBool(PlanParam::env_isStanding))
        {
            return m_configList;
        }
        else
        {
            return m_sittingConfigList;
        }
    }
}

bool OTPMotionPl::createTrajectoryFromOutputConf(OutputConf conf)
{
    clock_t start = clock();
    ENV.setBool(Env::isCostSpace,false);

    bool softmotionTraj = PlanEnv->getBool(PlanParam::env_trajSoftMotion);
    bool normalTraj = PlanEnv->getBool(PlanParam::env_trajNormal);
    bool rosTraj = PlanEnv->getBool(PlanParam::env_trajRos);

    m_ManipPl->setNavigationPlanner();
    //    ENV.setInt(Env::NbTry,10);

    vector<confPtr_t > robotVectorConf;

    p3d_multiLocalPath_disable_all_groupToPlan( _Robot->getP3dRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( _Robot->getP3dRobotStruct(), m_ManipPl->getUpBodyMLP(), 1, false);

    Move3D::Trajectory p3d_trajectory(_Robot);
    p3d_trajectory.clear();

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > robotTraj2D = conf.robotTraj;


    loadInitConf(true,true);
    confPtr_t q_cur_human(m_Human->getCurrentPos());
//    confPtr_t q_cur_robot(_Robot->getCurrentPos());

    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getP3dRobotStruct()->baseJnt)->user_dof_equiv_nbr;
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();

    string name( "Config_" );


    int nbConf = _Robot->getP3dRobotStruct()->nconf;


    for(int i=0 ; i<nbConf; i++)
    {
        p3d_del_config(_Robot->getP3dRobotStruct(), _Robot->getP3dRobotStruct()->conf[0]);
    }

#ifdef P3D_PLANNER
    if( !p3d_del_graph(XYZ_GRAPH) )
    {
#ifdef DEBUG_STATUS
        cerr << "XYZ_GRAPH allready deleted" << endl;
#endif
    }
#ifdef DEBUG_STATUS
    cerr <<  "XYZ_GRAPH = " << XYZ_GRAPH << endl;
#endif

#endif


    int id =0;

    clock_t beginCreate = clock();

    clock_t justSmoothing = clock();
    clock_t endCreateTraj = clock();
    int msg = 0;
    bool trajTest = NULL;
    if (robotTraj2D.size() > 1)
    {
        // in this part, three solution is available: using the normal trajectories of move3d, using softmotion, or getting a usable output for ROS usage.


        robotTraj2D = m_pts->smoothTrajectory(_Robot,robotTraj2D);

        std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > robotTraj3D;
        if (softmotionTraj || normalTraj)
        {
            robotTraj3D = m_pts->add3Dim(robotTraj2D,_Robot,0.05);
        }
        else if (rosTraj)
        {
            robotTraj3D = m_pts->add3DimwithoutTrajChange(robotTraj2D,_Robot,0.05);

        }
        else
        {
            cout << "ERROR: No chosen type of trajs" << endl;
            return false;
        }
        m_2DPath = m_pts->get2DtrajFrom3Dtraj(robotTraj3D);
        conf.robotTraj = m_2DPath;
        justSmoothing = clock();

        if (softmotionTraj || normalTraj)
        {
            for(unsigned int i =0; i < robotTraj3D.size(); i++)
            {

                confPtr_t q_tmp(_Robot->getInitPos());
                confPtr_t q_cur(_Robot->getInitPos());

                //                        cout << "cell nb: " << i << " coord =\n"<< robotTraj3D.at(i) << endl;

                (*q_tmp)[firstIndexOfRobotDof + 0] = robotTraj3D.at(i)[0];
                (*q_tmp)[firstIndexOfRobotDof + 1] = robotTraj3D.at(i)[1];
                (*q_tmp)[firstIndexOfRobotDof + 5] = angle_limit_PI(robotTraj3D.at(i)[2]);

                //            config_namePt config;
                string num;
                stringstream out;
                out << id++;
                num = out.str();

                if (i == robotTraj3D.size()-1)
                {
                    //TODO adding a better trajectory
                    (*q_tmp)[firstIndexOfRobotDof + 5] = (*conf.robotConf)[firstIndexOfRobotDof + 5];
                    p3d_trajectory.push_back(q_tmp);
                    p3d_trajectory.push_back(conf.robotConf);

                }
                else if (i == 0)
                {
                    //TODO change the departure
                    p3d_trajectory.push_back(q_cur);
                    p3d_trajectory.push_back(q_tmp);
                }
                else
                {
                    p3d_trajectory.push_back(q_tmp);
                }

            }

            trajTest = p3d_trajectory.replaceP3dTraj( _Robot->getP3dRobotStruct()->tcur );

    //        p3d_trajectory.print();
            endCreateTraj = clock();

            if (softmotionTraj)
            {
                p3d_rob * robotPt =  _Robot->getP3dRobotStruct();
                ManipulationPlanner m_p(robotPt);

                MANPIPULATION_TRAJECTORY_CONF_STR conf;
                SM_TRAJ smTraj;

                msg = m_p.computeSoftMotion( _Robot->getP3dRobotStruct()->tcur, conf, smTraj);
                m_smTrajs.clear();
                m_smTrajs.push_back(smTraj);
            }


        }
        else if (rosTraj)
        {
            Eigen::Vector3d endingPos;
            endingPos[0] = (*conf.robotConf)[firstIndexOfRobotDof + 0];
            endingPos[1] = (*conf.robotConf)[firstIndexOfRobotDof + 1];
            endingPos[2] = (*conf.robotConf)[firstIndexOfRobotDof + 5];
            m_2DPath = m_pts->get2DtrajFrom3Dtraj(robotTraj3D);
            robotTraj3D.push_back(endingPos);
            m_robotTraj3D = robotTraj3D;
            conf.robotTraj = m_2DPath;

        }
        else
        {
            cout << "ERROR: No chosen type of trajs" << endl;
            return false;
        }
    }



//    clock_t endAddingConfs = clock();

    clock_t end = clock();

    double initCreateTrajTime = ((double)beginCreate - start) / CLOCKS_PER_SEC;
    double justSmooth = ((double)justSmoothing - beginCreate) / CLOCKS_PER_SEC;
//    double smoothCreateTrajTime = ((double)endAddingConfs - justSmoothing) / CLOCKS_PER_SEC;
//    double createTrajectory = ((double)endCreateTraj - endAddingConfs) / CLOCKS_PER_SEC;
    double softMotionCreateTrajTime = ((double)end - endCreateTraj) / CLOCKS_PER_SEC;

    if (PlanEnv->getBool(PlanParam::env_showText))
    {
        cout << "----------" << endl <<"time elapsed to : " << endl;
        cout << "init create traj = " << initCreateTrajTime << " s"<< endl;
        cout << "just smoothing traj = " << justSmooth << " s"<< endl;
//        cout << "adding conf to traj = " << smoothCreateTrajTime << " s"<< endl;
//        cout << "creating traj = " << createTrajectory << " s"<< endl;
        cout << "applying softmotion = " << softMotionCreateTrajTime << " s"<< endl;
        cout << "----------" << endl;
    }
    if (robotTraj2D.size() > 1)
    {
        if (!trajTest && (softmotionTraj || normalTraj))
        {
            cout << "Fail: in replaceP3dTraj(), no traj found" <<endl;
            return false;
        }
        else if (msg != 0)
        {
            cout << "Fail: in computeSoftMotion(), no traj found" <<endl;
            return false;
        }
    }



    // this part is for moving human.


    if (PlanEnv->getBool(PlanParam::env_computeTrajForHuman))
    {
        vector<confPtr_t > humanVectorConf;
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > humanTraj2D = conf.humanTraj;
        humanVectorConf.push_back(q_cur_human);
        if (humanTraj2D.size() > 2)
        {
            humanTraj2D = m_pts->smoothTrajectory(m_Human,humanTraj2D);
            m_2DHumanPath = humanTraj2D;
            for(unsigned int i =0; i < humanTraj2D.size() - 1; i++)
            {
                confPtr_t q_tmp(m_Human->getCurrentPos());
                (*q_tmp)[firstIndexOfHumanDof + 0] = humanTraj2D.at (i)[0];
                (*q_tmp)[firstIndexOfHumanDof + 1] = humanTraj2D.at(i)[1];
//                if (humanTraj2D.at(i+1)[0] != humanTraj2D.at(i)[0])
//                {
                    (*q_tmp)[firstIndexOfRobotDof + 5] = atan2(
                            humanTraj2D.at(i+1)[1] - humanTraj2D.at(i)[1],
                            humanTraj2D.at(i+1)[0] - humanTraj2D.at(i)[0]);
//                }

                humanVectorConf.push_back(q_tmp);
                if (conf.isStandingInThisConf && m_humanCanStand)
                {
                    standUp();
                }
            }
        }
        humanVectorConf.push_back(conf.humanConf);

        if (humanVectorConf.size() > 1)
        {
            p3d_multiLocalPath_disable_all_groupToPlan( m_Human->getP3dRobotStruct() , false );
            p3d_multiLocalPath_set_groupToPlan( m_Human->getP3dRobotStruct(), m_ManipPlHum->getBaseMLP(), 1, false);

            //////////////////
            Move3D::Trajectory base_hum_traj(humanVectorConf);
            base_hum_traj.replaceHumanP3dTraj( m_Human, m_Human->getP3dRobotStruct()->tcur );
        }
        else
        {
            if ( m_Human->getTrajStruct())
            {
                destroy_list_localpath( m_Human->getP3dRobotStruct(), m_Human->getP3dRobotStruct()->tcur->courbePt );
                m_Human->getP3dRobotStruct()->tcur = NULL;
            }
        }
    }


    ENV.setBool(Env::isCostSpace,true);
    return true;
}

std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > OTPMotionPl::smoothTrajectory(
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > trajectory)
{

    int id =0;
    for (unsigned int i = 1; i < m_confList.size(); i++)
    {
        if (m_confList.at(i).cost < m_confList.at(id).cost)
        {
            id = i;
            break;
        }
    }
    m_2DPath = m_pts->smoothTrajectory(_Robot,trajectory);
    m_confList.at(id).robotTraj = m_2DPath;

    return m_2DPath;
}

void OTPMotionPl::navigate()
{
    int id = 0;
    for (unsigned int i = 1; i < m_confList.size(); i++)
    {
        if (m_confList.at(i).cost < m_confList.at(id).cost)
        {
            id = i;
            break;
        }
    }
    //    m_confList.at(id);

    //    m_savedConf.robotConf;

    std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
    std::vector <SM_TRAJ> smTrajs;

    m_ManipPl->planNavigation(m_savedConf.robotConf->getConfigStruct(), m_confList.at(id).robotConf->getConfigStruct(), true ,confs, smTrajs);
}

bool OTPMotionPl::testCurrentTraj()
{
    Robot* robCyl = NULL;
    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);
        if(name.find("PR_2CYLINDER") != string::npos )
        {
            robCyl = new Robot(XYZ_ENV->robot[i]);
        }

    }
    return m_pts->robotCanDoTraj(m_2DPath,robCyl,_Robot,0.02);
}

void OTPMotionPl::initTrajTest()
{
    if (!m_pts)
    {
        m_pts = new PlannarTrajectorySmoothing(_Robot);
    }

    Robot* robCyl = NULL;
    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);
        if(name.find("PR_2CYLINDER") != string::npos )
        {
            robCyl = new Robot(XYZ_ENV->robot[i]);
        }

    }

    m_pts->initTraj(m_2DPath,robCyl);
}

void OTPMotionPl::goToNextStep()
{
    m_pts->goToNextStep();
}

std::pair<double,double> OTPMotionPl::computeHumanRobotDist()
{
    cout << "in: std::pair<double,double> OTPMotionPl::computeHumanRobotDist()\n" << endl;
    cout << "Find minimal and maximal distance between human and robot to make the transfert" << endl;
    pair<double,double> minMax;
    minMax.first = numeric_limits<double>::max( );
    minMax.second = 0;
    std::vector<ConfigHR> list;
    if (PlanEnv->getBool(PlanParam::env_isStanding))
    {
        list = m_configList;
    }
    else
    {
        list = m_sittingConfigList;
    }
    confPtr_t q_human (m_Human->getCurrentPos());
    confPtr_t q_robot (_Robot->getCurrentPos());

    for(unsigned int i = 0; i < list.size(); i++)
    {
        setRobotsToConf(i,PlanEnv->getBool(PlanParam::env_isStanding));
        double dist = getHumanRobotDist();

        if (dist > minMax.second)
        {
            minMax.second = dist;
        }
        if (dist < minMax.first)
        {
            minMax.first = dist;
        }
    }

    m_Human->setAndUpdate(*q_human);
    _Robot->setAndUpdate(*q_robot);

    cout << "Minimal and maximal distance found with sucess\n" << endl;
    cout << "out of: std::pair<double,double> OTPMotionPl::computeHumanRobotDist()\n\n" << endl;
    return minMax;

}

double OTPMotionPl::getHumanRobotDist()
{
    confPtr_t q_human (m_Human->getCurrentPos());
    confPtr_t q_robot (_Robot->getCurrentPos());

    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getP3dRobotStruct()->baseJnt)->user_dof_equiv_nbr;
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();

    double xh = (*q_human)[firstIndexOfHumanDof + 0];
    double yh = (*q_human)[firstIndexOfHumanDof + 1];
    double xr = (*q_robot)[firstIndexOfRobotDof + 0];
    double yr = (*q_robot)[firstIndexOfRobotDof + 1];


    return sqrt( pow( xh - xr , 2) + pow( yh - yr , 2) );
}

bool OTPMotionPl::standUp()
{
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();


    Robot* humCyl = NULL;
    Robot* chair = NULL;
    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);
        if(name.find("HUMCYLINDER") != string::npos )
        {
            humCyl = new Robot(XYZ_ENV->robot[i]);
        }
        if(name.find("SIMPLECHAIR") != string::npos )
        {
            chair = new Robot(XYZ_ENV->robot[i]);
        }

    }

    if (!humCyl)
    {
        cout << "No human cylinder found" << endl;
        return false;
    }
    if (!chair)
    {
        cout << "No simple chair found" << endl;
        return false;
    }

    confPtr_t q_human_cur = m_Human->getCurrentPos();
    confPtr_t q_human = m_Human->getCurrentPos();
    double x = (*q_human)[firstIndexOfHumanDof + 0];
    double y = (*q_human)[firstIndexOfHumanDof + 1];
    double Rz = angle_limit_PI((*q_human)[firstIndexOfHumanDof + 5]);

    confPtr_t q_chair_cur = chair->getCurrentPos();
    double xC = (*q_chair_cur)[firstIndexOfHumanDof + 0];
    double yC = (*q_chair_cur)[firstIndexOfHumanDof + 1];
    double RzC = angle_limit_PI((*q_chair_cur)[firstIndexOfHumanDof + 5]);


    (*q_human)[firstIndexOfHumanDof + 0] = -3;
    (*q_human)[firstIndexOfHumanDof + 1] = 0;
    m_Human->setAndUpdate(*q_human);

    confPtr_t q_humCyl_cur = humCyl->getCurrentPos();
    confPtr_t q_humCyl = humCyl->getCurrentPos();

    confPtr_t q_chair = chair->getCurrentPos();

    double chairDist = 0;
    double standingDist = 0;
    double maxStandingDist = 1;
    double discr = 0.1;
    while (humCyl->isInCollisionWithOthersAndEnv() && chairDist < maxStandingDist)
    {
        (*q_chair)[6] = xC - chairDist*cos(RzC);
        (*q_chair)[7] = yC - chairDist*sin(RzC);
        //        cout << (*q_chair)[6] << endl;
        //        cout << (*q_chair)[7] << endl;
        chair->setAndUpdate(*q_chair);
        standingDist = -0.2;
        while (humCyl->isInCollisionWithOthersAndEnv() && standingDist < maxStandingDist)
        {
            (*q_humCyl)[6] = x + standingDist*cos(Rz);
            (*q_humCyl)[7] = y + standingDist*sin(Rz);
            //            cout << (*q_humCyl)[6] << endl;
            //            cout << (*q_humCyl)[7] << endl;
            humCyl->setAndUpdate(*q_humCyl);
            standingDist += discr;
        }
        if(!humCyl->isInCollisionWithOthersAndEnv())
        {
            break;
        }
        else
        {
            chairDist += discr;
        }


    }

    if (!humCyl->isInCollisionWithOthersAndEnv())
    {
        getReachability()->setRobotToConfortPosture();
        q_human = m_Human->getCurrentPos();
        (*q_human)[firstIndexOfHumanDof + 0] = (*q_humCyl)[6];
        (*q_human)[firstIndexOfHumanDof + 1] = (*q_humCyl)[7];
        (*q_human)[firstIndexOfHumanDof + 2] = 1.07;
        m_Human->setAndUpdate(*q_human);
        m_humanCanStand = true;

    }
    else
    {
        m_Human->setAndUpdate(*q_human_cur);
        chair->setAndUpdate(*q_chair_cur);
        cout << "Human can not stand up : too much obstacle in front of him" << endl;
        m_humanCanStand = false;
    }


    humCyl->setAndUpdate(*q_humCyl_cur);
    return m_humanCanStand;
}

void OTPMotionPl::dumpVar()
{
    cout << "------------ OTP variable -----------" <<endl;

    cout << "fused grid = "  << PlanEnv->getBool(PlanParam::env_fusedGridRand) << endl;
    cout << "fused grid and rotation = "  << PlanEnv->getBool(PlanParam::env_fusedGridAndRotRand) << endl;
    cout << "normal rand = "  << PlanEnv->getBool(PlanParam::env_normalRand) << endl;
    cout << "oriented slice = "  << PlanEnv->getBool(PlanParam::env_useOrientedSlice) << endl;
    cout << "slices = "  << PlanEnv->getBool(PlanParam::env_useSlice) << endl;
    cout << "all grid = "  << PlanEnv->getBool(PlanParam::env_useAllGrid) << endl << endl;

    cout << " createTrajs = " << PlanEnv->getBool(PlanParam::env_createTrajs) << endl;
    cout << " computeTrajForHuman = " << PlanEnv->getBool(PlanParam::env_computeTrajForHuman) << endl;
#ifdef MULTILOCALPATH
    cout << " plotSoftMotionCurve = " << ENV.getBool(Env::plotSoftMotionCurve) << endl;
    cout << " writeSoftMotionFiles = " << ENV.getBool(Env::writeSoftMotionFiles) << endl;
    // WARNING REVERT XAVIER
    //cout << " exportSoftMotionTrajAsArrayOfConf = " << ENV.getBool(Env::exportSoftMotionTrajAsArrayOfConf) << endl;
    //cout << " smoothSoftMotionTraj = " << ENV.getBool(Env::smoothSoftMotionTraj) << endl;
#endif

    cout << " no repetition = " << PlanEnv->getBool(PlanParam::env_noRepetition) << endl;
    cout << " isStanding = " << PlanEnv->getBool(PlanParam::env_isStanding) << endl;
    cout << " objectNecessity = " <<PlanEnv->getDouble(PlanParam::env_objectNessecity) << endl;
    cout << " pow for random  = " << PlanEnv->getInt(PlanParam::env_pow) << endl;
    cout << " pow for rotation random  = " << PlanEnv->getInt(PlanParam::env_anglePow) << endl;
    cout << " sleeping time  = " << PlanEnv->getInt(PlanParam::env_timeShow) << endl;
    cout << " maxIter = " << PlanEnv->getInt(PlanParam::env_maxIter)<< endl;
    cout << " totMaxIter = " <<PlanEnv->getInt(PlanParam::env_totMaxIter) << endl;
    cout << " timeMax = " <<PlanEnv->getDouble(PlanParam::env_timeLimitation) << endl;
    cout << " nbRotation = " <<PlanEnv->getInt(PlanParam::env_nbRandomRotOnly) << endl;
    cout << " nb sitting Rotation = " <<PlanEnv->getInt(PlanParam::env_nbSittingRotation) << endl;
    cout << " robotSpeed = " << PlanEnv->getDouble(PlanParam::env_robotSpeed)<< endl;
    cout << " humanSpeed = " <<PlanEnv->getDouble(PlanParam::env_humanSpeed) << endl;
    cout << " timeStamp = " <<PlanEnv->getDouble(PlanParam::env_timeStamp) << endl << endl;

    cout << " psi = " <<PlanEnv->getDouble(PlanParam::env_psi) << endl;
    cout << " delta = " <<PlanEnv->getDouble(PlanParam::env_delta) << endl;
    cout << " ksi = " <<PlanEnv->getDouble(PlanParam::env_ksi) << endl;
    cout << " rho = " <<PlanEnv->getDouble(PlanParam::env_rho) << endl;
    cout << " sittingOffset = " <<PlanEnv->getDouble(PlanParam::env_sittingOffset) << endl << endl;

    cout << " Distance = " <<ENV.getDouble(Env::Kdistance) << endl;
    cout << " visibility = " <<ENV.getDouble(Env::Kvisibility) << endl;
    cout << " Reacheable = " <<ENV.getDouble(Env::Kreachable) << endl << endl;

    cout << " Neutral = " <<ENV.getDouble(Env::coeffJoint) << endl;
    cout << " Energie = " <<ENV.getDouble(Env::coeffEnerg) << endl;
    cout << " Disconfort = " <<ENV.getDouble(Env::coeffConfo) << endl<< endl;

    cout << " drawDisabled = " << ENV.getBool(Env::drawDisabled) << endl;

    cout << "------------ OTP variable end--------" <<endl;
}

void OTPMotionPl::setVar()
{
    PlanEnv->setBool(PlanParam::env_fusedGridRand,false);
    PlanEnv->setBool(PlanParam::env_fusedGridAndRotRand,true);
    PlanEnv->setBool(PlanParam::env_normalRand,false);
    PlanEnv->setBool(PlanParam::env_useOrientedSlice,false);
    PlanEnv->setBool(PlanParam::env_useSlice,false);
    PlanEnv->setBool(PlanParam::env_useAllGrid,false);

//    PlanEnv->setBool(PlanParam::env_createTrajs,true);
    PlanEnv->setBool(PlanParam::env_noRepetition,false);
    PlanEnv->setBool(PlanParam::env_computeTrajForHuman,false);

#ifdef MULTILOCALPATH
    ENV.setBool(Env::plotSoftMotionCurve,false);
    // WARNING REVERT XAVIER
    //ENV.setBool(Env::writeSoftMotionFiles,false);
    //ENV.setBool(Env::exportSoftMotionTrajAsArrayOfConf,false);
    ENV.setBool(Env::smoothSoftMotionTraj,true);
#endif

    PlanEnv->setInt(PlanParam::env_timeShow,0);
    PlanEnv->setDouble(PlanParam::env_Cellsize,0.20);
    PlanEnv->setInt(PlanParam::env_nbSittingRotation,1);

    PlanEnv->setInt(PlanParam::env_pow,2);
    PlanEnv->setInt(PlanParam::env_anglePow,3);
    PlanEnv->setInt(PlanParam::env_timeShow,0);
    PlanEnv->setInt(PlanParam::env_maxIter,200);
    PlanEnv->setInt(PlanParam::env_totMaxIter,2000);
    PlanEnv->setDouble(PlanParam::env_timeLimitation,10.0);
    PlanEnv->setInt(PlanParam::env_nbRandomRotOnly,50);
    PlanEnv->setInt(PlanParam::env_nbSittingRotation,1);
    PlanEnv->setDouble(PlanParam::env_robotSpeed,1.0);
    PlanEnv->setDouble(PlanParam::env_humanSpeed,1);
    PlanEnv->setDouble(PlanParam::env_timeStamp,0.35);

    PlanEnv->setDouble(PlanParam::env_psi,0.99);
    PlanEnv->setDouble(PlanParam::env_delta,0.01);
    PlanEnv->setDouble(PlanParam::env_ksi,0.73);
    PlanEnv->setDouble(PlanParam::env_rho,0.57);
    PlanEnv->setDouble(PlanParam::env_sittingOffset,0.2);

    ENV.setDouble(Env::Kdistance,40.0);
    ENV.setDouble(Env::Kvisibility,35.0);
    ENV.setDouble(Env::Kreachable,50.0);

    ENV.setDouble(Env::coeffJoint,0.5);
    ENV.setDouble(Env::coeffEnerg,1);
    ENV.setDouble(Env::coeffConfo,10);

    ENV.setBool(Env::drawDisabled,true);

}

bool OTPMotionPl::InitMhpObjectTransfert(std::string humanName)
{
    if (PlanEnv->getBool(PlanParam::env_isInit))
    {
        cout << "The function is already initialized. Init only the grid" << endl;
        initGrid();
        return true;
    }
    PlanEnv->setBool(PlanParam::env_isInit,true);
    changeHumanByName(humanName);

    //    HRICS_init();
    //    HRICS_MotionPLConfig  = new HRICS::OTPMotionPl;
    //    PlanEnv->setBool(PlanParam::env_isStanding,true);

    setVar();


    HRICS_activeDist = HRICS_MotionPL->getDistance();

    ENV.setBool(Env::HRIPlannerCS,true);
    ENV.setBool(Env::enableHri,true);
    ENV.setBool(Env::isCostSpace,true);

    ENV.setBool(Env::useBallDist,false);
    ENV.setBool(Env::useBoxDist,true);

    bool gridDrawing = ENV.getBool(Env::drawGrid);
    if(ENV.getBool(Env::HRIPlannerCS))
    {
        ENV.setBool(Env::drawGrid,true);
    }

    setReachability(HRICS_activeNatu);


    string home( getenv("HOME_MOVE3D") );

    string fileNameStand;
    string fileNameSit;
    string fileNameStandSlice;
    string fileNameSitSlice;


    if (m_Human->getName().find("HERAKLES")!= string::npos)
    {
        fileNameStand = "/statFiles/OtpComputing/confHerakles.xml";
        fileNameSit = "/statFiles/OtpComputing/confHeraklesSit.xml";
        fileNameStandSlice = "/statFiles/OtpComputing/confHerakles.xml";
        fileNameSitSlice = "/statFiles/OtpComputing/confHeraklesSit.xml";
    }
    else if (m_Human->getName().find("OLDDUDE")!= string::npos)
    {
        fileNameStand = "/statFiles/OtpComputing/confOldDude.xml";
        fileNameSit = "/statFiles/OtpComputing/confOldDudeSit.xml";
        fileNameStandSlice = "/statFiles/OtpComputing/confOldDude.xml";
        fileNameSitSlice = "/statFiles/OtpComputing/confOldDudeSit.xml";
    }
    else if (m_Human->getName().find("ACHILE")!= string::npos)
    {
        fileNameStand = "/statFiles/OtpComputing/confAchile.xml";
        fileNameSit = "/statFiles/OtpComputing/confAchileSit.xml";
        fileNameStandSlice = "/statFiles/OtpComputing/confAchileTranche.xml";
        fileNameSitSlice = "/statFiles/OtpComputing/confAchileTrancheSit.xml";
    }
    else
    {
        cout << "No configurations found" << endl;
    }




    fileNameStand = home + fileNameStand;
    fileNameSit = home + fileNameSit;
    fileNameStandSlice = home + fileNameStandSlice;
    fileNameSitSlice = home + fileNameSitSlice;


    loadConfsFromXML(fileNameStand,true, false) ;
    loadConfsFromXML(fileNameSit,false, false);

    loadConfsFromXML(fileNameStandSlice,true, true);
    loadConfsFromXML(fileNameSitSlice,false, true);

    initGrid();

    API_activeGrid = getPlanGrid();
    //    newComputeOTP();
    ENV.setBool(Env::drawGrid,gridDrawing);
    return true;
}

bool OTPMotionPl::getOtp(std::string humanName, Eigen::Vector3d &dockPos,
                         std::vector<SM_TRAJ>& smTraj,
                         configPt& handConf,bool isStanding, double objectNessecity)
{
    getInputs();
    std::vector<pair<double,double> > traj;
    //    PlanEnv->setBool(PlanParam::env_createTrajs,true);
    bool result = getOtp(humanName,dockPos,traj,handConf,isStanding,objectNessecity);
    //    PlanEnv->setBool(PlanParam::env_createTrajs,false);

    smTraj = m_smTrajs;
    if (smTraj.size() > 0)
    {
        smTraj.at(0).print();
    }
    //    smTraj[0].plot();
    return result;
}

bool OTPMotionPl::getOtp(std::string humanName, Eigen::Vector3d &dockPos,
                         std::vector<pair<double,double> >& traj,
                         configPt& handConf,bool isStanding, double objectNessecity)
{

    if (m_Human->getName().find(humanName) == string::npos)
    {
        changeHumanByName(humanName);
    }
    PlanEnv->setBool(PlanParam::env_trajNormal,false);
    PlanEnv->setBool(PlanParam::env_trajSoftMotion,true);
    PlanEnv->setBool(PlanParam::env_trajRos,false);
    getInputs();
    saveInitConf();
    PlanEnv->setBool(PlanParam::env_isStanding,isStanding);
    PlanEnv->setDouble(PlanParam::env_objectNessecity,objectNessecity);
    //InitMhpObjectTransfert();
    //    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getP3dRobotStruct()->baseJnt)->user_dof_equiv_nbr;

    dumpVar();
    bool res = newComputeOTP();

    if (!res)
    {
        cout << "No otp found" << endl;
        return false;
    }

    if (m_confList.empty())
    {
        cout << "no result found" << endl;
        return false;
    }

    int id = 0;
    for (unsigned int i = 1; i < m_confList.size(); i++)
    {
        if (m_confList.at(i).cost < m_confList.at(id).cost)
        {
            id = i;
        }
    }



    //    double dockingDist = 0.5;
    OutputConf conf = m_confList.at(id);
    if (!conf.humanConf && !conf.robotConf && conf.cost < numeric_limits<double>::max( ))
    {
        cout << "ERROR: configuration problems" << endl;
        return false;
    }


    for (unsigned int i = 0; i < conf.robotTraj.size();i++)
    {
        pair<double,double> p;
        p.first = conf.robotTraj.at(i)[0];
        p.second = conf.robotTraj.at(i)[1];
        traj.push_back(p);
    }


    handConf = conf.robotConf->getConfigStruct();
    handConf[11] = angle_limit_PI(handConf[11]);
    ENV.setBool(Env::isCostSpace,false);
    return true;


}

bool OTPMotionPl::getOtp(std::string humanName,
                         std::vector<std::vector<double> >& traj,
                         configPt& handConf,bool isStanding, double objectNessecity)
{

    if (m_Human->getName().find(humanName) == string::npos)
    {
        changeHumanByName(humanName);
    }
    PlanEnv->setBool(PlanParam::env_trajNormal,false);
    PlanEnv->setBool(PlanParam::env_trajSoftMotion,false);
    PlanEnv->setBool(PlanParam::env_trajRos,true);
    getInputs();
    saveInitConf();
    PlanEnv->setBool(PlanParam::env_isStanding,isStanding);
    PlanEnv->setDouble(PlanParam::env_objectNessecity,objectNessecity);
    //InitMhpObjectTransfert();
    //    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getP3dRobotStruct()->baseJnt)->user_dof_equiv_nbr;

    dumpVar();
    bool res = newComputeOTP();

    if (!res)
    {
        cout << "No otp found" << endl;
        return false;
    }

    if (m_confList.empty())
    {
        cout << "no result found" << endl;
        return false;
    }

    int id = 0;
    for (unsigned int i = 1; i < m_confList.size(); i++)
    {
        if (m_confList.at(i).cost < m_confList.at(id).cost)
        {
            id = i;
        }
    }



    //    double dockingDist = 0.5;
    OutputConf conf = m_confList.at(id);
    if (!conf.humanConf && !conf.robotConf && conf.cost < numeric_limits<double>::max( ))
    {
        cout << "ERROR: configuration problems" << endl;
        return false;
    }

    for (unsigned int i = 0; i < m_robotTraj3D.size();i++)
    {
        std::vector<double> p;
        p.push_back(m_robotTraj3D.at(i)[0]);
        p.push_back(m_robotTraj3D.at(i)[1]);
        p.push_back(m_robotTraj3D.at(i)[2]);
        traj.push_back(p);
    }


    handConf = conf.robotConf->getConfigStruct();
    handConf[11] = angle_limit_PI(handConf[11]);
    ENV.setBool(Env::isCostSpace,false);
    return true;


}

void OTPMotionPl::dumpCosts()
{

    std::vector<ConfigHR> vectConfs;
    string str;
    if (PlanEnv->getBool(PlanParam::env_drawSlice) || PlanEnv->getBool(PlanParam::env_useOrientedSlice))
    {
        if (PlanEnv->getBool(PlanParam::env_isStanding))
        {
            str = "standing human with slice only";
            vectConfs = m_configListSlice;
        }
        else
        {
            str = "sitting human with slice only";
            vectConfs = m_sittingConfigListSlice;
        }
    }
    else
    {
        if (PlanEnv->getBool(PlanParam::env_isStanding))
        {
            str = "standing human with entire grid";
            vectConfs = m_configList;
        }
        else
        {
            str = "sitting human with entire grid";
            vectConfs = m_sittingConfigList;
        }
    }

    cout << str << endl;
    for (unsigned int i = 0; i < vectConfs.size(); i++)
    {
        cout << vectConfs.at(i).getCost() << endl;
    }

}

bool OTPMotionPl::changeHumanByName(std::string humanName)
{
    if ((humanName.find("HERAKLES") != string::npos && m_Human->getName().find("HERAKLES") != string::npos)
        || (humanName.find("ACHILE") != string::npos && m_Human->getName().find("ACHILE") != string::npos))
        {
        if (humanName.find(m_Human->getName()) != string::npos)
        {
            cout << "The human name corresponds to the nam of the human robot used" << endl;
        }
        else
        {
            Robot* newHum = NULL;
            for (int i=0; i<XYZ_ENV->nr; i++)
            {
                string name(XYZ_ENV->robot[i]->name);
                if(name.find(humanName) != string::npos )
                {
                    newHum = new Robot(XYZ_ENV->robot[i]);
                    break;
                }
            }
            m_Human = newHum;
            if (m_2DGrid)
            {
                m_2DGrid->setHuman(newHum);
            }
            cout << "Human loaded with success" << endl;
        }

        return true;
    }

    return false;
}

Eigen::Vector3d OTPMotionPl::getHumanActualPos()
{
    Eigen::Vector3d pos;
    confPtr_t q_human_cur = m_Human->getCurrentPos();
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
    pos[0] = (*q_human_cur)[firstIndexOfHumanDof + 0];
    pos[1] = (*q_human_cur)[firstIndexOfHumanDof + 1];
    pos[2] = angle_limit_PI((*q_human_cur)[firstIndexOfHumanDof + 5]);

    return pos;
}

Eigen::Vector3d OTPMotionPl::getRobotActualPos()
{
    Eigen::Vector3d pos;
    confPtr_t q_robot_cur = _Robot->getCurrentPos();
    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getP3dRobotStruct()->baseJnt)->user_dof_equiv_nbr;
    pos[0] = (*q_robot_cur)[firstIndexOfRobotDof + 0];
    pos[1] = (*q_robot_cur)[firstIndexOfRobotDof + 1];
    pos[2] = angle_limit_PI((*q_robot_cur)[firstIndexOfRobotDof + 5]);

    return pos;
}

void OTPMotionPl::addVectorToRealTreajectory(Eigen::Vector2d vect)
{
    m_2DHumanRealPath.push_back(vect);
}

void OTPMotionPl::compteTraj(bool compute)
{
    PlanEnv->setBool(PlanParam::env_createTrajs,compute);
}


bool OTPMotionPl::isAlreadyTested(Eigen::Vector3d vect)
{
    for (unsigned int i = 0; i < m_testedVectors.size();i++)
    {
        if (fabs(m_testedVectors.at(i)[0] - vect[0]) < EPS1/2 &&
            fabs(m_testedVectors.at(i)[1] - vect[1]) < EPS1/2 &&
            fabs(m_testedVectors.at(i)[2] - vect[2]) < EPS2)
        {
            return true;
        }
    }
    m_testedVectors.push_back(vect);
    return false;
}

bool OTPMotionPl::testTrajectories(bool fullbody)
{

    ENV.setBool(Env::isCostSpace,false);

    m_ManipPl->setNavigationPlanner();

    p3d_multiLocalPath_disable_all_groupToPlan( _Robot->getP3dRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( _Robot->getP3dRobotStruct(), m_ManipPl->getUpBodyMLP(), 1, false);

    Move3D::Trajectory p3d_trajectory(_Robot);
    p3d_trajectory.clear();


    confPtr_t q1(new Configuration(_Robot));
    //base
    (*q1)[6] = 5.957;
    (*q1)[7] = -1.533;
    (*q1)[11] = -94.007*M_PI/180;
    //torso
    (*q1)[12] = 0.150*M_PI/180;
    //hear
    (*q1)[13] = -0.007*M_PI/180;
    (*q1)[14] = -0.696*M_PI/180;
    //rarm
    (*q1)[16] = 7.515*M_PI/180;
    (*q1)[17] = 39.404*M_PI/180;
    (*q1)[18] = -26.539*M_PI/180;
    (*q1)[19] = -112.598*M_PI/180;
    (*q1)[20] = -24.585*M_PI/180;
    (*q1)[21] = -100.717*M_PI/180;
    (*q1)[22] = -115.448*M_PI/180;
    //larm
    (*q1)[25] = 0.410*M_PI/180;
    (*q1)[26] = 74.067*M_PI/180;
    (*q1)[27] = 85.201*M_PI/180;
    (*q1)[28] = -85.549*M_PI/180;
    (*q1)[29] = 0.012*M_PI/180;
    (*q1)[30] = -4.943*M_PI/180;
    (*q1)[31] = 90.278*M_PI/180;


    confPtr_t q2(new Configuration(_Robot));
    //base
    (*q2)[6] = 5.957;
    (*q2)[7] = -4.362;
    (*q2)[11] = 90.000*M_PI/180;
    //torso
    (*q2)[12] = 0.150*M_PI/180;
    //hear
    (*q2)[13] = -0.007*M_PI/180;
    (*q2)[14] = -0.696*M_PI/180;
    //rarm
    (*q2)[16] = 7.515*M_PI/180;
    (*q2)[17] = 39.404*M_PI/180;
    (*q2)[18] = -26.539*M_PI/180;
    (*q2)[19] = -112.598*M_PI/180;
    (*q2)[20] = -24.585*M_PI/180;
    (*q2)[21] = -100.717*M_PI/180;
    (*q2)[22] = -115.448*M_PI/180;
    //larm
    (*q2)[25] = 0.410*M_PI/180;
    (*q2)[26] = 74.067*M_PI/180;
    (*q2)[27] = 85.201*M_PI/180;
    (*q2)[28] = -85.549*M_PI/180;
    (*q2)[29] = 0.012*M_PI/180;
    (*q2)[30] = -4.943*M_PI/180;
    (*q2)[31] = 90.278*M_PI/180;

    confPtr_t q3(new Configuration(_Robot));
    //base
    (*q3)[6] = 5.086;
    (*q3)[7] = -3.534;
    (*q3)[11] = -178.452*M_PI/180;
    //torso
    (*q3)[12] = 0.150*M_PI/180;
    //h3ear
    (*q3)[13] = -0.007*M_PI/180;
    (*q3)[14] = -0.696*M_PI/180;
    //rarm
    (*q3)[16] = -82.806*M_PI/180;
    (*q3)[17] = 52.973*M_PI/180;
    (*q3)[18] = -1.203*M_PI/180;
    (*q3)[19] = -49.303*M_PI/180;
    (*q3)[20] = -85.356*M_PI/180;
    (*q3)[21] = -33.344*M_PI/180;
    (*q3)[22] = -115.448*M_PI/180;
    //larm
    (*q3)[25] = 0.410*M_PI/180;
    (*q3)[26] = 74.067*M_PI/180;
    (*q3)[27] = 85.201*M_PI/180;
    (*q3)[28] = -85.549*M_PI/180;
    (*q3)[29] = 0.012*M_PI/180;
    (*q3)[30] = -4.943*M_PI/180;
    (*q3)[31] = 90.278*M_PI/180;


    if (fullbody)
    {
        p3d_trajectory.push_back(q2);
        p3d_trajectory.push_back(q3);
        p3d_trajectory.push_back(q1);
    }else
    {
        p3d_trajectory.push_back(q1);
        p3d_trajectory.push_back(q2);
    }

    bool trajTest = p3d_trajectory.replaceP3dTraj( _Robot->getP3dRobotStruct()->tcur );


    p3d_rob * robotPt =  _Robot->getP3dRobotStruct();
    ManipulationPlanner m_p(robotPt);

    MANPIPULATION_TRAJECTORY_CONF_STR conf;
    SM_TRAJ smTraj;

    int msg = m_p.computeSoftMotion( _Robot->getP3dRobotStruct()->tcur , conf, smTraj);
    m_smTrajs.clear();
    m_smTrajs.push_back(smTraj);

    if (!trajTest)
    {
        cout << "Fail: in replaceP3dTraj(), no traj found" <<endl;
        return false;
    }
    else if (msg != 0)
    {
        cout << "Fail: in computeSoftMotion(), no traj found" <<endl;
        return false;
    }
//    delete q1;delete q2; delete q3;

    ENV.setBool(Env::isCostSpace,true);
    return true;
}

bool OTPMotionPl::getSimplePath(double x, double y, double theta, vector<vector<double> >& path)
{
    Vector2i pos;
    pos[0] = x;
    pos[1] = y;
    EnvCell* cell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(pos));
    vector<EnvCell*> cellPath = cell->getRobotTraj();
    m_2DPath.clear();
    for (unsigned int j = 0; j < cellPath.size(); j++)
    {
        m_2DPath.push_back(cellPath.at(j)->getCenter());
    }
    m_2DPath = m_pts->smoothTrajectory(_Robot,m_2DPath);
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > path3d = m_pts->add3DimwithoutTrajChange(m_2DPath,_Robot,0.05);
    for (unsigned int i = 0; i < path3d.size(); i ++)
    {
        vector<double> p;
        p.push_back(path3d.at(i)[0]);
        p.push_back(path3d.at(i)[1]);
        p.push_back(path3d.at(i)[2]);
        path.push_back(p);

        cout << "points " << i << " is : ( " << p[0] << ", " << p[1] << ", " << p[2] << " )" << endl;
    }

    m_PathExist =true;
    ENV.setBool(Env::drawOTPTraj,true);
    return true;
}

bool OTPMotionPl::hasHumanMovedAccordingToPlan(double error)
{

    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
    confPtr_t q_human_cur = m_Human->getCurrentPos();

    Vector2d v;
    v[0] = (*q_human_cur)[firstIndexOfHumanDof + 0];
    v[1] = (*q_human_cur)[firstIndexOfHumanDof + 1];

    if (!m_OTPsuceed)
    {
        return false;
    }
    double dist;

    if (m_2DHumanPath.size() >0)
    {
        dist = m_pts->computeDistBetweenTrajAndPoint(m_2DHumanPath,v);
    }
    else
    {
        Vector2d vF;
        vF[0] =  (*m_confList.at(0).humanConf)[firstIndexOfHumanDof + 0];
        vF[1] =  (*m_confList.at(0).humanConf)[firstIndexOfHumanDof + 1];
        dist = sqrt(pow(vF[0] - v[0],2) + pow(vF[1] - v[1],2));
    }

    if (dist > error)
    {
        return false;
    }
    return true;

}


