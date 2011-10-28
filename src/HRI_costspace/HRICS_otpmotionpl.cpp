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
#include "HRICS_costspace.hpp"

#include "P3d-pkg.h"
#include "move3d-headless.h"
#include "Planner-pkg.h"

#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "planEnvironment.hpp"
#include "time.h"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

//extern HRICS::HumanAwareMotionPlanner*		HRICS_MotionPLConfig;

extern string global_ActiveRobotName;
extern API::TwoDGrid* API_activeRobotGrid;

extern Eigen::Vector3d current_WSPoint;
extern pair<double,Eigen::Vector3d > current_cost;

std::vector<Eigen::Vector3d> OTPList;
int ConfigHR::index = 0;


/**
 * configuration cost sorter
 */
class ConfigurationCost
{
public:

    bool operator()(ConfigHR first, ConfigHR second)
    {
        return ( first.getCost() < second.getCost() );
    }

} ConfigurationCostCompObject;

/**
 * outputconf sorter
 */
class OutputConfSort
{
public:

        bool operator()(OutputConf first, OutputConf second)
        {
                return ( first.cost < second.cost );
        }

} OutputConfSortObj;

OTPMotionPl::OTPMotionPl() : HumanAwareMotionPlanner() , m_PathExist(false) , m_HumanPathExist(false)
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
            m_Human = new Robot(XYZ_ENV->robot[i]);
            cout << "Humans is " << name << endl;
        }
    }

    initAll();
}

OTPMotionPl::OTPMotionPl(Robot* R, Robot* H) : HumanAwareMotionPlanner() , m_Human(H) , m_PathExist(false) , m_HumanPathExist(false)
{
    this->setRobot(R);
    initAll();
}

OTPMotionPl::~OTPMotionPl()
{
    delete m_2DGrid;
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
            cout << "Robot get struct " << _Robot->getRobotStruct() << endl;
    }

    m_DistanceSpace->parseHumans();

    // initVisibility
    m_VisibilitySpace = new Visibility(m_Human);

    //initReachability
    setReachability(new Natural());

    // init pos
    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
    m_Human->setInitialPosition(*q_human_cur);

    // Init Manipulation Planner
    m_ManipPl = new ManipulationPlanner( _Robot->getRobotStruct() );
    m_ManipPlHum = new ManipulationPlanner( m_Human->getRobotStruct() );

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
            cout << "\n\n cell: \n" << m_2DPath[i] << endl;
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
    initPR2GiveConf();
    cout << "OTPMotionPl::computePR2GIK()" << endl;

    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
//    double angle = (*q_human_cur)[11] - (*q_robot_cur)[11] + M_PI;
//    if (angle>M_PI)
//    {
//        angle -= 2*M_PI;
//    }

    int armId = 0;
    ArmManipulationData& armData = (*_Robot->getRobotStruct()->armManipulationData)[armId];
    ManipulationConfigs manipConf(_Robot->getRobotStruct());
    gpGrasp grasp;
    double confCost = -1;

    vector<vector<double> > vect;

    vector<double> target(6);
    target.at(0) = current_WSPoint(0);
    target.at(1) = current_WSPoint(1);
    target.at(2) = current_WSPoint(2);
    target.at(3) = M_PI/2;
    target.at(4) = (*q_robot_cur)[11] - 7*M_PI/8 + M_PI;
    target.at(5) = 0;

    vect.push_back(target);

    target.at(3) = M_PI/2;
    target.at(4) = (*q_robot_cur)[11] - M_PI/2 + M_PI;
    target.at(5) = 0;

    vect.push_back(target);

    target.at(3) = M_PI/2;
    target.at(4) = (*q_robot_cur)[11] - 7*M_PI/8 + M_PI;
    target.at(5) = P3D_HUGE;

    vect.push_back(target);

    target.at(3) = M_PI/2;
    target.at(4) = P3D_HUGE;
    target.at(5) = P3D_HUGE;

    vect.push_back(target);

    target.at(3) = P3D_HUGE;
    target.at(4) = P3D_HUGE;
    target.at(5) = P3D_HUGE;

    vect.push_back(target);

    double* q;
    for (unsigned int i = 0; i < vect.size(); i++)
    {
        q = manipConf.getFreeHoldingConf(NULL, armId, grasp, armData.getCcCntrt()->Tatt, confCost, vect.at(i), NULL);
        if (q != NULL)
        {
            break;
        }
    }

    _Robot->activateCcConstraint();
    if (q != NULL)
    {
        shared_ptr<Configuration> m_q = shared_ptr<Configuration>(
                                              new Configuration(_Robot,p3d_copy_config(_Robot->getRobotStruct(),q)));
        _Robot->setAndUpdate( *m_q );
        return true;
    }
    else
    {
        initPR2GiveConf();
        return false;
    }
}

void OTPMotionPl::initPR2GiveConf()
{
    if (PlanEnv->getBool(PlanParam::env_showText))
    {
        cout << "OTPMotionPl::initPR2GiveConf()" << endl;
    }


    shared_ptr<Configuration> q_cur = _Robot->getCurrentPos();

   configPt q;
   q = p3d_alloc_config(_Robot->getRobotStruct());


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
   q[20] = -187.00*M_PI/180;
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



   shared_ptr<Configuration> m_q = shared_ptr<Configuration>(
                                         new Configuration(_Robot,p3d_copy_config(_Robot->getRobotStruct(),q)));
   _Robot->setAndUpdate( *m_q );
   _Robot->setInitialPosition(*m_q );
}

void OTPMotionPl::addToList(Eigen::Vector3d WSPoint)
{
	Vector3d OTPHumanCenter;
	OTPHumanCenter[0] = WSPoint[0];
	OTPHumanCenter[1] = WSPoint[1];
	OTPHumanCenter[2] = WSPoint[2];

	m_OTPList.push_back(OTPHumanCenter);
}

void OTPMotionPl::setCurrentOTP(Eigen::Vector3d WSPoint)
{

	m_Human->setAndUpdate(*m_Human->getInitialPosition());
	int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
	shared_ptr<Configuration> q_cur = m_Human->getCurrentPos();


	current_WSPoint[0] = WSPoint[0] + (*q_cur)[firstIndexOfHumanDof + 0];
	current_WSPoint[1] = WSPoint[1] + (*q_cur)[firstIndexOfHumanDof + 1];
	current_WSPoint[2] = WSPoint[2] + (*q_cur)[firstIndexOfHumanDof + 2];

	shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
	shared_ptr<Configuration> q_robot_tmp = _Robot->getCurrentPos();
	int firstIndexOfDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

	(*q_robot_tmp)[firstIndexOfDof + 0] = 0;
	(*q_robot_tmp)[firstIndexOfDof + 1] = 0;
	_Robot->setAndUpdate(*q_robot_tmp);

//	m_ReachableSpace->computeIsReachableAndMove(current_WSPoint,false);
	_Robot->setAndUpdate(*q_robot_cur);
}

bool OTPMotionPl::placeRobot()
{
	int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
	shared_ptr<Configuration> q_cur = m_Human->getCurrentPos();
	shared_ptr<Configuration> q_tmp = m_Human->getCurrentPos();

	Vector2d HumanPos;

	HumanPos[0] = (*q_cur)[firstIndexOfHumanDof + 0];
	HumanPos[1] = (*q_cur)[firstIndexOfHumanDof + 1];

	double gaze = (*q_cur)[firstIndexOfHumanDof + 5];
	double refAngle = std::atan2(current_WSPoint[1] - HumanPos[1], current_WSPoint[0] - HumanPos[0]) - gaze;

	shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
	int firstIndexOfDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

	double dist = sqrt(pow(HumanPos[0] - current_WSPoint[0], 2) + pow(HumanPos[1] - current_WSPoint[1], 2));

	(*q_robot_cur)[firstIndexOfDof + 0] = cos(refAngle)*(0.9*dist + 0.17) + current_WSPoint[0];
	(*q_robot_cur)[firstIndexOfDof + 1] = sin(refAngle)*(0.9*dist + 0.17) + current_WSPoint[1];

	Vector2d point;
	point[0] = current_WSPoint[0];
	point[1] = current_WSPoint[1];

	Vector2d gazeDirect = HumanPos - point;

	(*q_robot_cur)[firstIndexOfDof + 5] = atan2(gazeDirect.y(),gazeDirect.x());

	if (PlanEnv->getBool(PlanParam::env_isStanding))
	{
		(*q_robot_cur)[firstIndexOfDof + 6] = 0.15;
		if ((current_WSPoint[2] >= 0.7 + (*q_cur)[firstIndexOfHumanDof + 2]) ||
			(current_WSPoint[2] >= 0.5 + (*q_cur)[firstIndexOfHumanDof + 2] && dist > 0.6 ))
		{
			(*q_robot_cur)[firstIndexOfDof + 6] = 0.3;
		}
	}
	else
	{
		(*q_robot_cur)[firstIndexOfDof + 6] = 0.0;
		if (current_WSPoint[2] > 0.4 + (*q_cur)[firstIndexOfHumanDof + 2])
		{
			(*q_robot_cur)[firstIndexOfDof + 6] = 0.15;
		}
	}
	_Robot->setAndUpdate(*q_robot_cur);
	m_Human->setAndUpdate(*m_Human->getInitialPosition());
	if (!ComputePR2Gik())
	{
//		initPR2GiveConf();
		return false;
	}

	initPR2GiveConf();



//	if (!ComputePR2Gik())
//	{
//		(*q_robot_cur)[firstIndexOfDof + 6] = 0.31;
//		_Robot->setAndUpdate(*q_robot_cur);
//		if (!ComputePR2Gik())
//		{
//			(*q_robot_cur)[firstIndexOfDof + 6] = 0;
//			_Robot->setAndUpdate(*q_robot_cur);
//			return false;
//		}

//	}

	m_Human->setAndUpdate(*q_cur);


	return true;
}

void OTPMotionPl::drawOTPList(bool value)
{
	if (value)
	{
		OTPList = m_OTPList;
	}
	else
	{
		OTPList.clear();
	}
}

void OTPMotionPl::placeHuman()
{
	shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
	_Robot->setAndUpdate(*_Robot->getInitialPosition());

	m_ReachableSpace->computeIsReachableAndMove(current_WSPoint,false);
	_Robot->setAndUpdate(*q_robot_cur);
}

std::vector<ConfigHR> OTPMotionPl::addConfToList()
{
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;


	shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
	shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();

	(*q_robot_cur)[firstIndexOfRobotDof + 0] = (*q_robot_cur)[firstIndexOfRobotDof + 0] - (*q_human_cur)[firstIndexOfHumanDof + 0];
	(*q_robot_cur)[firstIndexOfRobotDof + 1] = (*q_robot_cur)[firstIndexOfRobotDof + 1] - (*q_human_cur)[firstIndexOfHumanDof + 1];
	(*q_robot_cur)[firstIndexOfRobotDof + 5] = (*q_robot_cur)[firstIndexOfRobotDof + 5] - (*q_human_cur)[firstIndexOfHumanDof + 5];
	(*q_human_cur)[firstIndexOfHumanDof + 0] = 0;
	(*q_human_cur)[firstIndexOfHumanDof + 1] = 0;
	(*q_human_cur)[firstIndexOfHumanDof + 2] = 1.07;
	(*q_human_cur)[firstIndexOfHumanDof + 5] = 0;


	ConfigHR chr;
	chr.setHumanConf(m_Human, q_human_cur->getConfigStruct());
	chr.setRobotConf(_Robot, q_robot_cur->getConfigStruct());

	m_configList.push_back(chr);

	return m_configList;
}

void OTPMotionPl::removeLastConf()
{
	m_configList.pop_back();
	ConfigHR::index--;
}

void OTPMotionPl::clearConfList()
{
	m_configList.clear();
	ConfigHR::index = 0;
}

void OTPMotionPl::saveToXml(string filename)
{
    stringstream ss;
    string str;

    //Creating the file Variable version 1.0
    xmlDocPtr doc = xmlNewDoc(xmlCharStrdup("1.0"));


    //Writing the root node
    xmlNodePtr root = xmlNewNode (NULL, xmlCharStrdup("StoredConfiguration"));

    //nb node
    str.clear(); ss << m_configList.size(); ss >> str; ss.clear();
    xmlNewProp (root, xmlCharStrdup("nb_node"), xmlCharStrdup(str.c_str()));

//    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
//    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

    for (unsigned int i = 0; i< m_configList.size(); i++)
    {
        std::ostringstream oss;
        oss << i;
        string name = "node_" + oss.str();
        xmlNodePtr cur = xmlNewChild (root, NULL, xmlCharStrdup(name.c_str()), NULL);
        xmlNodePtr curHuman = xmlNewChild (cur, NULL, xmlCharStrdup("humanConfig"), NULL);

//        shared_ptr<Configuration> hq(new Configuration(m_Human,m_configList.at(i).getHumanConf()));
//        (*hq)[firstIndexOfHumanDof + 0] = 0;
//        (*hq)[firstIndexOfHumanDof + 1] = 0;
//        (*hq)[firstIndexOfHumanDof + 5] = 0;
        writeXmlRobotConfig(curHuman,m_Human->getRobotStruct(),m_configList.at(i).getHumanConf());

        xmlNodePtr curRobot = xmlNewChild (cur, NULL, xmlCharStrdup("robotConfig"), NULL);

//        shared_ptr<Configuration> rq(new Configuration(_Robot,m_configList.at(i).getRobotConf()));
//        (*rq)[firstIndexOfRobotDof + 0] -= (*hq)[firstIndexOfHumanDof + 0];
//        (*rq)[firstIndexOfRobotDof + 1] -= (*hq)[firstIndexOfHumanDof + 1];
//        (*rq)[firstIndexOfRobotDof + 5] -= (*hq)[firstIndexOfHumanDof + 5];
        writeXmlRobotConfig(curRobot,_Robot->getRobotStruct(),m_configList.at(i).getRobotConf());
    }


    xmlDocSetRootElement(doc, root);
    //	writeRootNode(graph, root);
    //	writeSpeGraph(graph, file, root);

    //Writing the file on HD
    xmlSaveFormatFile (filename.c_str(), doc, 1);
    xmlFreeDoc(doc);

    cout << "Writing Grid to : " << filename << endl;

}

int OTPMotionPl::loadConfsFromXML(string filename, bool isStanding, bool isSlice)
{
    if (isSlice)
    {
        if (isStanding)
        {
            m_configListSlice = loadFromXml(filename);
            sortConfigList(m_configListSlice.size(),isStanding,isSlice);
            return m_configListSlice.size();
        }
        else
        {
            m_sittingConfigListSlice = loadFromXml(filename);
            sortConfigList(m_sittingConfigListSlice.size(),isStanding,isSlice);
            return m_sittingConfigListSlice.size();
        }
    }
    else
    {
        if (isStanding)
        {
            m_configList = loadFromXml(filename);
            sortConfigList(m_configList.size(),isStanding,isSlice);
            return m_configList.size();
        }
        else
        {
            m_sittingConfigList = loadFromXml(filename);
            sortConfigList(m_sittingConfigList.size(),isStanding,isSlice);
            return m_sittingConfigList.size();
        }
    }
}

std::vector<ConfigHR> OTPMotionPl::loadFromXml(string filename)
{
    vector<ConfigHR> vectConfs;
    ConfigHR::index = 0;

    int nbNode = 0;

    xmlDocPtr doc;
    xmlNodePtr cur;
    xmlNodePtr root;

    doc = xmlParseFile(filename.c_str());

    if(doc==NULL)
    {
        cout << "Document not parsed successfully (doc==NULL)" << endl;
        return vectConfs;
    }

    root = xmlDocGetRootElement(doc);

    if (root == NULL)
    {
        cout << "Document not parsed successfully" << endl;
        xmlFreeDoc(doc);
        return vectConfs;
    }

	if (xmlStrcmp(root->name, xmlCharStrdup("StoredConfiguration")))
	{
		cout << "Document of the wrong type root node not StoredConfiguration" << endl;
		xmlFreeDoc(doc);
		return vectConfs;
	}

	xmlChar* tmp;

	tmp = xmlGetProp(root, xmlCharStrdup("nb_node"));

	sscanf((char *) tmp, "%d", &nbNode );
	xmlFree(tmp);

	/***************************************/
	cur = root->xmlChildrenNode->next;
	int i =0;

    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

	vectConfs.clear();
	while (i < nbNode)
	{
		if (cur == NULL)
		{
			cout << "Document error on the number of node" << endl;
			return vectConfs;
		}
		std::string name(reinterpret_cast<const char*>(cur->name));


		if (name.find("node") != string::npos )
		{
			i++;
			xmlNodePtr ptrTmp = cur->xmlChildrenNode->next;
			if (xmlStrcmp(ptrTmp->name, xmlCharStrdup("humanConfig")))
			{
				cout << "Error: no human config" << endl;
				return vectConfs;
			}

			configPt q_humTmp = readXmlConfig(m_Human->getRobotStruct(),ptrTmp->xmlChildrenNode->next);


			ptrTmp = ptrTmp->next->next;
			if (xmlStrcmp(ptrTmp->name, xmlCharStrdup("robotConfig")))
			{
				cout << "Error: no robot config" << endl;
				return vectConfs;
			}
			configPt q_rob =readXmlConfig(_Robot->getRobotStruct(),ptrTmp->xmlChildrenNode->next);

			ConfigHR chr;

			configPt q_hum = q_humTmp;

			q_rob[firstIndexOfRobotDof + 0] = q_rob[firstIndexOfRobotDof + 0] - q_hum[firstIndexOfHumanDof + 0];
			q_rob[firstIndexOfRobotDof + 1] = q_rob[firstIndexOfRobotDof + 1] - q_hum[firstIndexOfHumanDof + 1];
			q_rob[firstIndexOfRobotDof + 5] = q_rob[firstIndexOfRobotDof + 5] - q_hum[firstIndexOfHumanDof + 5];
			q_hum[firstIndexOfHumanDof + 0] = 0;
			q_hum[firstIndexOfHumanDof + 1] = 0;
//			q_hum[firstIndexOfHumanDof + 2] = 1.07;
			q_hum[firstIndexOfHumanDof + 5] = 0;

			chr.setHumanConf(m_Human, q_hum);
			chr.setRobotConf(_Robot, q_rob);

			vectConfs.push_back(chr);
		}



		cur = cur->next;
	}

	return vectConfs;


//	configPt readXmlConfig(p3d_rob *robot, xmlNodePtr cur)

}

pair<shared_ptr<Configuration>,shared_ptr<Configuration> > OTPMotionPl::setRobotsToConf(int id, bool isStanding)
{
	pair<shared_ptr<Configuration>,shared_ptr<Configuration> > resConf;
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
			int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

//			shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
			shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();

			shared_ptr<Configuration> ptrQ_human(new Configuration(m_Human,vectConfs.at(i).getHumanConf()));
			(*ptrQ_human)[firstIndexOfHumanDof + 0] = (*q_human_cur)[firstIndexOfHumanDof + 0];
			(*ptrQ_human)[firstIndexOfHumanDof + 1] = (*q_human_cur)[firstIndexOfHumanDof + 1];
			(*ptrQ_human)[firstIndexOfHumanDof + 5] = (*q_human_cur)[firstIndexOfHumanDof + 5];
			m_Human->setAndUpdate(*ptrQ_human);
			resConf.first = ptrQ_human;

			shared_ptr<Configuration> ptrQ_robot(new Configuration(_Robot,vectConfs.at(i).getRobotConf()));
			(*ptrQ_robot)[firstIndexOfRobotDof + 5] += (*q_human_cur)[firstIndexOfHumanDof + 5];
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

pair<shared_ptr<Configuration>,shared_ptr<Configuration> > OTPMotionPl::setRobotsToConf(int id, bool isStanding ,double x, double y, double Rz)
{
	int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
	shared_ptr<Configuration> ptrQ_human = m_Human->getCurrentPos();
	(*ptrQ_human)[firstIndexOfHumanDof + 0] = x;
	(*ptrQ_human)[firstIndexOfHumanDof + 1] = y;
	(*ptrQ_human)[firstIndexOfHumanDof + 5] = Rz;
	m_Human->setAndUpdate(*ptrQ_human);

	return setRobotsToConf(id, isStanding);
}

void OTPMotionPl::sortConfigList(double nbNode, bool isStanding, bool isSlice)
{
    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();

    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
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
        shared_ptr<Configuration> ptrQ_robot(new Configuration(_Robot,vectConfs.at(i).getRobotConf()));
        shared_ptr<Configuration> ptrQ_human(new Configuration(m_Human,vectConfs.at(i).getHumanConf()));
        double tmpDist = sqrt(pow((*ptrQ_robot)[firstIndexOfRobotDof + 0]-(*ptrQ_human)[firstIndexOfHumanDof + 0],2) +
                              pow((*ptrQ_robot)[firstIndexOfRobotDof + 1]-(*ptrQ_human)[firstIndexOfHumanDof + 1],2));
        if (tmpDist > maxDist)
        {
            maxDist = tmpDist;
        }
    }


    for (int i = 0; i < nbNode; i++)
    {
        shared_ptr<Configuration> ptrQ_robot(new Configuration(_Robot,vectConfs.at(i).getRobotConf()));
        _Robot->setAndUpdate(*ptrQ_robot);
        double xRob = (*ptrQ_robot)[firstIndexOfRobotDof + 0];
        double yRob = (*ptrQ_robot)[firstIndexOfRobotDof + 1];

		shared_ptr<Configuration> ptrQ_human(new Configuration(m_Human,vectConfs.at(i).getHumanConf()));
		m_Human->setAndUpdate(*ptrQ_human);
		double xHum = (*ptrQ_human)[firstIndexOfHumanDof + 0];
		double yHum = (*ptrQ_human)[firstIndexOfHumanDof + 1];
		double rzHum = (*ptrQ_human)[firstIndexOfHumanDof + 5];

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
    saveAllCostsToFile();
    m_multipliComputeCostVector.clear();
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
    cout << "Average time = " << totalTimeSum / nb << endl;
    cout << "Average init Grid time = " << initGridTimeSum / nb << endl;
    cout << "Average First conf computing time = " << firstTimeSum / nb << endl;
    cout << "Average loop time = " << loopTimeSum / nb << endl;
    cout << "Average cost = " << costSum / nb << endl;
    cout << "Average nb Iteration = " << (double)nbIteration / nb << endl;
    cout << "Average nb solutions = " << (double)nbSolution / nb << endl;
    cout << "Formated :" << endl << totalTimeSum / nb << endl << initGridTimeSum / nb << endl << loopTimeSum / nb << endl
                   << costSum / nb << endl << (double)nbIteration / nb << endl << (double)nbSolution / nb << endl << (double)nb/id << endl;
    cout << "-----------------------------" << endl;

    return totalTimeSum / nb;


}




void OTPMotionPl::setInputs( Eigen::Vector3d humanPos, Eigen::Vector3d robotPos,bool isStanding, double mobility)
{
    m_humanPos = humanPos;
    m_robotPos = robotPos;
    isStanding = m_isStanding;
    mobility = m_mobility;
}



void OTPMotionPl::getInputs()
{
    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
    if (PlanEnv->getBool(PlanParam::env_realTime))
    {
        Robot* visball = global_Project->getActiveScene()->getRobotByNameContaining("VISBALL");
        (*q_human_cur)[firstIndexOfHumanDof + 0] = (*visball->getCurrentPos())[6];
        (*q_human_cur)[firstIndexOfHumanDof + 1] = (*visball->getCurrentPos())[7];
        (*q_human_cur)[firstIndexOfHumanDof + 5] = (*visball->getCurrentPos())[11];
        m_Human->setAndUpdate(*q_human_cur);
        saveInitConf();
    }


    m_humanPos[0] = (*q_human_cur)[firstIndexOfHumanDof + 0];
    m_humanPos[1] = (*q_human_cur)[firstIndexOfHumanDof + 1];
    m_humanPos[2] = (*q_human_cur)[firstIndexOfHumanDof + 5];

    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
    m_robotPos[0] = (*q_robot_cur)[firstIndexOfRobotDof + 0];
    m_robotPos[1] = (*q_robot_cur)[firstIndexOfRobotDof + 1];
    m_robotPos[2] = (*q_robot_cur)[firstIndexOfRobotDof + 5];


    m_isStanding = PlanEnv->getBool(PlanParam::env_isStanding);
    m_mobility = PlanEnv->getDouble(PlanParam::env_objectNessecity);
}

void OTPMotionPl::setRobotPos()
{
    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
    m_robotPos[0] = (*q_robot_cur)[firstIndexOfRobotDof + 0];
    m_robotPos[1] = (*q_robot_cur)[firstIndexOfRobotDof + 1];
    m_robotPos[2] = (*q_robot_cur)[firstIndexOfRobotDof + 5];
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


Vector3d OTPMotionPl::getRandomPoints(double id)
{
    Vector3d vect;

    if (PlanEnv->getBool(PlanParam::env_normalRand))
    {
        shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
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
        shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
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
            cout << "ERROR: no human placement possible. Might result from human not specified very well. (in multiple human case)" << endl ;
            vect[0] = 0;
            vect[1] = 0;
            vect[2] = 0;
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
    else if (PlanEnv->getBool(PlanParam::env_useOldDude))
    {
        std::vector<std::pair<double,EnvCell*> > sortedCell = m_2DGrid->getSortedGrid();
        if (sortedCell.empty())
        {
            cout << "ERROR: no human placement possible. Might result from human not specified very well. (in multiple human case)" << endl ;
            vect[0] = 0;
            vect[1] = 0;
            vect[2] = 0;
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
    }

    return vect;
}

bool OTPMotionPl::newComputeOTP()
{
    clock_t start = clock();
//    getInputs();
//    m_2DGrid->init(computeHumanRobotDist());
//    m_2DGrid->initGrid(m_humanPos);
    if (!PlanEnv->getBool(PlanParam::env_normalRand) && ! PlanEnv->getBool(PlanParam::env_useAllGrid))
    {
        m_2DGrid->recomputeGridWhenHumanMove(m_humanPos);
    }
//    m_2DGrid->setCellsToblankCost();

    m_costVector.clear();
    clock_t gridInit = clock();

    bool isStanding = m_isStanding;
    double objectNecessity = m_mobility;
    bool m_showText = PlanEnv->getBool(PlanParam::env_showText);
    double timeLimitation = PlanEnv->getDouble(PlanParam::env_timeLimitation);
    double dumpTime = PlanEnv->getDouble(PlanParam::env_timeToDump);
    double currntDumpTime = 0;

    int maxIter = PlanEnv->getInt(PlanParam::env_maxIter); //300

    clearCostsfile();
    if (m_configList.empty() || m_sittingConfigList.empty())
    {
        cout << "No configuration lists" << endl;
        return false;
    }
    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();


    double x = m_humanPos[0];
    double y = m_humanPos[1];
    double Rz = m_humanPos[2];

    if (m_showText)
    {
        cout.clear(ios_base::goodbit);
        cout.flush();
        cout << "---------------------------------------------------" << endl;
        cout << "start OTP search" << endl <<"---------------------------------------------------" << endl;
    }
    OutputConf bestConf;


    m_confList.clear();

    clock_t varInit = clock();
    if (isStanding)
    {
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
        m_isInitSiting = true;
    }


//    cout << "init conf cost :"<< bestConf.cost << endl;
    int i = 0;
    int beginId = m_confList.size();
    int id = beginId;
    bestConf.id = id;
//    cout << "current id = "<< id << endl;

    m_Human->setAndUpdate(*q_human_cur);
    _Robot->setAndUpdate(*q_robot_cur);

//    m_2DGrid->setAsNotSorted();

    bool finished = true;
    if (!m_humanCanStand && m_isInitSiting)
    {
        finished = false;
    }

    clock_t firstConfs = clock();
    while (finished)
    {
        id++;
//        cout << "---------------------------------------------------" << endl;
//        cout << "new section, init pos : x = " << x << " y = " << y << " Rz = " << Rz << endl;
        Vector3d vect = getRandomPoints(id);
        double randomX = vect[0];
        double randomY = vect[1];
        double randomRz = vect[2];
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

        if (tmpConf.cost < bestConf.cost)
        {
            bestConf = tmpConf;
            i = 0;

        }
        else if (tmpConf.cost < numeric_limits<double>::max( )){ i++; }

//        cout << "current id = "<< id << endl;
//        cout << "current iteration worst than the best one : " << i << endl;
        if (i > maxIter + beginId || id > PlanEnv->getInt(PlanParam::env_totMaxIter) + beginId) { break; }

        int time = PlanEnv->getInt(PlanParam::env_timeShow)*1000;
        if (PlanEnv->getBool(PlanParam::env_drawOnlyBest) && (time > 0))
        {
            if (bestConf.cost < numeric_limits<double>::max( ))\
            {
                double xf = (*bestConf.humanConf)[firstIndexOfHumanDof + 0];
                double yf = (*bestConf.humanConf)[firstIndexOfHumanDof + 1];
                double Rzf = (*bestConf.humanConf)[firstIndexOfHumanDof + 5];
                setRobotsToConf(bestConf.configNumberInList,bestConf.isStandingInThisConf,xf,yf,Rzf);
                g3d_draw_allwin_active();
                usleep(time);
            }
        }


        m_Human->setAndUpdate(*q_human_cur);
        _Robot->setAndUpdate(*q_robot_cur);
        cout.clear(ios_base::goodbit);
        clock_t curTime = clock();
        if (((double)curTime - firstConfs) / CLOCKS_PER_SEC > timeLimitation) { break;}
        if (((double)curTime - firstConfs) / CLOCKS_PER_SEC > dumpTime+currntDumpTime)
        {
            m_costVector.push_back(bestConf.cost);
            currntDumpTime += dumpTime;
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
    }


    sort(m_confList.begin(),m_confList.end(),OutputConfSortObj);
    if (m_showText)
    {
        cout.clear(ios_base::goodbit);
        cout.flush();
        cout << "--------------------------------------" << endl << "End of OTP search" << endl << "--------------------------------------" << endl;
        cout << "Number of tested configuration = " << id << endl;
        cout << "Number of solutions found = " << m_confList.size() << endl;
        cout << "Success rate = " << (double)m_confList.size() / (double)id << endl;
    //    bestConf.humanConf->print();
    //    bestConf.robotConf->print();
    //    cout << "the choosed conf id is : " << bestConf.id << endl;
        cout << "the choosed configuration cost is : " << bestConf.cost << endl;
    }

    // trajectory

    m_2DHumanPath.clear();
    m_2DPath.clear();

    m_2DHumanPath = bestConf.humanTraj;
    m_HumanPathExist = bestConf.humanTrajExist;

    m_2DPath = bestConf.robotTraj;
    m_PathExist = bestConf.robotTrajExist;


//    double xf = (*bestConf.humanConf)[firstIndexOfHumanDof + 0];
//    double yf = (*bestConf.humanConf)[firstIndexOfHumanDof + 1];
//    double Rzf = (*bestConf.humanConf)[firstIndexOfHumanDof + 5];
//    setRobotsToConf(bestConf.configNumberInList,bestConf.isStandingInThisConf,xf,yf,Rzf);

    createTrajectoryFromOutputConf(bestConf);

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
        cout << "==========" << endl <<"time elapsed to : " << endl;
        cout << "init grid = " << tInitGrid << " s"<< endl;
        cout << "init variable = " << tFirst << " s"<< endl;
        cout << "look for the first conf = " << tFirst << " s"<< endl;
        cout << "pass the loop = " << ((double)endLoop - firstConfs) / CLOCKS_PER_SEC << " s"<< endl;
        cout << "end the function = " << ((double)end - endLoop) / CLOCKS_PER_SEC << " s"<< endl;
        cout << "----------" << endl << "Total time = " << m_time << " s"<< endl;
        cout << "==========" << endl;
    }
    showBestConf();
    m_costVector.clear();
    return true;
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

    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();

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


    for(unsigned int i = 0; i < vectConfs.size(); i++)
    {
        pair<shared_ptr<Configuration>,shared_ptr<Configuration> > conf = setRobotsToConf(i,true,x,y,Rz);

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
    double hRot = (*bestLocalConf.humanConf)[firstIndexOfHumanDof + 5] - m_humanPos[2];

    double mvCost = (psi*hDist + delta*fabs(hRot) )/(psi+delta);

//    cout << "moving cost = " << mvCost << endl;

    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

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

    bestLocalConf.robotTraj = rCell->getRobotVectorTraj();
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

    cell->addPoint(Rz);

    _Robot->setAndUpdate(*q_robot_cur);
    m_Human->setAndUpdate(*q_human_cur);

    return bestLocalConf;
}

OutputConf OTPMotionPl::findBestPosForHumanSitConf(double objectNecessity)
{

    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();

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
    shared_ptr<Configuration> q_hum = m_Human->getCurrentPos();

    m_2DGrid->setAsNotSorted();

    double elapsedTime;
    clock_t beginLoop = clock();
    for (int j = 0; j < nbSittingRot; j ++)
    {
        (*q_hum)[firstIndexOfHumanDof + 5] =  p3d_random(- M_PI , M_PI);
        m_Human->setAndUpdate(*q_hum);
        for(unsigned int i = 0; i < vectConfs.size(); i++)
        {
    //        cout << "---------------------------" << endl;
            pair<shared_ptr<Configuration>,shared_ptr<Configuration> > conf = setRobotsToConf(i,false);

            conf.first->setAsNotTested();
            conf.second->setAsNotTested();


//            string humanStr = testCol(true,false)?"is in collision":"is NOT in collision";
//            string robotStr = testCol(false,false)?"is in collision":"is NOT in collision";
    //        cout << "test colision : human " << humanStr << " and robot " << robotStr << endl;

            if (!testCol(true,false) && !testCol(false,true))
            {
                int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
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
                    vect.push_back(v);
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

    m_2DGrid->initGrid(m_humanPos);


    return bestConf;

}

bool  OTPMotionPl::testCol(bool isHuman, bool useConf)
{
    bool ret = true;
    if (isHuman)
    {
        if (useConf)
        {
            shared_ptr<Configuration> q(m_Human->getCurrentPos());
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
            shared_ptr<Configuration> q(_Robot->getCurrentPos());
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
        double Rzf = (*conf.humanConf)[firstIndexOfHumanDof + 5];
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

void OTPMotionPl::showBestConfRobOnly()
{
    shared_ptr<Configuration> ptrQ_human = m_Human->getCurrentPos();
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
        Robot* chair;
        for (int i=0; i<XYZ_ENV->nr; i++)
        {
            string name(XYZ_ENV->robot[i]->name);
            if(name.find("SIMPLECHAIR") != string::npos )
            {
                chair = new Robot(XYZ_ENV->robot[i]);
                break;
            }
        }
        shared_ptr<Configuration> q_chair_cur = chair->getCurrentPos();
        shared_ptr<Configuration> q_human = m_Human->getCurrentPos();

        cout << "make human stand" << endl;
        if(standUp())
        {
            cout << "human is standing\n" << endl;
        }
        clock_t start = clock();
        m_2DGrid->init(computeHumanRobotDist());
        clock_t second = clock();
        m_2DGrid->initGrid(m_humanPos);
        clock_t stop = clock();

        m_Human->setAndUpdate(*q_human);
        chair->setAndUpdate(*q_chair_cur);
        cout << "Human is sitting" << endl;

        cout << "initializing the grid take : " << ((double)second - start) / CLOCKS_PER_SEC << " s"<< endl;
        cout << "initializing the content of the grid take : " << ((double)stop - second) / CLOCKS_PER_SEC << " s"<< endl;


    }
    else
    {
        cout << "case: human is standing\n" << endl;
        clock_t start = clock();
        m_2DGrid->init(computeHumanRobotDist());
        clock_t second = clock();
        getInputs();
        m_2DGrid->initGrid(m_humanPos);
        clock_t stop = clock();

        cout << "initializing the grid take : " << ((double)second - start) / CLOCKS_PER_SEC << " s"<< endl;
        cout << "initializing the content of the grid take : " << ((double)stop - second) / CLOCKS_PER_SEC << " s"<< endl;
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


    for (unsigned int i = 0; i < maxSize; i++)
    {
        for (unsigned int costVectorId=0; costVectorId < m_multipliComputeCostVector.size(); costVectorId++)
        {
            double cost = m_multipliComputeCostVector.at(costVectorId).at(i);
            if (cost > 10)
            {
                cost = 2;
            }
            myfile << cost << " ";
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

void OTPMotionPl::createTrajectoryFromOutputConf(OutputConf conf)
{
    vector<shared_ptr<Configuration> > robotVectorConf;
    vector<shared_ptr<Configuration> > humanVectorConf;

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > robotTraj2D = conf.robotTraj;
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > humanTraj2D = conf.humanTraj;

    loadInitConf(true,true);
    initPR2GiveConf();

    shared_ptr<Configuration> q_cur_human(m_Human->getCurrentPos());
    shared_ptr<Configuration> q_cur_robot(_Robot->getCurrentPos());

    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();

    if (robotTraj2D.size() > 1)
    {
        for(int i =0; i < robotTraj2D.size() - 1; i++)
        {
            shared_ptr<Configuration> q_tmp(_Robot->getCurrentPos());

            (*q_tmp)[firstIndexOfRobotDof + 0] = robotTraj2D.at(i)[0];
            (*q_tmp)[firstIndexOfRobotDof + 1] = robotTraj2D.at(i)[1];
            (*q_tmp)[firstIndexOfRobotDof + 5] = atan2(
                    robotTraj2D.at(i+1)[1] - robotTraj2D.at(i)[1],
                    robotTraj2D.at(i+1)[0] - robotTraj2D.at(i)[0]);

            robotVectorConf.push_back(q_tmp);

        }
    }
    else
    {
        robotVectorConf.push_back(q_cur_robot);
    }

    robotVectorConf.push_back(conf.robotConf);

    if (humanTraj2D.size() > 1)
    {
        for(int i =0; i < humanTraj2D.size() - 1; i++)
        {
            shared_ptr<Configuration> q_tmp(m_Human->getCurrentPos());
            (*q_tmp)[firstIndexOfHumanDof + 0] = humanTraj2D.at (i)[0];
            (*q_tmp)[firstIndexOfHumanDof + 1] = humanTraj2D.at(i)[1];
            if (humanTraj2D.at(i+1)[0] != humanTraj2D.at(i)[0])
            {
                (*q_tmp)[firstIndexOfRobotDof + 5] = atan2(
                        humanTraj2D.at(i+1)[1] - humanTraj2D.at(i)[1],
                        humanTraj2D.at(i+1)[0] - humanTraj2D.at(i)[0]);
            }

            humanVectorConf.push_back(q_tmp);
            if (conf.isStandingInThisConf && m_humanCanStand)
            {
                standUp();
            }
        }
    }

    humanVectorConf.push_back(conf.humanConf);




    if (robotTraj2D.size() > 1)
    {
        p3d_multiLocalPath_disable_all_groupToPlan( _Robot->getRobotStruct() , false );
        p3d_multiLocalPath_set_groupToPlan( _Robot->getRobotStruct(), m_ManipPl->getBaseMLP(), 1, false);

        //////////////////
        API::Trajectory base_traj(robotVectorConf);
        base_traj.replaceP3dTraj();
    }


    if (humanTraj2D.size() > 1)
    {
        p3d_multiLocalPath_disable_all_groupToPlan( m_Human->getRobotStruct() , false );
        p3d_multiLocalPath_set_groupToPlan( m_Human->getRobotStruct(), m_ManipPlHum->getBaseMLP(), 1, false);

        //////////////////
        API::Trajectory base_hum_traj(humanVectorConf);
//        if (m_Human->getTrajStruct()->)
        base_hum_traj.replaceHumanP3dTraj(m_Human, m_Human->getTrajStruct());
    }
    else
    {
        if ( m_Human->getTrajStruct())
        {
            destroy_list_localpath(m_Human->getRobotStruct(), m_Human->getTrajStruct()->courbePt);
            m_Human->getRobotStruct()->tcur = NULL;
        }
    }

//    cout << base_traj.getRangeMax() << endl;
//    cout << base_traj.getNbOfPaths() << endl;

    //////////////////
//    API::Trajectory arm_traj;

//    shared_ptr<Configuration> qInit( base_traj.configAtParam( base_traj.getRangeMax()) );
//    configPt qConfInit =;
//   (*conf.robotConf)[11] = angle_limit_PI( (*conf.robotConf)[11] );
//   (*conf.robotConf)[22] = angle_limit_PI( (*conf.robotConf)[22] );

//    (*q_cur_robot)[6]  = (*conf.robotConf)[6];
//    (*q_cur_robot)[7]  = (*conf.robotConf)[7];
//    (*q_cur_robot)[11] = (*conf.robotConf)[11];

//    shared_ptr<Configuration> qGoal( conf.robotConf );

//    q_cur->print();
//    qGoal->print();

//    if( this->computeArmMotion(   q_cur->getConfigStruct(),
//                                  qGoal->getConfigStruct(),
//                                  arm_traj ) )
//    {
//        cout << "Concat traj ..." << endl;
//       base_traj.concat( arm_traj );
//    }

//	base_traj.replaceP3dTraj();

//    return base_traj;
}

bool OTPMotionPl::computeArmMotion(double* qInit, double* qGoal, API::Trajectory& traj)
{
	MANIPULATION_TASK_TYPE_STR type = ARM_FREE;

	bool succeed = false;

	std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
	std::vector <SM_TRAJ> smTrajs;
	std::vector <p3d_traj*> trajs;

	string OBJECT_NAME = "GREY_TAPE";

	vector<double> objGoto,objStart;

	objGoto.resize(6);
//    m_objGoto[0] = 4.23;
//    m_objGoto[1] = -2.22;
//    m_objGoto[2] = 1.00;

    objGoto[0] = P3D_HUGE;
    objGoto[1] = P3D_HUGE;
    objGoto[2] = P3D_HUGE;

    objGoto[3] = P3D_HUGE;
    objGoto[4] = P3D_HUGE;
    objGoto[5] = P3D_HUGE;

	cout << "Manipulation planning for " << OBJECT_NAME << endl;
	MANIPULATION_TASK_MESSAGE status;
	string str = m_ManipPl->robot()->name;

	if( str == "PR2_ROBOT" )
	{
		fixAllJointsWithoutArm(m_ManipPl->robot(),0);
	}

	switch ( (unsigned int) m_ManipPl->robot()->lpl_type )
	{
	case LINEAR :
		{
			gpGrasp grasp;
			status = m_ManipPl->armPlanTask(type,0,qInit,qGoal, objStart, objGoto,
												 /* m_OBJECT_NAME.c_str() */ "", "", (char*)"", grasp, trajs);

			if(status == MANIPULATION_TASK_OK )
			{
				m_ManipPl->robot()->tcur = p3d_create_traj_by_copy(trajs[0]);

				for(unsigned int i = 1; i < trajs.size(); i++){
					p3d_concat_traj(m_ManipPl->robot()->tcur, trajs[i]);
				}

				//m_manipulation->setRobotPath(m_manipulation->robot()->tcur);
			}
			break;
		}

	case MULTI_LOCALPATH : {
			gpGrasp grasp;
			status = m_ManipPl->armPlanTask(type,0,qInit,qGoal, objStart, objGoto,
							 /*m_OBJECT_NAME.c_str()*/ "", "", (char*)"", grasp, confs, smTrajs);
			break;
		}

	case SOFT_MOTION:{
			cout << "Manipulation : localpath softmotion should not be called" << endl;
			succeed = false;
			break;
		}
	}

	if (status != MANIPULATION_TASK_OK )
	{ succeed = false; }
	else
	{ succeed = true; }

	if( succeed )
	{
		traj = API::Trajectory( _Robot, m_ManipPl->robot()->tcur );
	}

	return succeed;
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
	shared_ptr<Configuration> q_human (m_Human->getCurrentPos());
	shared_ptr<Configuration> q_robot (_Robot->getCurrentPos());

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
	shared_ptr<Configuration> q_human (m_Human->getCurrentPos());
	shared_ptr<Configuration> q_robot (_Robot->getCurrentPos());

	int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
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


    Robot* humCyl;
    Robot* chair;
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

    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
    shared_ptr<Configuration> q_human = m_Human->getCurrentPos();
    double x = (*q_human)[firstIndexOfHumanDof + 0];
    double y = (*q_human)[firstIndexOfHumanDof + 1];
    double Rz = (*q_human)[firstIndexOfHumanDof + 5];

    shared_ptr<Configuration> q_chair_cur = chair->getCurrentPos();
    double xC = (*q_chair_cur)[firstIndexOfHumanDof + 0];
    double yC = (*q_chair_cur)[firstIndexOfHumanDof + 1];
    double RzC = (*q_chair_cur)[firstIndexOfHumanDof + 5];


    (*q_human)[firstIndexOfHumanDof + 0] = -3;
    (*q_human)[firstIndexOfHumanDof + 1] = 0;
    m_Human->setAndUpdate(*q_human);

    shared_ptr<Configuration> q_humCyl_cur = humCyl->getCurrentPos();
    shared_ptr<Configuration> q_humCyl = humCyl->getCurrentPos();

    shared_ptr<Configuration> q_chair = chair->getCurrentPos();

    double chairDist = 0;
    double standingDist = 0;
    double maxStandingDist = 0.7;
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
    cout << "normal rand = "  << PlanEnv->getBool(PlanParam::env_normalRand) << endl;
    cout << "oriented slice = "  << PlanEnv->getBool(PlanParam::env_useOrientedSlice) << endl;
    cout << "slices = "  << PlanEnv->getBool(PlanParam::env_useSlice) << endl;
    cout << "all grid = "  << PlanEnv->getBool(PlanParam::env_useAllGrid) << endl << endl;

	cout << " isStanding = " << PlanEnv->getBool(PlanParam::env_isStanding) << endl;
	cout << " objectNecessity = " <<PlanEnv->getDouble(PlanParam::env_objectNessecity) << endl;
	cout << " pow for random  = " << PlanEnv->getInt(PlanParam::env_pow) << endl;
	cout << " sleeping time  = " << PlanEnv->getInt(PlanParam::env_timeShow) << endl;
	cout << " maxIter = " << PlanEnv->getInt(PlanParam::env_maxIter)<< endl;
	cout << " totMaxIter = " <<PlanEnv->getInt(PlanParam::env_totMaxIter) << endl;
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
	cout << " Disconfort = " <<ENV.getDouble(Env::coeffConfo) << endl;

	cout << "------------ OTP variable end--------" <<endl;
}

void OTPMotionPl::setVar()
{
    PlanEnv->setBool(PlanParam::env_fusedGridRand,true);
    PlanEnv->setBool(PlanParam::env_normalRand,false);
    PlanEnv->setBool(PlanParam::env_useOrientedSlice,false);
    PlanEnv->setBool(PlanParam::env_useSlice,false);
    PlanEnv->setBool(PlanParam::env_useAllGrid,false);

    PlanEnv->setInt(PlanParam::env_timeShow,0);
    PlanEnv->setDouble(PlanParam::env_Cellsize,0.20);
    PlanEnv->setInt(PlanParam::env_nbSittingRotation,300);

	PlanEnv->setInt(PlanParam::env_pow,2);
	PlanEnv->setInt(PlanParam::env_timeShow,0);
	PlanEnv->setInt(PlanParam::env_maxIter,100);
	PlanEnv->setInt(PlanParam::env_totMaxIter,400);
	PlanEnv->setInt(PlanParam::env_nbRandomRotOnly,50);
	PlanEnv->setInt(PlanParam::env_nbSittingRotation,300);
	PlanEnv->setDouble(PlanParam::env_robotSpeed,1.0);
	PlanEnv->setDouble(PlanParam::env_humanSpeed,1);
	PlanEnv->setDouble(PlanParam::env_timeStamp,0.35);

	PlanEnv->setDouble(PlanParam::env_psi,0.99);
	PlanEnv->setDouble(PlanParam::env_delta,0.01);
	PlanEnv->setDouble(PlanParam::env_ksi,0.43);
	PlanEnv->setDouble(PlanParam::env_rho,0.57);
	PlanEnv->setDouble(PlanParam::env_sittingOffset,0.2);

    ENV.setDouble(Env::Kdistance,10.0);
    ENV.setDouble(Env::Kvisibility,35.0);
    ENV.setDouble(Env::Kreachable,50.0);

	ENV.setDouble(Env::coeffJoint,0.5);
	ENV.setDouble(Env::coeffEnerg,1);
	ENV.setDouble(Env::coeffConfo,10);

}

bool OTPMotionPl::InitMhpObjectTransfert(std::string humanName)
{
    if (PlanEnv->getBool(PlanParam::env_isInit))
    {
        cout << "The function is already initialized" << endl;
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
	ENV.setBool(Env::drawGrid,false);
	return true;
}

bool OTPMotionPl::getOtp(std::string humanName, Eigen::Vector3d &dockPos,
                         std::vector<pair<double,double> >& traj,
                         configPt& handConf,bool isStanding, double objectNessecity)
{

    PlanEnv->setBool(PlanParam::env_isStanding,isStanding);
    PlanEnv->setDouble(PlanParam::env_objectNessecity,objectNessecity);
    //InitMhpObjectTransfert();
    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

    dumpVar();
    newComputeOTP();

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



    double dockingDist = 0.5;
    OutputConf conf = m_confList.at(id);
    if (!conf.humanConf && !conf.robotConf && conf.cost < numeric_limits<double>::max( ))
    {
        cout << "ERROR: configuration problems" << endl;
        return false;
    }

    Vector2d pos;
    if (conf.robotTraj.size() > 0)
    {
        pos = conf.robotTraj.at(conf.robotTraj.size()-1);
    }
    else
    {
        return false;
        pos[0] = (*conf.robotConf)[firstIndexOfRobotDof + 0];
        pos[1] = (*conf.robotConf)[firstIndexOfRobotDof + 1];
    }

    double rot = angle_limit_PI((*conf.robotConf)[firstIndexOfRobotDof + 5]);


    dockPos[0] = pos[0]-( dockingDist * cos(rot));
    dockPos[1] = pos[1]-( dockingDist * sin(rot));
    dockPos[2] = rot;


    for (unsigned int i = 0; i < conf.robotTraj.size();i++)
    {
        pair<double,double> p;
        p.first = conf.robotTraj.at(i)[0];
        p.second = conf.robotTraj.at(i)[1];
        traj.push_back(p);
    }


    handConf = conf.robotConf->getConfigStruct();
    handConf[11] = angle_limit_PI(handConf[11]);

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
            Robot* newHum;
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
    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
    pos[0] = (*q_human_cur)[firstIndexOfHumanDof + 0];
    pos[1] = (*q_human_cur)[firstIndexOfHumanDof + 1];
    pos[2] = (*q_human_cur)[firstIndexOfHumanDof + 5];

    return pos;
}

Eigen::Vector3d OTPMotionPl::getRobotActualPos()
{
    Eigen::Vector3d pos;
    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
    pos[0] = (*q_robot_cur)[firstIndexOfRobotDof + 0];
    pos[1] = (*q_robot_cur)[firstIndexOfRobotDof + 1];
    pos[2] = (*q_robot_cur)[firstIndexOfRobotDof + 5];

    return pos;
}


void OTPMotionPl::addVectorToRealTreajectory(Eigen::Vector2d vect)
{
//    cout << "saving to list : x = " << vect[0] << " y = " << vect[1] << endl;
    m_2DHumanRealPath.push_back(vect);
}



/**
 * show trajectory of both robot and human
 */
void g3d_show_tcur_both_rob(p3d_rob *robotPt, int (*fct)(p3d_rob* robot, p3d_localpath* curLp),
                            p3d_rob *hum_robotPt, int (*hum_fct)(p3d_rob* hum_robot, p3d_localpath* hum_curLp))
{

  if (robotPt->tcur == NULL)
  {
    PrintInfo(("g3d_show_tcur_both_rob : no current trajectory\n"));
    return;
  }

  if (hum_robotPt->tcur == NULL)
  {
      PrintInfo(("g3d_show_tcur_both_rob : no trajectoryfor human\n"));
  }

  G3D_Window *win;
//  configPt q;
  configPt q_rob;
  configPt q_hum;
  pp3d_localpath localpathPt;
  pp3d_localpath humLocalpathPt;

  int end = FALSE;
//  int njnt = robotPt->njoints;
  double u = 0, du, umax; /* parameters along the local path */
  double uHum = 0, duHum, umaxHum; /* parameters along the local path */

//  robotPt->draw_transparent = false;

  win = g3d_get_cur_win();
//  win->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;
  //g3d_draw_env_box();
//  g3d_draw_obstacles(win);
#ifdef P3D_PLANNER
  if(XYZ_GRAPH && ENV.getBool(Env::drawGraph)){g3d_draw_graph();}
#endif

  umax = p3d_compute_traj_rangeparam(robotPt->tcur);
  umaxHum = p3d_compute_traj_rangeparam(hum_robotPt->tcur);

  localpathPt = robotPt->tcur->courbePt;
  humLocalpathPt = hum_robotPt->tcur->courbePt;

  int loopOut = 0;
  double robotSpeed = PlanEnv->getDouble(PlanParam::env_robotSpeed);
  double humanSpeed = PlanEnv->getDouble(PlanParam::env_humanSpeed);
  clock_t start = clock();
  while (loopOut == 0)
  {
    /* position of the robot corresponding to parameter u */
    if (u < umax - EPS6)
    {
        q_rob = p3d_config_at_param_along_traj(robotPt->tcur,u);
        p3d_set_and_update_this_robot_conf_multisol(robotPt, q_rob, NULL, 0, localpathPt->ikSol);
//        p3d_set_and_update_robot_conf(q_rob);
    }
    if (uHum < umaxHum - EPS6)
    {
        q_hum = p3d_config_at_param_along_traj(hum_robotPt->tcur,uHum);
        p3d_set_and_update_this_robot_conf_multisol(hum_robotPt, q_hum, NULL, 0, humLocalpathPt->ikSol);
//        p3d_set_and_update_robot_conf(q_hum);
    }

    g3d_draw_allwin_active();
    if (fct) if (((*fct)(robotPt, localpathPt)) == FALSE) return;
    if (hum_fct) if (((*hum_fct)(hum_robotPt, humLocalpathPt)) == FALSE) return;

    //((double)gridInit - start) / CLOCKS_PER_SEC
    if (u < umax - EPS6)
    {
        du = ENV.getDouble(Env::showTrajFPS)*(robotSpeed * ((double)clock() - start) / CLOCKS_PER_SEC);/* localpathPt->stay_within_dist(robotPt, localpathPt,*/
    }

    if (uHum < umaxHum - EPS6)
    {
        duHum = ENV.getDouble(Env::showTrajFPS)*(humanSpeed * ((double)clock() - start) / CLOCKS_PER_SEC);/* localpathPt->stay_within_dist(robotPt, localpathPt,*/
    }

      u = du;
      if (u > umax - EPS6) {
        u = umax;
        end = TRUE;
      }

      uHum = duHum;
      if (uHum > umaxHum - EPS6) {
        uHum = umaxHum;
        end = TRUE;
      }
      if (uHum > umaxHum - EPS6 && u > umax - EPS6)
      {
          loopOut++;
      }
  }
}


void ConfigHR::setHumanConf(Robot* human, configPt q)
{
	q_hum = p3d_copy_config(human->getRobotStruct(),q);
}

void ConfigHR::setRobotConf(Robot* robot, configPt q)
{
	q_rob = p3d_copy_config(robot->getRobotStruct(),q);
}

void OutputConf::clearAll()
{
//    humanConf = NULL;
    humanTraj.clear();
    humanTrajExist = false;

//    robotConf = NULL;
    robotTraj.clear();
    robotTrajExist = false;

    cost = numeric_limits<double>::max( );
    configNumberInList = -1;
    id = -1;
    isStandingInThisConf = true; }

