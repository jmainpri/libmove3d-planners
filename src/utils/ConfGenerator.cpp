#include "ConfGenerator.h"



#include "P3d-pkg.h"
#include "move3d-headless.h"
#include "Planner-pkg.h"

#include "planEnvironment.hpp"
#include "plannerFunctions.hpp"
#include "OtpUtils.hpp"

using namespace std;
using namespace tr1;

//using namespace HRICS;

// import most common Eigen types
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;
extern Eigen::Vector3d current_WSPoint;

std::vector<Eigen::Vector3d> OTPList;

ConfGenerator::ConfGenerator()
{
}

ConfGenerator::ConfGenerator(Robot* rob,Robot* human):_robot(rob),_human(human)
{
}

bool ConfGenerator::initialize( string filename, HRICS::Natural* reachableSpace )
{
  bool isStanding = true;
  bool isSlice = true;
  
  m_configList = loadFromXml(filename);
  sortConfig( m_configList, m_configList.size(), isStanding, isSlice, reachableSpace );
  return true;
}

bool ConfGenerator::computeRobotIkForGrabing( configPt& q )
{
  return computeRobotIkForGrabing( q, current_WSPoint );
}

bool ConfGenerator::computeRobotIkForGrabing( configPt &q, const Eigen::Vector3d& point)
{
  confPtr_t q_robot_cur = _robot->getCurrentPos();
  
  int armId = 0;
  ArmManipulationData& armData = (*_robot->getRobotStruct()->armManipulationData)[armId];
  ManipulationConfigs manipConf(_robot->getRobotStruct());
  gpGrasp grasp;
  double confCost = -1;
  
  vector<vector<double> > vect;
  
  vector<double> target(6);
  target.at(0) = point(0);
  target.at(1) = point(1);
  target.at(2) = point(2);
  
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
  
  for (unsigned int i = 0; i < vect.size(); i++)
  {
    q = manipConf.getFreeHoldingConf(NULL, armId, grasp, armData.getCcCntrt()->Tatt, confCost, vect.at(i), NULL);
    if (q != NULL)
    {
      break;
    }
  }
  _robot->activateCcConstraint();
  
  if (q != NULL)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void ConfGenerator::addToList(Eigen::Vector3d WSPoint)
{
  Vector3d OTPHumanCenter;
  OTPHumanCenter[0] = WSPoint[0];
  OTPHumanCenter[1] = WSPoint[1];
  OTPHumanCenter[2] = WSPoint[2];
  
  m_OTPList.push_back(OTPHumanCenter);
}

Eigen::Vector3d ConfGenerator::setCurOTP( Eigen::Vector3d WSPoint)
{
  
  //        r->setAndUpdate(*r->getInitialPosition());
  int firstIndexOfHumanDof = _robot->getJoint(1)->getIndexOfFirstDof();
  confPtr_t q_cur = _human->getCurrentPos();
  
  Eigen::Vector3d vec;
  current_WSPoint[0] = WSPoint[0] + (*q_cur)[firstIndexOfHumanDof + 0];
  current_WSPoint[1] = WSPoint[1] + (*q_cur)[firstIndexOfHumanDof + 1];
  current_WSPoint[2] = WSPoint[2] + (*q_cur)[firstIndexOfHumanDof + 2];
  
  return current_WSPoint;
}

void ConfGenerator::drawOTPList(bool value)
{
  OTPList.clear();
  if (value)
  {
    OTPList = m_OTPList;
  }
}

bool ConfGenerator::computeHandoverConfigFromList( pair<confPtr_t,confPtr_t>& best_handover_conf, double& best_cost )
{
  best_cost = numeric_limits<double>::max();
  
  for(int i=0; i<int(m_configList.size()); i++)
  {
    pair<confPtr_t,confPtr_t> conf = setRobotsToConf( m_configList[i] );
    
//    conf.first->setAsNotTested();
//    conf.second->setAsNotTested();
    
    _human->setAndUpdate(*conf.first);
    _robot->setAndUpdate(*conf.second);
    
    if ( (!_human->isInCollisionWithOthersAndEnv()) && (!conf.second->isInCollision()) )
    {
      best_handover_conf = conf;
      best_cost = m_configList[i].getCost();
      // cout << "the choosed configuration in the configuration list is : " << i << endl;
      return true;
    }
  }
  return false;
}

pair<confPtr_t,confPtr_t> ConfGenerator::setRobotsToConf( const HRICS::ConfigHR& conf )
{
  pair<confPtr_t,confPtr_t> resConf;
  vector<HRICS::ConfigHR> vectConfs;
  
  int firstIndexOfHumanDof = _human->getJoint("Pelvis")->getIndexOfFirstDof();
  int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
  
  // First is human
  confPtr_t q_human_cur = _human->getCurrentPos();
  confPtr_t ptrQ_human(new Configuration( _human, conf.getHumanConf() ));
  (*ptrQ_human)[firstIndexOfHumanDof + 0] = (*q_human_cur)[firstIndexOfHumanDof + 0];
  (*ptrQ_human)[firstIndexOfHumanDof + 1] = (*q_human_cur)[firstIndexOfHumanDof + 1];
  (*ptrQ_human)[firstIndexOfHumanDof + 5] = angle_limit_PI((*q_human_cur)[firstIndexOfHumanDof + 5]);
  _human->setAndUpdate(*ptrQ_human);
  resConf.first = ptrQ_human;
  
  // Second is robot
  // confPtr_t q_robot_cur = _Robot->getCurrentPos();
  confPtr_t ptrQ_robot(new Configuration( _robot, conf.getRobotConf() ));
  (*ptrQ_robot)[firstIndexOfRobotDof + 5] = angle_limit_PI((*ptrQ_robot)[firstIndexOfRobotDof + 5] + (*q_human_cur)[firstIndexOfHumanDof + 5]);
  double dist = sqrt(pow( (*ptrQ_robot)[firstIndexOfRobotDof + 0], 2) + pow( (*ptrQ_robot)[firstIndexOfRobotDof + 1], 2));
  (*ptrQ_robot)[firstIndexOfRobotDof + 0] = -cos((*ptrQ_robot)[firstIndexOfRobotDof + 5])*dist + (*ptrQ_human)[firstIndexOfHumanDof + 0];
  (*ptrQ_robot)[firstIndexOfRobotDof + 1] = -sin((*ptrQ_robot)[firstIndexOfRobotDof + 5])*dist + (*ptrQ_human)[firstIndexOfHumanDof + 1];
  _robot->setAndUpdate(*ptrQ_robot);
  resConf.second = ptrQ_robot;
  
  return resConf;
}

bool ConfGenerator::sortConfig(std::vector<HRICS::ConfigHR>& vectConfs, int nbNode, bool isStanding, bool isSlice, HRICS::Natural* reachableSpace)
{
  confPtr_t q_robot_cur = _robot->getCurrentPos();
  confPtr_t q_human_cur = _human->getCurrentPos();
  
  int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
  int firstIndexOfHumanDof = _human->getJoint("Pelvis")->getIndexOfFirstDof();
  
  double reachFactor = ENV.getDouble(Env::Kreachable);
  double distFactor = ENV.getDouble(Env::Kdistance);
  double visFactor = ENV.getDouble(Env::Kvisibility);
  
  double maxDist = 0;
  for (int i = 0; i < nbNode; i++)
  {
    confPtr_t ptrQ_robot(new Configuration(_robot,vectConfs[i].getRobotConf()));
    confPtr_t ptrQ_human(new Configuration(_human,vectConfs[i].getHumanConf()));
    double tmpDist = sqrt(pow((*ptrQ_robot)[firstIndexOfRobotDof + 0]-(*ptrQ_human)[firstIndexOfHumanDof + 0],2) +
                          pow((*ptrQ_robot)[firstIndexOfRobotDof + 1]-(*ptrQ_human)[firstIndexOfHumanDof + 1],2));
    if (tmpDist > maxDist)
    {
      maxDist = tmpDist;
    }
  }
  
  for (int i = 0; i < nbNode; i++)
  {
    confPtr_t ptrQ_robot(new Configuration(_robot,vectConfs.at(i).getRobotConf()));
    _robot->setAndUpdate(*ptrQ_robot);
    double xRob = (*ptrQ_robot)[firstIndexOfRobotDof + 0];
    double yRob = (*ptrQ_robot)[firstIndexOfRobotDof + 1];
    
    confPtr_t ptrQ_human(new Configuration(_human,vectConfs.at(i).getHumanConf()));
    _human->setAndUpdate(*ptrQ_human);
    double xHum = (*ptrQ_human)[firstIndexOfHumanDof + 0];
    double yHum = (*ptrQ_human)[firstIndexOfHumanDof + 1];
    double rzHum = angle_limit_PI((*ptrQ_human)[firstIndexOfHumanDof + 5]);
    
    double reachCost = reachableSpace->getConfigCost() * reachFactor;
    double distCost = (1 - sqrt(pow(xRob-xHum,2) + pow(yRob-yHum,2))/maxDist) * distFactor;
    double visCost = rzHum - atan2(yRob-yHum,xRob-xHum);
    if (visCost < -M_PI)
    {
      visCost += 2 * M_PI;
    }
    visCost = fabs(visCost/M_PI) * visFactor;
    double cost = (reachCost + distCost + visCost) / (reachFactor + distFactor + visFactor);
    vectConfs[i].setCost(cost);
  }
  
  _robot->setAndUpdate(*q_robot_cur);
  _human->setAndUpdate(*q_human_cur);
  
  HRICS::ConfigurationCost ConfigurationCostCompObject;
  
  sort(vectConfs.begin(),vectConfs.end(),ConfigurationCostCompObject);
  
  for (int i = 0; i < nbNode; i++)
  {
    vectConfs[i].setId(i);
  }
  
  return true;
}

//void ConfGenerator::drawHandoverList(bool value)
//{
//  OTPList.clear();
//  if (value)
//  {
//    OTPList = m_OTPList;
//  }
//}

bool ConfGenerator::placeRobot()
{
  int firstIndexOfHumanDof = _human->getJoint(1)->getIndexOfFirstDof();
  confPtr_t q_cur = _human->getCurrentPos();
  //        confPtr_t q_tmp = human->getCurrentPos();
  
  Vector2d HumanPos;
  
  HumanPos[0] = (*q_cur)[firstIndexOfHumanDof + 0];
  HumanPos[1] = (*q_cur)[firstIndexOfHumanDof + 1];
  
  double gaze = angle_limit_PI((*q_cur)[firstIndexOfHumanDof + 5]);
  double refAngle = std::atan2(current_WSPoint[1] - HumanPos[1], current_WSPoint[0] - HumanPos[0]) - gaze;
  
  confPtr_t q_robot = _robot->getCurrentPos();
  confPtr_t q_robot_cur = _robot->getCurrentPos();
  int firstIndexOfDof = dynamic_cast<p3d_jnt*>(_robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
  
  double dist = sqrt(pow(HumanPos[0] - current_WSPoint[0], 2) + pow(HumanPos[1] - current_WSPoint[1], 2));
  
  (*q_robot_cur)[firstIndexOfDof + 0] = cos(refAngle)*(0.9*dist + 0.17) + current_WSPoint[0];
  (*q_robot_cur)[firstIndexOfDof + 1] = sin(refAngle)*(0.9*dist + 0.17) + current_WSPoint[1];
  
  Vector2d point;
  point[0] = current_WSPoint[0];
  point[1] = current_WSPoint[1];
  
  Vector2d gazeDirect = HumanPos - point;
  
  (*q_robot_cur)[firstIndexOfDof + 5] = angle_limit_PI(atan2(gazeDirect.y(),gazeDirect.x()));
  
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
  
  _robot->setAndUpdate(*q_robot_cur);
  configPt q;
  if (!computeRobotIkForGrabing(q))
  {
    _robot->setAndUpdate(*q_robot);
    return false;
  }
  
  confPtr_t m_q = confPtr_t(
                            new Configuration(_robot,p3d_copy_config(_robot->getRobotStruct(),q)));
  _robot->setAndUpdate( *m_q );
  return true;
}

std::vector<HRICS::ConfigHR> ConfGenerator::addConfToList()
{
  int firstIndexOfHumanDof = _human->getJoint(1)->getIndexOfFirstDof();
  int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
  
  
  confPtr_t q_robot_cur = _robot->getCurrentPos();
  confPtr_t q_human_cur = _human->getCurrentPos();
  
  (*q_robot_cur)[firstIndexOfRobotDof + 0] = (*q_robot_cur)[firstIndexOfRobotDof + 0] - (*q_human_cur)[firstIndexOfHumanDof + 0];
  (*q_robot_cur)[firstIndexOfRobotDof + 1] = (*q_robot_cur)[firstIndexOfRobotDof + 1] - (*q_human_cur)[firstIndexOfHumanDof + 1];
  (*q_robot_cur)[firstIndexOfRobotDof + 5] = angle_limit_PI((*q_robot_cur)[firstIndexOfRobotDof + 5] - (*q_human_cur)[firstIndexOfHumanDof + 5]);
  
  (*q_human_cur)[firstIndexOfHumanDof + 0] = 0;
  (*q_human_cur)[firstIndexOfHumanDof + 1] = 0;
  (*q_human_cur)[firstIndexOfHumanDof + 5] = 0;
  
  
  HRICS::ConfigHR chr;
  chr.setHumanConf(_human, q_human_cur->getConfigStruct());
  chr.setRobotConf(_robot, q_robot_cur->getConfigStruct());
  
  m_configList.push_back(chr);
  
  return m_configList;
}

void ConfGenerator::removeLastConf()
{
  m_configList.pop_back();
  HRICS::ConfigHR::index--;
}

void ConfGenerator::clearConfList()
{
  m_configList.clear();
  HRICS::ConfigHR::index = 0;
}

void ConfGenerator::saveToXml(string filename)
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
  
  for (unsigned int i = 0; i< m_configList.size(); i++)
  {
    std::ostringstream oss;
    oss << i;
    string name = "node_" + oss.str();
    xmlNodePtr cur = xmlNewChild (root, NULL, xmlCharStrdup(name.c_str()), NULL);
    xmlNodePtr curHuman = xmlNewChild (cur, NULL, xmlCharStrdup("humanConfig"), NULL);
    
    writeXmlRobotConfig(curHuman,_human->getRobotStruct(),m_configList.at(i).getHumanConf());
    
    xmlNodePtr curRobot = xmlNewChild (cur, NULL, xmlCharStrdup("robotConfig"), NULL);
    
    writeXmlRobotConfig(curRobot,_robot->getRobotStruct(),m_configList.at(i).getRobotConf());
  }
  
  xmlDocSetRootElement(doc, root);
  
  //Writing the file on HD
  xmlSaveFormatFile (filename.c_str(), doc, 1);
  xmlFreeDoc(doc);
  
  cout << "Writing OTPConfList to : " << filename << endl;
}

std::vector<HRICS::ConfigHR> ConfGenerator::loadFromXml(string filename)
{
  vector<HRICS::ConfigHR> vectConfs;
  HRICS::ConfigHR::index = 0;
  
  //    confPtr_t q_robot_cur = _Robot->getCurrentPos();
  
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
  
  int firstIndexOfHumanDof = _human->getJoint(1)->getIndexOfFirstDof();
  int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
  
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
      
      configPt q_hum = readXmlConfig(_human->getRobotStruct(),ptrTmp->xmlChildrenNode->next);
      
      
      ptrTmp = ptrTmp->next->next;
      if (xmlStrcmp(ptrTmp->name, xmlCharStrdup("robotConfig")))
      {
        cout << "Error: no robot config" << endl;
        return vectConfs;
      }
      configPt q_rob =readXmlConfig(_robot->getRobotStruct(),ptrTmp->xmlChildrenNode->next);
      
      
      
      //            configPt q_hum = q_humTmp;
      
      q_rob[firstIndexOfRobotDof + 0] = q_rob[firstIndexOfRobotDof + 0] - q_hum[firstIndexOfHumanDof + 0];
      q_rob[firstIndexOfRobotDof + 1] = q_rob[firstIndexOfRobotDof + 1] - q_hum[firstIndexOfHumanDof + 1];
      q_rob[firstIndexOfRobotDof + 5] = angle_limit_PI(q_rob[firstIndexOfRobotDof + 5] - q_hum[firstIndexOfHumanDof + 5]);
      
      q_hum[firstIndexOfHumanDof + 0] = 0;
      q_hum[firstIndexOfHumanDof + 1] = 0;
      q_hum[firstIndexOfHumanDof + 5] = 0;
      
      for (unsigned int k = firstIndexOfHumanDof + 6; k < _robot->getNumberOfActiveDoF(); k++)
      {
        q_rob[k] = angle_limit_PI(q_rob[k]);
      }
      
      for (unsigned int k = 34; k < _robot->getNumberOfActiveDoF(); k++)
      {
        q_rob[k] = 0;
      }
      
      HRICS::ConfigHR chr;
      chr.setHumanConf(_human, q_hum);
      chr.setRobotConf(_robot, q_rob);
      
      vectConfs.push_back(chr);
    }
    
    cur = cur->next;
  }
  
  return vectConfs;
}


//########################################
//new version :
//########################################

vector<pair<double,double> > findOverlapingRanges(vector<pair<double,double> > overlapped)
{

}


bool ConfGenerator::addConstraintToList(constrainedLink CL)
{
    linkList.push_back(CL);



    if(linkListFiltred.count(CL.R->getName()) > 0 )
    {
        linkListFiltred.at(CL.R->getName());
    }
    else
    {

        linkListFiltred[CL.R->getName()] = CL;
    }

}


bool ConfGenerator::generateConfiguration(int nbConfs)
{
    int nbConfsFound = 0;

    Robot* table = global_Project->getActiveScene()->getRobotByNameContaining("TABLE_4");

    vector<vector<double> > rot;
    vector<double> trans;
    trans.push_back(0);
    trans.push_back(0);
    trans.push_back(0);
    rot.push_back(trans);
    rot.push_back(trans);
    rot.push_back(trans);


    constrainedLink CL(table,rot,trans);

    cout << "table->getNumberOfActiveDoF() = " << table->getNumberOfActiveDoF() <<endl;

    pair<int,vector<pair<double,double> > > c;
    c.first = 0;
    vector<pair<double,double> > v;
    pair<double,double> p;
    p.first = 1;
    p.second = 2;
    v.push_back(p);
    c.second = v;
    CL.addRange(c);
    addConstraintToList(CL);

    while (nbConfsFound < nbConfs)
    {
        for (unsigned int i; i < linkList.size(); i++)
        {

            linkList.at(i);
        }
        nbConfsFound++;
    }
    return true;

}

constrainedLink::constrainedLink()
{

}


constrainedLink::constrainedLink(Robot* Robot, vector<vector<double> > matrixRot, vector<double> vectTrans)
{
    R = Robot;
    matrixRotation = matrixRot;
    vectTranslation = vectTrans;
}

constrainedLink::~constrainedLink()
{
}

void constrainedLink::addRange(std::pair<int,vector<pair<double,double> > > range)
{
    rangeList[range.first] = range.second;
}
