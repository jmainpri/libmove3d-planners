#ifndef CONFGENERATOR_H
#define CONFGENERATOR_H

#include "API/Device/robot.hpp"
#include "planner/planner.hpp"

#include "utils/OtpUtils.hpp"
#include "utils/ConfGenerator.h"

#include "LightPlanner-pkg.h"
#include "planner/TrajectoryOptim/plannarTrajectorySmoothing.hpp"
#include "hri_costspace/HRICS_Natural.hpp"


#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <Eigen/StdVector>


class constrainedLink
{
public:
    constrainedLink();
    constrainedLink(Robot* first, std::vector<std::vector<double> > matrixRot, std::vector<double> vectTrans);
    virtual ~constrainedLink();
    void addRange(std::pair<int,std::vector<std::pair<double,double> > > range);


    string name;
    Robot* R;
    std::vector<std::vector<double> > matrixRotation;
    std::vector<double> vectTranslation;

    std::map<int,std::vector<std::pair<double,double> > > rangeList;
    int nbDofs;
};


//typedef struct linkStruct
//{
//    Robot* first;
//    Robot* second;
//    configurationConstraint CC;
//}linked;

//typedef struct configurationConstraintStruct
//{
//
//}configurationConstraint;

class ConfGenerator
{
public:
  ConfGenerator();
  ConfGenerator(Robot* rob,Robot* human);
  
  std::vector<Eigen::Vector3d> getOTPList(){return m_OTPList;}
  
  /**
   * Compute a configuration for hand-over
   */
  bool computeRobotIkForGrabing( configPt& q );
  
  /**
   * Compute a configuration for handing over an object for the arm Dofs
   * @point the point in the workspace
   * @q the configuration of the robot
   */
  bool computeRobotIkForGrabing( configPt& q, const Eigen::Vector3d& point );
  
  /**
   * Compute a handover configuration by iterating through the set of configurations
   * that has been pre-loaded
   */
  bool computeHandoverConfigFromList( std::pair<confPtr_t,confPtr_t>& best_handover_conf, double& best_cost );
  
  /**
   * Initializes the list config generator
   */
  bool initialize( string filename, HRICS::Natural* reachableSpace );
  
  /**
   * load and return configs stored in filename
   */
  bool sortConfig(std::vector<HRICS::ConfigHR>& configList, int nbNode, bool isStanding, bool isSlice, HRICS::Natural* reachableSpace);
  
  /**
   * Adding a 3D point to the OTP list This function is used when loading a set of OTPs in order to test them
   */
  void addToList(Eigen::Vector3d WSPoint);
  
  /**
   * Change the current OTP. Used when choosing new configurations
   */
  Eigen::Vector3d setCurOTP( Eigen::Vector3d WSPoint);
  
  /**
   * draw the OTP list
   */
  void drawOTPList(bool value);
  
  /**
   * Compute GIK and place robot base
   */
  bool placeRobot();
  
  /**
   * Adding the actual configuration to m_configList
   */
  std::vector<HRICS::ConfigHR> addConfToList();
  
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
  std::vector<HRICS::ConfigHR> loadFromXml(std::string filename);

  //########################################
  //new version :
  //########################################
  
  bool generateConfiguration(int nbConfs);
  bool addConstraintToList(constrainedLink CL);

  
private:
  
  /**
   * Set the human into the configuration at the ihe index given as parametter
   * @param ith index if vector
   * @param vectConfs vector of human robot config
   */
  std::pair<confPtr_t,confPtr_t> setRobotsToConf(const HRICS::ConfigHR& handover_conf);
  
  /**
   * the list of OTPs used to create a new config list
   */
  std::vector<Eigen::Vector3d> m_OTPList;
  
  /**
   * store configs (for standing human when loading)
   */
  std::vector<HRICS::ConfigHR> m_configList;
  
  Robot* _robot;
  Robot* _human;

  //########################################
  //new version :
  //########################################
  std::vector<constrainedLink> linkList;
  std::map<std::string,constrainedLink> linkListFiltred;
};

#endif // CONFGENERATOR_H
