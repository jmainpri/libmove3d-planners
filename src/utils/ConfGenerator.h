#ifndef CONFGENERATOR_H
#define CONFGENERATOR_H


#include "API/planningAPI.hpp"

#include "API/Trajectory/trajectory.hpp"
#include "planner/planner.hpp"

#include "utils/OtpUtils.hpp"
#include "utils/ConfGenerator.h"

#include "LightPlanner-pkg.h"
#include "planner/TrajectoryOptim/plannarTrajectorySmoothing.hpp"


#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <Eigen/StdVector>

class ConfGenerator
{
public:
    ConfGenerator();
    ConfGenerator(Robot* rob,Robot* human);

    std::vector<Eigen::Vector3d> getOTPList(){return m_OTPList;}

    /**
      * Compute
      */
    bool computeRobotGikForGrabing( configPt& q);

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

private:

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
};

#endif // CONFGENERATOR_H