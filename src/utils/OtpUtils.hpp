#ifndef OTPUTILS_HPP
#define OTPUTILS_HPP

#include "API/Trajectory/trajectory.hpp"

#include "planner/planner.hpp"
#include "planner/TrajectoryOptim/plannarTrajectorySmoothing.hpp"

#include "utils/OtpUtils.hpp"

#include <libmove3d/include/LightPlanner-pkg.h>

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <Eigen/StdVector>



extern void g3d_show_tcur_both_rob(p3d_rob *robotPt, int (*fct)(p3d_rob* robot, p3d_localpath* curLp),
                                   p3d_rob *hum_robotPt, int (*hum_fct)(p3d_rob* hum_robot, p3d_localpath* hum_curLp));

extern bool detectSittingFurniture( Move3D::Robot* human, double threshold, Move3D::Robot** furniture);

namespace HRICS
{
    class OutputConf
    {
    public:
        Move3D::confPtr_t humanConf;
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > humanTraj;
        bool humanTrajExist;

        Move3D::confPtr_t robotConf;
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > robotTraj;
        bool robotTrajExist;

        double cost;
        int configNumberInList;
        int id;
        bool isStandingInThisConf;

        void clearAll();

        Move3D::confPtr_t chairConf;
//        OutputConf& operator= (const OutputConf& o);

    };

    class ConfigHR
    {
    public:

        static int index;
        ConfigHR() { id = index++; cost = 1.; }

        configPt getHumanConf() const { return q_hum; }
        void setHumanConf( Move3D::Robot* human, configPt q);

        configPt getRobotConf() const { return q_rob; }
        void setRobotConf( Move3D::Robot* robot, configPt q);

        int getId() const { return id; }
        void setId(int value) { id = value; }

        double getCost() const { return cost; }
        void setCost(double value) { cost = value; }

    private:
        configPt q_hum;
        configPt q_rob;
        int id;
        double cost;

    };

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

    };

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

    };

}


#endif // OTPUTILS_HPP
