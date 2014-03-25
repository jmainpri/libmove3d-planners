#ifndef COST_SPACE_HPP_INCLUDED
#define COST_SPACE_HPP_INCLUDED

#include <boost/function.hpp>
#include <map>
#include <string>

#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/localpath.hpp"
#include "API/Roadmap/graph.hpp"

namespace Move3D {

/*!
 * Delta step cost method enum
 */
enum CostSpaceDeltaStepMethod 
{
    cs_integral,
    cs_mechanical_work,
    cs_visibility,
    cs_average,
    cs_config_cost_and_dist,
    cs_boltzman_cost,
    cs_max
};

/*!
 * Delta step cost method enum
 */
enum CostSpaceResolutionMethod
{
    cs_classic,
    cs_pr2_manip
};

/*!
 * Class thats holding the CostSpace
 */
class CostSpace
{
public:
    CostSpace();

    // Get selected cost function name
    std::string getSelectedCostName();

    // Get All Cost Functions
    std::vector<std::string> getAllCost();

    // Select the cost function with the given name in the map
    bool setCost(std::string name);

    // Register a new cost function.
    void addCost(std::string name,
                 boost::function<double(Configuration&)> f);

    // Delete a cost function
    void deleteCost(std::string name);

    // Compute the cost of the configuration conf.
    double cost(Configuration& conf);

    // Compute the cost of
    double cost(LocalPath& path, int& nb_test);

    // Set node cost
    void setNodeCost(Node* node, Node* parent);

    // Initializes the Cost space motion planning problem
    void initMotionPlanning(graph* graph, node* start, node* goal);

    // Set DeltaStepCost
    void setDistanceMethod(CostSpaceResolutionMethod method) { m_resolution = method; }

    // Set DeltaStepCost
    void setDeltaStepMethod(CostSpaceDeltaStepMethod method) { m_deltaMethod = method; }

    // Get DeltaStepCost
    CostSpaceDeltaStepMethod getDeltaStepMethod() { return m_deltaMethod; }

    // Compute the delta step cost
    double deltaStepCost(double cost1, double cost2, double length);

protected:
    std::string mSelectedCostName;
    boost::function<double(Configuration&)> mSelectedCost;
    std::map<std::string, boost::function<double(Configuration&)> > mFunctions;

    void getPr2ArmConfiguration( Eigen::VectorXd& x, confPtr_t q );
    double getPr2ArmDistance( Robot* robot, Eigen::VectorXd& q_i, Eigen::VectorXd& q_f );

private:

    // Delta
    enum CostSpaceDeltaStepMethod m_deltaMethod;
    enum CostSpaceResolutionMethod m_resolution;

    double m_dmax;
};

namespace GlobalCostSpace 
{
void initialize();
};

double computeIntersectionWithGround(Configuration& conf);
double computeFlatCost(Configuration& conf);
double computeDistanceToObstacles(Configuration& conf);
double computeInCollisionCost(Configuration& conf);
double computeCollisionSpaceCost(Configuration& conf);
double computeLocalpathKinematicCost(void* rob, localpath* LP);

extern CostSpace* global_costSpace;

}

#endif
