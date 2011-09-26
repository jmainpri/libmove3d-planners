#ifndef COST_SPACE_HPP_INCLUDED
#define COST_SPACE_HPP_INCLUDED

#include <boost/function.hpp>
#include <map>
#include <string>

#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/localpath.hpp"
#include "API/Roadmap/graph.hpp"


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
	cs_boltzman_cost
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
  double cost(LocalPath& path);
	
	// Set node cost
	void setNodeCost(Node* node, Node* parent);
	
  // Initializes the Cost space motion planning problem
  void initMotionPlanning(Graph* graph, Node* start, Node* goal);
	
  // Set DeltaStepCost
  void setDeltaStepMethod(CostSpaceDeltaStepMethod method) { m_deltaMethod = method; }
	
	// Compute the delta step cost
  double deltaStepCost(double cost1, double cost2, double length);
  
protected:
  std::string mSelectedCostName;
  boost::function<double(Configuration&)> mSelectedCost;
  std::map<std::string, boost::function<double(Configuration&)> > mFunctions;
	
private:
	
  // Delta
  enum CostSpaceDeltaStepMethod m_deltaMethod;
	
};

namespace GlobalCostSpace 
{
	void initialize();
};

extern CostSpace* global_costSpace;

double computeIntersectionWithGround(Configuration& conf);
double computeFlatCost(Configuration& conf);
double computeDistanceToObstacles(Configuration& conf);
double computeLocalpathKinematicCost(p3d_rob* rob, p3d_localpath* LP);

#endif
