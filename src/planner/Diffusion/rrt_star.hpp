// author: Romain Iehl <riehl@laas.fr>

#ifndef RRT_STAR_HPP_INCLUDED
#define RRT_STAR_HPP_INCLUDED

//#include "atoms_and_robot.hpp"
#include "light_node.hpp"
#include "light_graph.hpp"
#include "riehl_graph_operations.hpp"
#include "riehl_cspace.hpp"
#include "cspace.hpp"

#include <tr1/memory>
#include <vector>
#include <set>
#include <utility>
#include <boost/function.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/fusion/container/vector.hpp>
#include <boost/fusion/include/at_c.hpp> 
#include <libxml/parser.h>
#include <boost/timer.hpp>
#include "mutable_graph_with_components.hpp"

// Forward declarations
class StopCondition;
class ExpansionData;
template<typename Graph> class RRTStar;
//------------------------------------------------------------------------------
// Memory management of the NodeData objects :
// Everything through shared pointers, note that there is a
// dynamically allocated shared pointer in p3d_node,
// it will get freed when the p3d_node is deleted with a call
// to p3d_APInode_desalloc

template<typename Vertex>
struct RRTStarNodeData : public paf::GraphNodeData
{
  template<typename Graph> friend class RRTStar;
  
  // Don't use this, used only when creating a p3d_node,
  // due to the limitations of the memset based initialization
  RRTStarNodeData() : id(0), conf(), parent(NULL), parent_id(0), localpath_cost(0), component_id(0)
  {}
  
  // Constructor for the spirit parser rule
  RRTStarNodeData(const boost::fusion::vector<long unsigned, std::vector<double>, long unsigned, double, long unsigned, Robot*>& v) :
  id(boost::fusion::at_c<0>(v)),
  parent_id(boost::fusion::at_c<2>(v)),
  localpath_cost(boost::fusion::at_c<3>(v)),
  component_id(boost::fusion::at_c<4>(v))
  {
    Robot* robot(boost::fusion::at_c<5>(v));
    configPt c_conf = p3d_alloc_config_n(boost::fusion::at_c<1>(v).size());
    for(size_t i(0); i < boost::fusion::at_c<1>(v).size(); ++i)
    {
      c_conf[i] = boost::fusion::at_c<1>(v)[i];
    }
    conf = confPtr_t(new Configuration(robot, c_conf, true));
  }
  
  RRTStarNodeData(confPtr_t _conf) : id(0), conf(_conf), parent(NULL), parent_id(0), localpath_cost(0), component_id(0)
  {}
  
  RRTStarNodeData(const RRTStarNodeData<Vertex>& other) : id(other.id), conf(other.conf), parent(other.parent), parent_id(other.parent_id), localpath_cost(other.localpath_cost), component(other.component), component_id(other.component_id)
  {}
  
  RRTStarNodeData<Vertex>& operator=(const RRTStarNodeData<Vertex>& other)
  {
    id = other.id;
    conf = other.conf;
    parent = other.parent;
    parent_id = other.parent_id;
    localpath_cost = other.localpath_cost;
    component = other.component;
    component_id = other.component_id;
    return(*this);
  }
  
  // How to encode the node data in XML, used in writing the graph to a file.
  void write(xmlNodePtr parent)
  {
    xmlNewChild(parent, NULL, (const xmlChar*)("nodeData"), NULL);
  }
  
  std::vector<double> get_as_vector() const
  {
    std::vector<double> conf_vector(conf->getRobot()->getRobotStruct()->nb_dofs, 0.);
    for(unsigned i(0); i < conf_vector.size(); ++i)
    {
      conf_vector[i] = (*conf)[i];
    }
    return(conf_vector);
  }
  
  long unsigned id;
  confPtr_t conf;
  Vertex parent;
  long unsigned parent_id;
  double localpath_cost;
  typename std::list<Component<Vertex> >::iterator component;
  long unsigned component_id;
};
//------------------------------------------------------------------------------
class RRTStarEdgeData
{
  template<typename Graph> friend class RRTStar;
public:
  RRTStarEdgeData() {};
private:
  LocalPath* lp;
};
//------------------------------------------------------------------------------
template<typename Vertex>
class RRTStarGraphData
{
  template<typename Graph> friend class RRTStar;
public:
  typedef typename std::list<Component<Vertex> > ComponentList;
  typedef typename ComponentList::iterator ComponentIter;
  
  RRTStarGraphData() : num_vertices(0) {};
  
  GraphUidGenerator graph_ids;
  ComponentList clist;
  unsigned num_vertices;
};
//------------------------------------------------------------------------------
template <typename Graph>
class RRTStar
{
public:
  typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
  typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
  typedef RRTStarNodeData<Vertex> VertexBundle;
  typedef typename boost::graph_bundle_type<Graph>::type::ComponentList ComponentList;
  typedef typename ComponentList::iterator ComponentIter;
  typedef typename ComponentList::value_type Component;
  //----------------------------- Node data ------------------------------------
  typedef std::tr1::shared_ptr<RRTStarNodeData<Vertex> > NodeData_t;    
  
  static NodeData_t getData(LightNode* node)
  {
    return(std::tr1::dynamic_pointer_cast<VertexBundle , paf::GraphNodeData>(node->data()));
  }
  //------------------------------------------------------------------------------
  // Constructor, destructor
  // BOOST GRAPH
  RRTStar(Robot* R, Graph& graph2);
  
  // Dtor
  ~RRTStar();
  
  //------------------------------------------------------------------------------
  // General setters, getters
  Vertex get_start();
  Vertex get_goal();
  void set_start(Vertex v);
  void set_goal(Vertex v);
  void set_goal_region_flag(bool enable);
  void set_goal_region_configuration(confPtr_t q);
  // The goal region is defined as all configuration within
  // radius distance of the goal region configuration
  void set_goal_region_radius(double radius);
  // If this function is set, then it is used to determine if a configuration is within the goal region
  // in this case, the configuration and radius defined above will be unused.
  void set_goal_region_function(boost::function<bool(confPtr_t)> fun);
  void set_goal_region_active_dist(bool enable)
  { 
    m_goal_region_active_dist = enable;
  }
  void add_stop_condition(StopCondition* cond);
  void add_expand_node_condition(StopCondition* cond);
  void set_connect_to_goal(bool enable);
  void set_stop_when_connected_to_goal(bool enable);
  // Update several internal variable with the values that they have in the environment object ENV.
  // There is several reasons for keeping internal variables instead of directly accessing ENV :
  // 1. It ensures that the value of these variables cannot change during an RRT iteration.
  // 2. Update from the environment can be disabled, if needed.
  // 3. The code is cleaner
  void update_from_env();
  // Localpath setters and getters.
  void set_localpath_check(bool enable);
  bool get_localpath_check();
  // Extension step functions.
  void set_fixed_step(double value);
  void set_goal_connection_step(double value);
  void set_configuration_dependent_step(boost::function<double(LocalPath&)> fun);
  void set_coefs(std::vector<double> coefs);
  void set_rrg_radius_factor(double value);
  void set_sample_passive(bool enable);
  // cspace
  void set_cspace(CSpace* cspace);
  void set_max_time(double value) { m_max_time = value; }
  // Serialization
  static void set_ids_from_pointers(Graph& g);
  static void set_pointers_from_ids(Graph& g);
  //------------------------------------------------------------------------------
  // helpers - small functions that perform common tasks
  bool start_and_goal_linked()
  {
    return(m_goal && m_start && m_graph[m_start].component == m_graph[m_goal].component);
  }
  //------------------------------------------------------------------------------
  // The main functions of the algorithm.
  // The documentation for these functions is in the rrt_star.cpp source file.
  bool preConditions();
  
  void report();
  
  bool checkAdditionalStopConditions(confPtr_t conf);
  
  bool checkStopConditions(Vertex n);
  
  bool connect_node_to_comp(Vertex v, ComponentIter comp);
  
  unsigned run();
  
  enum extendStatus {
    trapped,
    advanced,
    reached};
  
  enum extendFailure {
    no_failure,
    collision,
    constraints
  };
  
  enum extend_return_components {
    status_e, // 0 : status
    data_e, // 1 : data of qNew (includes the accepted configuration)
    q_rej_e, // 2 : rejected configuration
    failure_e // 3 : cause of failure
  };
  
  typedef boost::tuple<extendStatus, NodeData_t, confPtr_t, extendFailure> extend_return_t;
  
  // A general implementation of the extend part of the algorithm.
  // Don't call this, call the RRTStar::extend function instead,
  // which will call extend_generic with all the appropriate arguments.
  extend_return_t extend_generic(
                                 // Input : exploration state
                                 confPtr_t qNear,
                                 confPtr_t qRand,
                                 // Input : general parameters
                                 double stepLength,
                                 bool extendOneStep); // one step or n-step ?
  
  extend_return_t extend(
                         confPtr_t qNear,
                         confPtr_t qRand,
                         bool extendOneStep);
  
  bool collision_free(confPtr_t q1, confPtr_t q2);
  
  double localpath_cost(confPtr_t q1, confPtr_t q2);
  double path_cost(Vertex n);
  std::vector<Vertex> get_path(Vertex n);
  double rrg_ball_radius();
  bool is_in_goal_region(confPtr_t q);
  Vertex extract_best_goal();
  
  bool valid_neighbour(Vertex n_nearest, confPtr_t q_new, Vertex neighbour);
  
  Vertex expand_simple(ComponentIter from_comp);
  
  void stats();
  
  void draw(Configuration& q);
  //----------------------------------------------------------------------------
  
protected:
  //----------------------------------------------------------------------------
  // General data members.
  // An instance of Atoms containing the coordinates of the start configuration.
  // Used in rmsd calculations (for informational purposes).
  //std::tr1::shared_ptr<AtomsAndRobot::atoms_t> m_start_atoms;
  // False by default. If set to true,
  // then m_goal_region_configuration must be set too.
  // m_goal_region_radius should also be set (the default value is 0)
  bool m_goal_region_flag;
  confPtr_t m_goal_region_configuration;
  double m_goal_region_radius;
  boost::function<bool(confPtr_t)> m_goal_region_function;
  bool m_goal_region_active_dist;
  Robot* m_robot;
  // cspace
  CSpace* m_cspace;
  unsigned m_nb_iteration;
  // Used to instanciate Atoms objects corresponding to m_robot
  //AtomsAndRobot m_atoms_and_robot;
  bool m_connect_to_goal;
  bool m_stop_when_connected_to_goal;
  unsigned m_max_nodes;
  bool m_draw_graph;
  bool m_draw_robot;
  bool m_draw_trajectory;
  unsigned m_drawing_delay;
  // User specified conditions for stopping the planning iterative process.
  std::vector<StopCondition*> m_stop_conditions;
  // User specified conditions for rejecting the selected expansion node (not used currently).
  std::vector<StopCondition*> m_expand_node_conditions;
  confPtr_t m_last_conf;
  int ExpansionDirectionMethod; // = GLOBAL_CS_EXP;
  // Determines if the passive dofs are sampled during the random smapling process.
  // True by default.
  bool m_sample_passive;
  //double GoalBias;
  //bool IsGoalBias;
  //------------------------------------------------------------------------------
  // Localpath related data members.
  enum step_type {fixed, configuration_dependent};
  step_type m_step_type;
  double m_step_length;
  double m_goal_connection_step_length;
  double m_rrg_radius_factor;
  // If this flag is false, in the extend phase,
  // the localpath between two consecutive configurations is not tested.
  // If this flag is false, in the extend phase,
  // the localpath between two consecutive configurations is not tested.
  bool m_localpath_test;
  boost::function<double(LocalPath&)> f_step_length_from_configurations;  
  bool m_use_coefs;
  std::vector<double> m_coefs;
  //------------------------------------------------------------------------------
  // Stats
  unsigned stat_rising_node;
  unsigned stat_expand_counter;
  unsigned stat_successful_col_active;
  unsigned stat_failed_col_active;
  unsigned stat_successful_col_passive;
  unsigned stat_failed_col_passive;
  unsigned stat_active_constraints_successful;
  unsigned stat_active_constraints_failed;
  double stat_max_cost;
  double stat_distance_to_goal_region;
  // Controls the use of a partial goal configuration :
  // we want to reach a position specific for one arm,
  // but any configuration of the two arms is acceptable.
  // In other words, we want to apply the same principle
  // as an ML-RRT (active/passive) goal, but even when using an
  // all active model.
  bool m_iros_kuka_partial_goal;
  double m_cost_to_goal;
  // BIG HACK
  std::vector<Vertex> in_goal;
  // timer
  boost::timer m_timer;
  double m_max_time;
  // gnuplot data
  
public:
  std::string plot_data_full;
  std::ofstream plot_data_full_s;
  std::string plot_data_10;
  std::ofstream plot_data_10_s;
  std::string plot_data_100;
  std::ofstream plot_data_100_s;
  
private:
  // BOOST GRAPH
  Graph& m_graph;
  Vertex m_start;
  Vertex m_goal;
};

#endif
