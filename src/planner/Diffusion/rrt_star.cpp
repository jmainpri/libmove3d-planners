// author: Romain Iehl <riehl@laas.fr>

#include "rrt_star.hpp"
#include "logging.hpp"
#include "stop_condition.hpp"
#include "localpath_costspace.hpp"
#include "cost_space.hpp"
#include "riehl_localpaths.hpp"
#include "localpath_scaled/localpath_scaled.hpp"
#include "localpath_scaled/localpath_scaled_move3d_impl.hpp"
#include "mltrrt_cost_based_selection.hpp" // for the localpath cost function
#include "algorithm_ext2.hpp"
#include "boost_graph_operations.hpp"

#include <limits>
#include <algorithm>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/format.hpp>
#include <tr1/tuple>
// Another macro that breaks everything
#undef EPS
// BOOST GRAPH
#include <boost/graph/adjacency_list.hpp>
#include "mutable_graph_with_components.hpp"
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "draw_boost_graph.hpp"

typedef boost::graph_traits<boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS> >::vertex_descriptor Vertex;
typedef RRTStar<boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS, RRTStarNodeData<Vertex> , RRTStarEdgeData, RRTStarGraphData<Vertex> > > RRTStar_t;
// ugly hack to draw the boost graph.
void* boost_graph(0);
Robot* boost_robot(0);
bool boost_mode(true);
std::vector<RRTStar_t::Vertex> boost_traj;

BOOST_FUSION_ADAPT_STRUCT(
                          RRTStarNodeData<Vertex>,
                          (long unsigned, parent_id)
                          (long unsigned, component_id)
                          (double, localpath_cost)
                          )

//#include <QtGui/QImage>
//void save_opengl_buffer(QString str);

extern int fct_stop();
extern int fct_draw();

// hackish kuka stuff
void copy_passive_arms_from_q1_to_q2(confPtr_t q1, confPtr_t q2);

// A couple of helper macros, to avoid typing out long templated types.
// Iterate through all elements of a container.
#ifndef __GXX_EXPERIMENTAL_CXX0X__
#define FORT(i, a) for(typeof(a.begin()) i = a.begin(); i != a.end(); ++i)
#else
#define FORT(i, a) for(auto i = a.begin(); i != a.end(); ++i)
#endif
// Get the type of an iterator.
#ifndef __GXX_EXPERIMENTAL_CXX0X__
#define ITERATOR_OF(c) typeof(c.begin())
#else
#define ITERATOR_OF(c) auto
#endif

#ifndef NDEBUG
#define unreachable(MSG)                        \
(assert(0 && MSG), abort())
#else
#define unreachable(MSG)                                        \
(std::fprintf(stderr, "UNREACHABLE executed at %s:%d\n",    \
__FILE__, __LINE__), abort())
#endif 

//------------------------------------------------------------------------------
// Constructor
//
//------------------------------------------------------------------------------
// BOOST GRAPH
template<class Graph>
RRTStar<Graph>::RRTStar(Robot* R, Graph& graph) :
// General
m_goal_region_flag(false),
m_goal_region_radius(0),
m_goal_region_active_dist(false),
m_robot(R),
m_nb_iteration(0),
//m_atoms_and_robot(m_robot->getRobotStruct()),
m_connect_to_goal(true),
m_max_nodes(0),
m_draw_graph(false),
m_draw_robot(false),
m_draw_trajectory(false),
m_drawing_delay(0),
ExpansionDirectionMethod(GLOBAL_CS_EXP),
m_sample_passive(true),
// Localpath stuff
m_step_type(fixed),
m_step_length(0.0),
m_goal_connection_step_length(0.0),
m_rrg_radius_factor(1.0),
m_localpath_test(true),
m_use_coefs(false),
// Statistics data members initialization
stat_rising_node(0),
stat_expand_counter(0),
stat_successful_col_active(0),
stat_failed_col_active(0),
stat_successful_col_passive(0),
stat_failed_col_passive(0),
stat_active_constraints_successful(0),
stat_active_constraints_failed(0),
stat_max_cost(std::numeric_limits<double>::min()),
stat_distance_to_goal_region(std::numeric_limits<double>::max()),
m_iros_kuka_partial_goal(false),
m_cost_to_goal(std::numeric_limits<double>::max()),
m_max_time(0),
plot_data_full(""),
plot_data_10(""),
plot_data_100(""),
// BOOST GRAPH
m_graph(graph),
m_start(0),
m_goal(0)
{
  boost_graph = (void*) &m_graph;
  boost_robot = m_robot;
  this->update_from_env();
}
//------------------------------------------------------------------------------
template<class Graph>
RRTStar<Graph>::~RRTStar()
{
  std::cout << "RRTStar state at destruction : " << std::endl;
  this->report();
  // What is best... managing them outside, maybe ? or use a shared_ptr ?
  // for(unsigned i(0); i < m_stop_conditions.size(); i++) { delete m_stop_conditions[i]; }
  // for(unsigned i(0); i < m_expand_node_conditions.size(); i++) { delete m_expand_node_conditions[i]; }
}
//------------------------------------------------------------------------------
template<class Graph>
typename RRTStar<Graph>::Vertex RRTStar<Graph>::get_start() { return(m_start); }

template<class Graph>
typename RRTStar<Graph>::Vertex RRTStar<Graph>::get_goal() { return(m_goal); }

template<class Graph>
void RRTStar<Graph>::set_start(Vertex v)
{ 
  m_start = v;
}

template<class Graph>
void RRTStar<Graph>::set_goal(Vertex v)
{
  m_goal = v;
}

template<class Graph>
void RRTStar<Graph>::set_goal_region_flag(bool enable)
{
  m_goal_region_flag = enable;
}

template<class Graph>
void RRTStar<Graph>::set_goal_region_configuration(confPtr_t q)
{
  m_goal_region_configuration = q;
}

template<class Graph>
void RRTStar<Graph>::set_goal_region_radius(double radius)
{
  m_goal_region_radius = radius;
}

template<class Graph>
void RRTStar<Graph>::set_goal_region_function(boost::function<bool(confPtr_t)> fun)
{
  m_goal_region_function = fun;
}
// Stop conditions that are checked during the extend phase.
// Each time that a new configuration is accepted, the function evaluate(q)
// of each stop condition will be called with the new configuration, if any
// of these returns true, the algorithm is stopped.
template<class Graph>
void RRTStar<Graph>::add_stop_condition(StopCondition* cond)
{
  m_stop_conditions.push_back(cond);
}

// Same principle as above, intended as a test on the q_near that is selected
// during the expansion.
// Currently unused.
template<class Graph>
void RRTStar<Graph>::add_expand_node_condition(StopCondition* cond)
{
  m_expand_node_conditions.push_back(cond);
}

// Is the exploration global, or does it try to link to a goal configuration ?
template<class Graph>
void RRTStar<Graph>::set_connect_to_goal(bool enable) { m_connect_to_goal = enable; }
template<class Graph>
void RRTStar<Graph>::set_stop_when_connected_to_goal(bool enable) { m_stop_when_connected_to_goal = enable; }
//------------------------------------------------------------------------------
template<class Graph>
void RRTStar<Graph>::update_from_env()
{
  m_connect_to_goal = ENV.getBool(Env::expandToGoal);
  m_max_nodes = (unsigned) ENV.getInt(Env::maxNodeCompco);
  m_draw_trajectory = ENV.getBool(Env::drawTraj);
}
//------------------------------------------------------------------------------
// Localpath setters and getters.
// Do we test the validity of the localpath, between the consecutive
// configurations of the extend phase ?
template<class Graph>
void RRTStar<Graph>::set_localpath_check(bool enable) { m_localpath_test = enable; }

template<class Graph>
bool RRTStar<Graph>::get_localpath_check() { return(m_localpath_test); }
//------------------------------------------------------------------------------
// Extension step functions
template<class Graph>
void RRTStar<Graph>::set_fixed_step(double value)
{
  m_step_type = fixed;
  m_step_length = value;
}
template<class Graph>
void RRTStar<Graph>::set_goal_connection_step(double value)
{
  m_goal_connection_step_length = value;
}
template<class Graph>
void RRTStar<Graph>::set_configuration_dependent_step(boost::function<double(LocalPath&)> fun)
{
  m_step_type = configuration_dependent;
  f_step_length_from_configurations = fun;
}

template<class Graph>
void RRTStar<Graph>::set_coefs(std::vector<double> coefs)
{
  m_use_coefs = true;
  m_coefs = coefs;
}

template<class Graph>
void RRTStar<Graph>::set_rrg_radius_factor(double value)
{
  m_rrg_radius_factor = value;
}
template<class Graph>
void RRTStar<Graph>::set_sample_passive(bool enable)
{
  m_sample_passive = enable;
}
// cspace
template<class Graph>
void RRTStar<Graph>::set_cspace(CSpace* cspace)
{
  m_cspace = cspace;
}
//------------------------------------------------------------------------------
// Serialization
//------------------------------------------------------------------------------
template<class Graph>
void RRTStar<Graph>::set_ids_from_pointers(Graph& g)
{
  BOOST_FOREACH(Vertex v, boost::vertices(g))
  {
    // no parent case (null pointer)
    if(g[v].parent == 0)
    {
      g[v].parent_id = 0;
    }
    // default case
    else
    {
      g[v].parent_id = g[g[v].parent].id;
    }
  }
}
//------------------------------------------------------------------------------
template<class Graph>
void RRTStar<Graph>::set_pointers_from_ids(Graph& g)
{
  
}
//------------------------------------------------------------------------------
// Verify that everything is properly initialized.
template<class Graph>
bool RRTStar<Graph>::preConditions()
{
  this->update_from_env();
  
  std::cout << "Checking the initialization of the planner..." << std::endl;
  
  if(!m_start)
  {
    std::cout << "ERROR: the start vertex is null." << std::endl;
    return false;
  }
  
  if (m_graph[m_start].conf->IsInCollision())
  {
    std::cout << "The start configuration is in collision" << std::endl;
    return false;
  }
  
  // Check that there is a goal node, and that the goal configuration
  // is different from the start configuration
  if(m_connect_to_goal && !m_goal_region_flag)
  {
    if(m_goal == 0)
    {
      std::cout << "ERROR: the goal node pointer is null." << std::endl;
      return false;
    }
    if(*m_graph[m_start].conf == *m_graph[m_goal].conf)
    {
      std::cout << "Tree Expansion failed: start and goal nodes are the same" << std::endl;
      return false;
    }
    if(m_graph[m_goal].conf->IsInCollision())
    {
      std::cout << "The goal configuration is in collision" << std::endl;
      return false;
    }
    if(m_stop_when_connected_to_goal && this->start_and_goal_linked())
    {
      std::cout << "Warning: the search is set to stop once the start and goal configuration are in the same connected component, however they are already in the same connected component." << std::endl;
      return true;
    }
  }
  
  // Verify the goal region stuff
  if(m_goal_region_flag)
  {
    // Either use the user supplied goal region function, or a simple
    // region defined as a sphere around a configuration.
    if(m_goal_region_function.empty())
    {
      if(!m_goal_region_configuration.get())
      {
        std::cout << "ERROR: the goal region is enabled (m_goal_region_flag == true), however there is no \
        goal region function and no goal region configuration (m_goal_region_configuration.get() == 0), \
        at least one of these must be set." << std::endl;
      }
      if(m_goal_region_radius <= 0.0)
      {
        std::cout << "Warning: the goal region is enabled (m_goal_region_flag == true), however the goal region radius seems erroneous (m_goal_region_radius == " << m_goal_region_radius << ")" << std::endl;
      }
    }
  }
  
  if(!m_robot)
  {
    std::cout << "ERROR: the robot pointer is null." << std::endl;
    return false;
  }
  
  std::cout << "RRTStar is properly initiliazed." << std::endl;
  std::cout << "The state at initialization is : " << std::endl;
  this->report();
  return(true);
}
//------------------------------------------------------------------------------
template<class Graph>
void RRTStar<Graph>::report()
{
  std::cout << boost::format("There is %d vertices in the graph.\n") % m_graph[boost::graph_bundle].num_vertices;
  // Iterate over the components, print some information.
  BOOST_FOREACH(Component& c, m_graph[boost::graph_bundle].clist)
  {
    std::cout << boost::format("Component %p :\n") % &c;
    std::cout << "nodes.size() : " << c.num_vertices << std::endl;
  }
  std::cout << " m_step_length : " << m_step_length << std::endl;
  std::cout << " m_goal_connection_step_length : " << m_goal_connection_step_length << std::endl;
}
//------------------------------------------------------------------------------
/*!
 Check the stop conditions in m_stop_conditions, stop if any is met.
 */
template<class Graph>
bool RRTStar<Graph>::checkAdditionalStopConditions(confPtr_t conf)
{
  for(unsigned i(0); i < m_stop_conditions.size(); i++)
  {
    if(m_stop_conditions[i]->evaluate(conf))
    {
      printf("Stop condition %i verified.\n", i);
      return(true);
    }
  }
  return(false);
}
//------------------------------------------------------------------------------
/*!
 Check the stop conditions of the planner, stop if any is met.
 */
template<class Graph>
bool RRTStar<Graph>::checkStopConditions(Vertex v)
{
  if(v != 0)
  {
    if(this->checkAdditionalStopConditions(m_graph[v].conf))
    { return(true); }
  }
  
  if(m_stop_when_connected_to_goal && m_connect_to_goal &&
     m_graph[m_start].component == m_graph[m_goal].component)
  {
    std::cout << "Success: the start and goal components are connected." << std::endl;
    return(true);
  }
  
  if(m_graph[m_start].component->num_vertices >= m_max_nodes)
  {
    std::cout << "Failure: the maximum number of nodes in the start component is reached." << std::endl;
    return (true);
  }
  
  if(m_graph[boost::graph_bundle].num_vertices >= m_max_nodes)
  {
    std::cout << "Failure: the maximum number of nodes in the graph is reached." << std::endl;
    return (true);
  }
  
  if (!fct_stop())
  {
    p3d_SetStopValue(true);
  }
  
  if (p3d_GetStopValue())
  {
    std::cout << "Tree expansion cancelled." << std::endl;
    return (true);
  }
  
  double elapsed(m_timer.elapsed());
  if (m_max_time > 0 && elapsed > m_max_time)
  {
    std::cout << "Maximum time reached : " << elapsed << " seconds out of " << m_max_time << " seconds.\n";
    return(true);
  }
  
  return (false);
}

//------------------------------------------------------------------------------
// Try connecting the node n1 to the connected component c2.
// The extend function is used to attempt the connection between n1 (in the
// component c1) and its nearest neighbour n2 in c2.
// pre-condition : c1 != c2
template<class Graph>
bool RRTStar<Graph>::connect_node_to_comp(Vertex n1, ComponentIter c2)
{
  if(m_graph[n1].component == c2)
  {
    PrintInfo(("Warning: trying to connect a node to its own connected component\n"));
    return(false);
  }
  
  Vertex n2 = graph_nn_boost(m_robot, c2->nodes, m_graph[n1].conf, cspace_dist_euclidean, m_graph);
  
  if (n2 == 0)
  {
    PrintInfo(("Warning: connect_node_to_comp: failed to find a nearest node in the component to connect to.\n"));
    return(false);
  }
  
  RRTStar<Graph>::extend_return_t extend_rv =
  this->extend_generic(m_graph[n1].conf, m_graph[n2].conf, m_goal_connection_step_length, true);
  //printf("connecting to goal : %s\n", extend_rv.get<status_e>() == reached ? "reached" : "not reached");
  if(extend_rv.get<status_e>() == reached)
  {
    merge_components(n1, n2, m_graph);
    m_graph[n2].parent = n1;
    m_graph[n2].localpath_cost = m_cspace->lp_cost(m_graph[n1].conf, m_graph[n2].conf);
    return(true);
  }
  else { return(false); }
}

// Main function of planner.
// The preconditions are checked, and if they are correct, the iterative process of the
// algorithm is started. Each iteration performs a single expansion step, while handling
// the connection to the goal and the bi-directionality.
template<class Graph>
unsigned RRTStar<Graph>::run()
{
  typedef typename std::back_insert_iterator<std::string> OutputIterator;
  typedef typename std::string::iterator InputIterator;
  //  namespace karma = boost::spirit::karma;
  //  namespace qi = boost::spirit::qi;
  //  namespace phoenix = boost::phoenix;
  //  namespace ascii = boost::spirit::ascii;
  
  //  // karma::rule<OutputIterator, VertexBundle()> vdata_gen_r =
  //  //     karma::lit("parent ") << karma::ulong_ << karma::lit(" ") <<
  //  //     karma::lit("component ") << karma::ulong_ << karma::lit(" ") <<
  //  //     karma::lit("localpath_cost ") << karma::double_;
  
  //  karma::rule<OutputIterator, VertexBundle()> vdata_gen_r = 
  //      karma::lit("id ") << karma::ulong_[karma::_1 = phoenix::bind(&VertexBundle::id, karma::_val)] << karma::lit(" ") <<
  //      karma::lit("configuration ") << (*(karma::double_ << karma::lit(" ")))[karma::_1 = phoenix::bind(&VertexBundle::get_as_vector, karma::_val)] <<
  //      karma::lit("parent ") << karma::ulong_[karma::_1 = phoenix::bind(&VertexBundle::parent_id, karma::_val)] << karma::lit(" ") <<
  //      karma::lit("localpath_cost ") << karma::double_[karma::_1 = phoenix::bind(&VertexBundle::localpath_cost, karma::_val)] << karma::lit(" ") <<
  //      karma::lit("component ") << karma::ulong_[karma::_1 = phoenix::bind(&VertexBundle::component_id, karma::_val)] << karma::lit(" ");
  
  //  qi::rule<InputIterator, boost::fusion::vector<long unsigned, std::vector<double>, long unsigned, double, long unsigned, Robot*>()> vdata_parse_helper_r =
  //      qi::lit("id") >> qi::ulong_ >>
  //      qi::lit("configuration") >> (*qi::double_) >>
  //      qi::lit("parent") >> qi::ulong_ >>
  //      qi::lit("localpath_cost") >> qi::double_ >>
  //      qi::lit("component") >> qi::ulong_ >>
  //      qi::attr(m_robot);
  //  qi::rule<InputIterator, VertexBundle(), ascii::space_type> vdata_parse_r = vdata_parse_helper_r;
  
  // graph_io::MutableGraphGenerator<Graph, OutputIterator, InputIterator> gen(vdata_gen_r, vdata_parse_r);
  
  // If the log filenames are set, open the filestreams.
  if(plot_data_full != "")
  {
    plot_data_full_s.open(plot_data_full.c_str());
  }
  if(plot_data_10 != "")
  {
    plot_data_10_s.open(plot_data_10.c_str());
  }
  if(plot_data_100 != "")
  {
    plot_data_100_s.open(plot_data_100.c_str());
  }
  m_timer.restart();
  if(!preConditions())
  {
    return 0;
  }
  
  unsigned nb_nodes_init(m_graph[boost::graph_bundle].num_vertices);
  
  m_nb_iteration = 0;
  while(true)
  {
    this->update_from_env();
    ++m_nb_iteration;
    // Expansion.
    Vertex n = this->expand_simple(m_graph[m_start].component);
    this->set_ids_from_pointers(m_graph);
    
    // If new nodes been added :
    // maybe we need to draw the new graph,
    // and attempt to connect the new nodes to the goal
    if(n != 0)
    {
      m_last_conf = m_graph[n].conf;
      // Update the rendering.
      this->draw(*m_last_conf);
      
      // Try to connect to the goal.
      if (m_connect_to_goal && m_graph[m_start].component != m_graph[m_goal].component)
      {
        if(this->connect_node_to_comp(n, m_graph[m_goal].component))
        {
          std::cout << "The start and goal components have been connected." << std::endl;
        }
      }
    }
    
    // Exit if any stop condition is verified.
    if(checkStopConditions(n)) { break; }
  }
  //this->stats();
  //ENV.setBool(Env::isRunning,false);
  // close the filestreams
  if(plot_data_full_s.is_open())
  {
    plot_data_full_s.close();
  }
  if(plot_data_10_s.is_open())
  {
    plot_data_10_s.close();
  }
  if(plot_data_100_s.is_open())
  {
    plot_data_100_s.close();
  }
  return(m_graph[boost::graph_bundle].num_vertices - nb_nodes_init);
}
//------------------------------------------------------------------------------
// Extend
//------------------------------------------------------------------------------
// Steps :
// Until qRand is reached or until there is a failure :
//     Get the next configuration C
// 1.  Verify the constraints of C and geometrically verify the path to C
// 2.  Perform the cost visibility test :
// 3.  Success : add C
// 4.  Failed : perform the cost transition test, adjust temp
// 5.    Success : add C
// 6.    Failed : exit
template<class Graph>
typename RRTStar<Graph>::extend_return_t RRTStar<Graph>::extend_generic
(
 // Input : exploration state
 confPtr_t qNear,
 confPtr_t qRand,
 // Input : general parameters
 double stepLength,
 bool extendOneStep) // false = n-step, true = one step
{
  //bool useDistanceControl(true);
  extend_return_t rv(trapped, NodeData_t(new VertexBundle()), confPtr_t(), no_failure); // return value
  // Scaled localpath : only used if m_use_coefs == true
  LocalpathScaled<confPtr_t, double>* extensionScaledPath(0);
  if(m_use_coefs)
  {
    extensionScaledPath = new LocalpathScaled<confPtr_t, double>(
                                                                 m_robot->getRobotStruct()->nb_dofs,
                                                                 qNear,
                                                                 qRand,
                                                                 m_coefs,
                                                                 copy,
                                                                 at,
                                                                 set,
                                                                 plus,
                                                                 minus,
                                                                 mult,
                                                                 general_distance,
                                                                 interpolate_dim
                                                                 );
  }
  LocalPath extensionPath(qNear, qRand);
  PathSegments segments(m_use_coefs ?
                        extensionScaledPath->length() :
                        extensionPath.getParamMax(),
                        // WARNING : maximum displacement can't be computed on a scaled localpath,
                        // so for now compute it on extensionPath (a reasonable approximation?)
                        std::max(10e-6,
                                 m_step_type == configuration_dependent ?
                                 f_step_length_from_configurations(extensionPath) :
                                 stepLength));
  _DEBUG_MSG("extend step is %f\n", segments.step());
  // We need at least two points for one segment.
  if(segments.nbPoints() <= 1)
  {
    fprintf(stderr,
            "WARNING : in extend_generic : 0-length localpath, aborting extension\n");
    return(rv);
  }
  // Beginning configuration of the current segment.
  confPtr_t segBegConf = qNear;
  bool singleStepBreak = false;
  for(unsigned i(1); i < segments.nbPoints() && !singleStepBreak; ++i)
  {
    //printf("nb points : %d\n", segments.nbPoints());
    _DEBUG_MSG("extend iteration %d - ", i);
    if(extendOneStep) { singleStepBreak = true; }
    //--------------------------------------------------------------------------
    // 1. Verify the constraints and the collisions
    //--------------------------------------------------------------------------
    confPtr_t segEndConf = m_use_coefs ?
    extensionScaledPath->point_at_normalized(segments[i] / extensionScaledPath->length()) :
    extensionPath.configAtParam(segments[i]);
    // constraints
    if(!m_robot->setAndUpdate(*segEndConf))
    { 
      ++stat_active_constraints_failed;
      //_DEBUG_MSG("CONSTRAINTS FAILED"); 
      rv.get<q_rej_e>() = segEndConf;
      rv.get<failure_e>() = constraints;
      break;
    }
    ++stat_active_constraints_successful;
    // Test collisions.
    if(m_localpath_test) // with a localpath
    {
      LocalPath segmentPath(segBegConf, segEndConf);
      if(!segmentPath.getValid()) {
        ++stat_failed_col_active;
        _DEBUG_MSG("\n");
        rv.get<q_rej_e>() = segEndConf;
        rv.get<failure_e>() = collision;
        break;
      } // collision -> leave the loop
    }
    else // without a localpath - only test the end configuration
    {
      if(segEndConf->IsInCollisionWithoutConstraints())
      {
        ++stat_failed_col_active;
        _DEBUG_MSG("\n");
        rv.get<q_rej_e>() = segEndConf;
        rv.get<failure_e>() = collision;
        break; 
      }
    }
    ++stat_successful_col_active;
    //printf("   nocol\n");
    NodeData_t segEndData = NodeData_t(new VertexBundle(segEndConf));
    //paf::Node* segEndNode = graph.add_node(segEndData);
    //graph.addEdge(segBegNode, segEndNode);
    rv.get<data_e>() = segEndData;
    rv.get<status_e>() = (i == segments.nbPoints() - 1 ? reached : advanced);
    // Condition hack has to be added here as well as the end of the function
    // After everything is done, check the stop conditions. kinda hackish.
    if(this->checkAdditionalStopConditions(segEndConf)) { break; }
    segBegConf = segEndConf;
    _DEBUG_MSG("\n");
    continue;
  }
  if(m_use_coefs) { delete(extensionScaledPath); }
  return(rv);
}
//------------------------------------------------------------------------------
// Perform an extension from qNear to qRand, in the component comp.
template<class Graph>
typename RRTStar<Graph>::extend_return_t RRTStar<Graph>::extend(
                                                                confPtr_t qNear,
                                                                confPtr_t qRand,
                                                                bool extendOneStep)
{
  return(this->extend_generic(
                              qNear,
                              qRand,
                              m_step_length,
                              extendOneStep));
}
//------------------------------------------------------------------------------
template<class Graph>
bool RRTStar<Graph>::collision_free(confPtr_t q1, confPtr_t q2)
{
  // Test collisions.
  if(m_localpath_test) // with a localpath
  {
    LocalPath p(q1, q2);
    return(p.getValid());
  }
  else // without a localpath - only test the end configuration
  {
    return(!q2->IsInCollisionWithoutConstraints());
  }
  return(false);
}
//------------------------------------------------------------------------------
template<class Graph>
double RRTStar<Graph>::localpath_cost(confPtr_t q1, confPtr_t q2)
{
  LocalPath p(q1, q2);
  return(localpath_clearance_cost_by_min_dist(p, m_step_length));
}
//------------------------------------------------------------------------------
template<class Graph>
double RRTStar<Graph>::path_cost(Vertex n)
{
  double cost(0);
  Vertex parent = m_graph[n].parent;
  while(parent != NULL)
  {
    cost += m_graph[n].localpath_cost;
    n = parent;
    parent = m_graph[n].parent;
  }
  return(cost);
}
//------------------------------------------------------------------------------
template<class Graph>
std::vector<typename boost::graph_traits<Graph>::vertex_descriptor> RRTStar<Graph>::get_path(Vertex n)
{
  std::vector<Vertex> path;
  path.push_back(n);
  Vertex parent = m_graph[n].parent;
  while(parent != NULL)
  {
    path.push_back(parent);
    n = parent;
    parent = m_graph[n].parent;
  }
  return(path);
}
//------------------------------------------------------------------------------
template<class Graph>
double RRTStar<Graph>::rrg_ball_radius()
{
  if(m_cspace->get_connection_radius_flag())
  {
    double inv_d = 1.0 / m_cspace->dimension();
    double gamma_rrg =
    2 * pow(1.0 + inv_d, inv_d) *
    pow(m_cspace->volume() / m_cspace->unit_sphere(), inv_d);
    double nb_nodes = m_graph[boost::graph_bundle].num_vertices;
    return(std::min(m_cspace->get_step(), gamma_rrg * pow((log(nb_nodes)/nb_nodes), inv_d)));
  }
  else
  {
    return(m_cspace->get_step());
  }
}
//------------------------------------------------------------------------------
template<class Graph>
bool RRTStar<Graph>::is_in_goal_region(confPtr_t q)
{
  if(!m_goal_region_function.empty())
  {
    return(m_goal_region_function(q));
  }
  double dist(cspace_distance(m_goal_region_active_dist ?
                              cspace_dist_active_euclidean :
                              cspace_dist_euclidean, m_robot, q, m_goal_region_configuration));
  if(dist < stat_distance_to_goal_region)
  {
    printf("best distance to goal region: %f\n", dist);
    stat_distance_to_goal_region = dist;
  }
  //printf("distance to goal region: %10f\n", dist);
  return(dist < m_goal_region_radius);
}
//------------------------------------------------------------------------------
// Call this only if m_goal_region == true
template<class Graph>
typename RRTStar<Graph>::Vertex RRTStar<Graph>::extract_best_goal()
{
  if(!m_goal_region_flag) { return(0); }
  double min_cost(std::numeric_limits<double>::max());
  Vertex min_vertex(0);
  BOOST_FOREACH(Vertex v, in_goal)
  {
    if(this->path_cost(v) < min_cost)
    {
      boost_traj = this->get_path(v);
      min_cost = this->path_cost(v);
      min_vertex = v;
    }
  }
  if(min_vertex != 0)
  {
    m_goal = min_vertex;
    //printf("Cost to goal : %f\n", min_cost);
  }
  return(min_vertex);
}
//------------------------------------------------------------------------------
// Predicate used in RRT* to transform the list of neighbouring nodes n_neighbours
// into the list of all nodes n in n_neighbours, excluding n_nearest,
// such that the localpath from n to q_new is collision-free
// This list is used in the first step of RRT* to establish the initial connection between
// the best neigbhour (in terms of path cost) and q_new;
// and in the second step of RRT* to rewire the graph by taking into account
// the new path to q_new.
template<class Graph>
bool RRTStar<Graph>::valid_neighbour(Vertex n_nearest, confPtr_t q_new, Vertex neighbour)
{
  return(neighbour != n_nearest && this->collision_free(m_graph[neighbour].conf, q_new));
}
//------------------------------------------------------------------------------
// Performs one iteration of the standard RRTStar algorithm :
// Sample a random configuration q_new, get the nearest neighbour q_near in from_comp,
// and extend from q_near towards q_new.
template<class Graph>
typename RRTStar<Graph>::Vertex RRTStar<Graph>::expand_simple(ComponentIter from_comp)
{
  Vertex n_new(0);
  bool rrt_star(true);
  ++stat_expand_counter;
  _DEBUG_MSG("expand: iteration %i\n", stat_expand_counter);
  // 3rd argument : sample the passive dofs
  confPtr_t qRand = m_robot->shoot(m_sample_passive);
  Vertex n_nearest = graph_nn_boost(m_robot, from_comp->nodes, qRand, cspace_dist_euclidean, m_graph);
  confPtr_t qNear = m_graph[n_nearest].conf;
  RRTStar<Graph>::extend_return_t extend_rv = this->extend(qNear, qRand, true);
  if(extend_rv.get<status_e>() != trapped)
  {
    // RRT*
    if(rrt_star)
    {
      _DEBUG_MSG("Entering RRT* iteration\n");
      //------------------------------------------------------------------------------
      // Step 1 of 2 :
      // Connect to the neighbour whose path to q_new has the lowest cost
      //------------------------------------------------------------------------------
      confPtr_t q_new = extend_rv.get<data_e>()->conf;
      
      // Compute XnearFree, the list of neighbours that can be connected to q_new
      std::vector<Vertex> XnearFree_unfiltered;
      std::vector<Vertex> XnearFree;
      
      // Add the nearest neighbour
      XnearFree.push_back(n_nearest);
      
      // Add the other candidates
      graph_nn_radius_boost(m_robot, from_comp->nodes, q_new, cspace_dist_euclidean, this->rrg_ball_radius() * m_rrg_radius_factor, m_graph, std::back_inserter(XnearFree_unfiltered));
      
      range_copy_if(XnearFree_unfiltered,
                    std::back_inserter(XnearFree),
                    boost::bind(&RRTStar::valid_neighbour, this, n_nearest, q_new, _1));
      
      _DEBUG_MSG("XnearFree.size() is %d\n", XnearFree.size());
      // For each neighbour in XnearFree, the path cost information is computed.
      // The tuple members, given a node n, and a configuration q_new are :
      // (cost of path from root to q_new by going through n, cost of path to n, cost of localpath from n to q_new, n)
      static const int q_new_cost(0);
      static const int node_cost(1);
      static const int n_to_q_new_cost(2);
      static const int node_pointer(3);
      std::vector<std::tr1::tuple<double, double, double, Vertex> > XnearFreePlusCost;
      for(unsigned nb_index(0); nb_index < XnearFree.size(); ++nb_index)
      {
        Vertex v(XnearFree[nb_index]);
        double cost_to_node(this->path_cost(v));
        double cost_of_lp(m_cspace->lp_cost(m_graph[v].conf, q_new));
        XnearFreePlusCost.push_back(std::tr1::make_tuple(cost_to_node + cost_of_lp, cost_to_node, cost_of_lp, v));
      }
      
      // Get the neighbour n_min whose path has the lowest cost.
      using std::tr1::get;
      const std::tr1::tuple<double, double, double, Vertex>& min_tuple = *std::min(XnearFreePlusCost.begin(), XnearFreePlusCost.end());
      double c_min = get<q_new_cost>(min_tuple);
      Vertex n_min = get<node_pointer>(min_tuple);
      
      // Add n_new(q_new) to the graph, and add the edge n_min -> n_new
      extend_rv.get<data_e>()->localpath_cost = get<n_to_q_new_cost>(min_tuple);
      extend_rv.get<data_e>()->parent = n_min;
      n_new = add_vertex_to_component(*extend_rv.get<data_e>(), n_min, m_graph);
      //------------------------------------------------------------------------------
      // Step 2 of 2 :
      // Rewire the graph among the remaining neighbours to optimize the path cost
      //------------------------------------------------------------------------------
      for(unsigned nb_index(0); nb_index < XnearFreePlusCost.size(); ++nb_index)
      {
        Vertex n_near = get<node_pointer>(XnearFreePlusCost[nb_index]);
        if(n_near != n_min)
        {
          double cost_from_q_new = m_cspace->lp_cost(m_graph[n_new].conf, m_graph[n_near].conf);
          double rewired_cost = c_min + cost_from_q_new;
          if(rewired_cost < get<node_cost>(XnearFreePlusCost[nb_index]))
          {
            if(m_graph[n_near].parent)
            {
              boost::remove_edge(m_graph[n_near].parent, n_near, m_graph);
            }
            boost::add_edge(n_new, n_near, m_graph);
            m_graph[n_near].localpath_cost = cost_from_q_new;
            m_graph[n_near].parent = n_new;
          }
        }
      }
      // Update the goal if needed
      if(m_goal_region_flag && this->is_in_goal_region(m_graph[n_new].conf))
      {
        in_goal.push_back(n_new);
        if(c_min < m_cost_to_goal)
        {
          m_goal = n_new;
          m_cost_to_goal = c_min;
          //printf("cost to goal: %f\n", m_cost_to_goal);
        }
      }
      else
      {
        if(m_goal && m_graph[m_goal].component == m_graph[m_start].component)
        {
          double c = this->path_cost(m_goal);
          if(c < m_cost_to_goal)
          {
            m_cost_to_goal = c;
            //printf("%d %f\n", m_nb_iteration, m_cost_to_goal);
            //printf("cost to goal: %f\n", m_cost_to_goal);
          }
        }
      }
      if(m_draw_trajectory)
      {
        if(m_goal && m_graph[m_goal].component == m_graph[m_start].component)
        {
          boost_traj = this->get_path(m_goal);
        }
      }
    }
    // Standard RRT
    else
    {
      //this->add_node(n_nearest, n_nearest, extend_rv.get<data_e>(), NULL, new_vertices);
    }
  }
  else
  {
    //printf("GNUPLOT %d %d %f\n", m_nb_iteration, m_graph->nbNodes(), 0.0);
  }
  if(plot_data_full_s.is_open())
  {
    plot_data_full_s << "Iteration " << m_nb_iteration <<  " Nodes " << m_graph[m_start].component->num_vertices << " Time " << m_timer.elapsed() << " Cost " << (m_goal ? m_cost_to_goal : 0) << "\n";
  }
  // Also print to std::cout, because why not
  if((m_nb_iteration - 1) % 50 == 0)
  {
    std::cout << m_nb_iteration << " : " << m_graph[m_start].component->num_vertices << " nodes, " << m_timer.elapsed() << " seconds, " << (this->start_and_goal_linked() ? m_cost_to_goal : 0) << "\n";
  }
  if(plot_data_10_s.is_open())
  {
    if(this->start_and_goal_linked() && (m_nb_iteration - 1) % 10 == 0)
    {
      plot_data_10_s << m_timer.elapsed() << " " << m_cost_to_goal << "\n";
    }
  }
  if(plot_data_100_s.is_open())
  {
    if(this->start_and_goal_linked() && (m_nb_iteration - 1) % 100 == 0)
    {
      plot_data_100_s << m_timer.elapsed() << " " << m_cost_to_goal << "\n";
    }
  }
  return(n_new);
}
//-- Stats ---------------------------------------------------------------------
template<class Graph>
void RRTStar<Graph>::stats()
{
  //std::cout << "Statistics : " << std::endl;
  //  std::cout << "Added rising nodes : " << stat_rising_node << std::endl;
  //std::cout << "Total cost computations performed so far : " << global_costSpace->getCount() << std::endl;
  printf("EXPAND: %d\n", stat_expand_counter);
  printf("COL_ACTIVE: %d / %d\n", stat_successful_col_active, stat_successful_col_active + stat_failed_col_active);
  printf("COL_PASSIVE: %d / %d\n", stat_successful_col_passive, stat_successful_col_passive + stat_failed_col_passive);
  printf("CONSTRAINTS: %d / %d\n", stat_active_constraints_successful, stat_active_constraints_successful + stat_active_constraints_failed);
}
//------------------------------------------------------------------------------
// Update and draw the robot.
// If DEBUG_GRAPHIC is true, pause for some time after drawing,
// for debugging purposes.
template<class Graph>
void RRTStar<Graph>::draw(Configuration& q)
{
  if(ENV.getBool(Env::drawGraph) || ENV.getBool(Env::drawRobot) || ENV.getBool(Env::drawTraj))
  {
    {
      m_robot->setAndUpdateWithoutConstraints(q);
      fct_draw();
    }
    if(m_drawing_delay > 0)
    {
      usleep(m_drawing_delay);
    }
  }
}

void draw_boost_static()
{
  typedef boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS, RRTStarNodeData<Vertex> , RRTStarEdgeData> Graph;
  draw_boost_graph<Graph>(boost_robot, *(Graph*)boost_graph);
  if(boost_traj.size() > 1)
  {
    draw_boost_path<Graph>(boost_robot, boost_traj, *(Graph*)boost_graph);
  }
}

template class RRTStar<boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS, RRTStarNodeData<Vertex> , RRTStarEdgeData, RRTStarGraphData<Vertex> > >;
