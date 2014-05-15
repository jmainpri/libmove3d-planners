#ifndef RRG_HPP
#define RRG_HPP

/**
 * @ingroup Diffusion
 *
 * This class implements a RRT
 */

#include "planner/Diffusion/RRT.hpp"
#include "planner/Diffusion/Variants/Threshold-RRT.hpp"

#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/cspace.hpp"
#include "API/Roadmap/compco.hpp"
#include "API/Trajectory/trajectory.hpp"

namespace Move3D
{

/**
 @ingroup Diffusion
 */
class RRGExpansion : public RRTExpansion
{
public:
    /**
     * Constructors with default values
     */
    RRGExpansion( Graph* G );

    /**
   * Set the init and goal
   */
    void setInitAndGoal( confPtr_t q_init, confPtr_t q_goal );

    /**
   * Set the CSpace distance
   */
    void initCSpace();

    /**
     * Destructor
     */
    ~RRGExpansion();

    /**
     * Checks that the localpath
     * between epansionNode and directionConfig can expanded in the costspace
     */
    bool expandToGoal( Node* expansionNode, confPtr_t directionConfig );

    /**
     * expandProcess
     *
     * checks the validity of the local path in one direction and adds nodes
     * to the trees with a different behaviour depending on the method variable
     *
     * @param expansionNode
     * @param directionConfig
     * @param directionNode
     * @param method
     *
     * @return the number of nodes created
     */
    unsigned expandProcess( Node* expansionNode, confPtr_t directionConfig, Node* directionNode, Env::expansionMethod method);

    /**
   * Set the initial compco
   */
    void setInitialCompco( ConnectedComponent* compco );

private:

    /**
     * Get the extepansion direction
     */
    confPtr_t getExpansionDirection( Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode);

    /**
     * Connect expansion method
     */
    int connectExpandProcess( Node* expansionNode, confPtr_t directionConfig, Node* directionNode );

    /**
     * Extend expansion method
     */
    int extendExpandProcess( Node* expansionNode, confPtr_t directionConfig, Node* directionNode);

    /**
   * Get the rgg ball raduis
   */
    double rrgBallRadius();

    int m_K_Nearest;
    double m_RrgRadiusFactor;
    double m_step;
    int m_iteration;

    ConnectedComponent* m_compco;

    // cspace
    CSpace* m_cspace;

    confPtr_t m_q_init;
    confPtr_t m_q_goal;

    bool m_compute_edge_cost;
};

class RRG : public RRT
{

public:
    /**
     * Constructor from a WorkSpace object
     * @param WS the WorkSpace
     */
    RRG(Robot* R, Graph* G);

    /**
     * Destructor
     */
    virtual ~RRG();

    /**
     * Initialzation of the plannificator
     * @return the number of node added during the init phase
     */
    virtual unsigned init();

    /**
     * Connects the two connected components
   * if the seconed is within a certain reach and there exists
   * a collision free path
     */
    bool connectNodeToCompco(Node* N, Node* CompNode);

private:

    /**
    * Exctracts the trajectory if it exists
    * this function is called when a connection to goal is found
    */
    void extractTrajectory();

    /**
    * save convergence to file
    */
    void saveConvergenceToFile();

    Move3D::Trajectory* m_current_traj;
    std::vector< Move3D::Trajectory > m_pareto_trajs;
    std::vector< std::pair<double,TrajectoryStatistics> > m_convergence_rate;
};

}

#endif // RRG_HPP
