#include "RRTExpansion.hpp"

#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Roadmap/compco.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/Grids/PointCloud.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE


RRTExpansion::RRTExpansion() :
    BaseExpansion()
{
}

RRTExpansion::RRTExpansion(Graph* ptrGraph) :
    BaseExpansion(ptrGraph)
{
}

RRTExpansion::~RRTExpansion()
{
}

confPtr_t RRTExpansion::getExpansionDirection( Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode)
{
    if (p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH)
    {

        confPtr_t q = m_Graph->getRobot()->shootDir(samplePassive);

        p3d_addConfig(m_Graph->getRobot()->getP3dRobotStruct(),
                      q->getConfigStruct(),
                      expandComp->getConnectedComponent()->getCompcoStruct()->dist_nodes->N->q,
                      q->getConfigStruct());

        return q;

    }

    confPtr_t q;
    //int savedRlg;
    // if (m_IsDirSampleWithRlg)
    // {
    //    // Save the previous Rlg setting to shoot without Rlg
    //     savedRlg = p3d_get_RLG();
    //     p3d_set_RLG(false);
    // }

    // Selection in the entire CSpace and
    // biased to the Comp of the goal configuration
    if (ENV.getBool(Env::isGoalBiased) && p3d_random(0., 1.) <= ENV.getDouble(Env::Bias))
    {
        // select randomly a node in the goal component as direction of expansion
        directionNode = m_Graph->randomNodeFromComp(goalComp);
        q = directionNode->getConfiguration();
    }
    else
    {
        switch (m_ExpansionDirectionMethod)
        {
        case SUBREGION_CS_EXP:
            // Selection in a subregion of the CSpace
            // (typically close to the current tree)
            // and  biased to the goal configuration
            q = confPtr_t ( new Configuration(m_Graph->getRobot() ) );

#ifdef P3D_PLANNER
            p3d_shoot_inside_box(m_Graph->getRobot()->getP3dRobotStruct(),
                                 /*expandComp->getConfiguration()->getConfigStruct(),*/
                                 q->getConfigStruct(),
                                 expandComp->getConnectedComponent()->getCompcoStruct()->box_env_small,
                                 (int) samplePassive);
#else
            printf("P3D_PLANNER not compiled in %s in %s",__PRETTY_FUNCTION__,__FILE__);
#endif

            break;
#ifdef LIGHT_PLANNER
        case NAVIGATION_BEFORE_MANIPULATION:
        {
            m_Graph->getRobot()->deactivateCcConstraint();

            q = m_Graph->getRobot()->shoot();
            Node* closest = m_Graph->nearestWeightNeighbour(expandComp,q,false,ONLY_ROBOT_BASE);

            LocalPath path(closest->getConfiguration(),q);
            //				cout << "Param max = " << path.getParamMax() << " , step = " << step() << endl;
            q = path.configAtParam(2*step());

            //				cout << "1 :" << endl;
            //				q->print();

            m_Graph->getRobot()->setAndUpdate(*q);
            q = m_Graph->getRobot()->shootAllExceptBase();
            q->setConstraintsWithSideEffect();

            //				cout << "2 :" << endl;
            //				q->print();

            //				m_Graph->getRobot()->setAndUpdate(*q);
            //				g3d_draw_allwin_active();

            m_Graph->getRobot()->activateCcConstraint();
            //				q->setConstraintsWithSideEffect();
            //				m_Graph->getRobot()->setAndUpdate(*q);
            //				g3d_draw_allwin_active();
        }
            break;
#endif
        case GLOBAL_CS_EXP:
            
        default:
            // Selection in the entire CSpace
            q = m_Graph->getRobot()->shoot(samplePassive);
        }
    }
    // Todo fix
    //    if (!m_IsDirSampleWithRlg)
    //    {
    //        // Restore the previous Rlg setting
    //        p3d_set_RLG( savedRlg );
    //    }
    return (q);
}

Node* RRTExpansion::getExpansionNode(Node* compNode, shared_ptr<Configuration> direction, int distance)
{
    //    cout << "Distance == " << distance << endl;

    if (p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH)
    {
        return m_Graph->getNode(compNode->getConnectedComponent()->getCompcoStruct()->dist_nodes->N);
    }

    //    int KNearest = -1;
    //    int NearestPercent;

    switch (distance)
    {

    case NEAREST_EXP_NODE_METH:
        /* Choose the nearest node of the componant*/
        return (m_Graph->nearestWeightNeighbour( compNode, direction, p3d_GetIsWeightedChoice(), distance));

    case K_NEAREST_EXP_NODE_METH:
        /* Select randomly among the K nearest nodes of a componant */
        //NearestPercent = m_kNearestPercent;
        // KNearest = MAX(1,(int)((NearestPercent*(compNode->getConnectedComponent()->getNumberOfNodes()))/100.));
        // TODO : fix
        //   ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ,
        // KNearest);

        return (m_Graph->nearestWeightNeighbour( compNode, direction, p3d_GetIsWeightedChoice(), distance));

    case BEST_SCORE_EXP_METH:
        /* Select the node which has the best score: weight*dist */
        return (m_Graph->nearestWeightNeighbour( compNode, direction, p3d_GetIsWeightedChoice(), distance));

    case K_BEST_SCORE_EXP_METH:
       // NearestPercent = m_kNearestPercent;
        //KNearest = MAX(1,(int)((NearestPercent*(compNode->getConnectedComponent()->getNumberOfNodes()))/100.));
        // TODO : fix
        // ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ, KNearest);
        return (m_Graph->nearestWeightNeighbour( compNode, direction, p3d_GetIsWeightedChoice(), distance));

    case RANDOM_IN_SHELL_METH:
        /* Select randomly among all the nodes inside a given portion of shell */
        return (m_Graph->getNode(hrm_selected_pb_node(m_Graph->getGraphStruct(), direction->getConfigStruct(), compNode->getNodeStruct()->comp)));

    case RANDOM_NODE_METH:
        return (m_Graph->getNode(p3d_RandomNodeFromComp(
                                     compNode->getConnectedComponent()->getCompcoStruct())));

        /*case NAVIGATION_BEFORE_MANIPULATION:
    {
        Node* node = m_Graph->nearestWeightNeighbour(compNode,direction,p3d_GetIsWeightedChoice(), distance);

        LocalPath path(node->getConfiguration(),direction);
        double pathDelta = path.getParamMax() <= 0. ? 1. : MIN(1., this->step() / path.getParamMax());

        shared_ptr<Configuration> newDirection = path.configAtParam(pathDelta);
        m_Graph->getRobot()->shootObjectJoint(*newDirection);

        int ObjectDof = m_Graph->getRobot()->getObjectDof();

        (*direction)[ObjectDof+0] = (*newDirection)[ObjectDof+0];
        (*direction)[ObjectDof+1] = (*newDirection)[ObjectDof+1];
        (*direction)[ObjectDof+2] = (*newDirection)[ObjectDof+2];
        (*direction)[ObjectDof+3] = (*newDirection)[ObjectDof+3];
        (*direction)[ObjectDof+4] = (*newDirection)[ObjectDof+4];
        (*direction)[ObjectDof+5] = (*newDirection)[ObjectDof+5];

        if (ENV.getBool(Env::drawPoints)) {

            Eigen::Vector3d point;
            point[0] = (*direction)[ObjectDof+0];
            point[1] = (*direction)[ObjectDof+1];
            point[2] = (*direction)[ObjectDof+2];

            PointsToDraw->push_back(point);
        }

        return node;
    }*/

    default:
        /* By default return the nearest node of the componant */
        return (m_Graph->nearestWeightNeighbour(compNode, direction,
                                                p3d_GetIsWeightedChoice(), distance));
    }
}

bool RRTExpansion::expandToGoal(Node* expansionNode,
                                MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig)
{
    return false;
}


unsigned RRTExpansion::expandProcess(Node* expansionNode, confPtr_t directionConfig, Node* directionNode,
                                     Env::expansionMethod method)
{
    bool extensionSucceeded(false);
    bool failed(false);
    int nbCreatedNodes(0);
    Node* fromNode = expansionNode;
    Node* extensionNode(NULL);
    shared_ptr<LocalPath> directionLocalpath;
    double positionAlongDirection(0.);
    shared_ptr<LocalPath> extensionLocalpath;
    bool firstIteration(true);

    // Perform extension toward directionConfig
    // Additional nodes creation in the nExtend case, but without checking for expansion control
    while (firstIteration || (method == Env::nExtend && !failed
                              && positionAlongDirection < 1.))
    {
        directionLocalpath = shared_ptr<LocalPath> (new LocalPath(
                                                        fromNode->getConfiguration(), directionConfig));

        // Expand one step along the local path "extensionLocalpath"
        extensionSucceeded = nextStep(*directionLocalpath, directionNode,
                                      positionAlongDirection, extensionLocalpath, method);


        failed |= !extensionSucceeded;

        // Expansion Control
        if (firstIteration && !failed)
        {
            if (ENV.getBool(Env::expandControl)
                    && !this->expandControl (*directionLocalpath, *expansionNode ))
            {
                failed = true;
            }
        }
        // Add node to graph if everything succeeded
        if (!failed)
        {
            extensionNode = addNode( fromNode, *extensionLocalpath,
                                     positionAlongDirection, directionNode,
                                     nbCreatedNodes);

            m_last_added_node = extensionNode;
        }
        if (firstIteration && failed)
        {
            expansionFailed(*expansionNode);
        }
        if (!failed)
        {
            fromNode = extensionNode;
        }
        firstIteration = false;
        //
    }

    return nbCreatedNodes;
}
