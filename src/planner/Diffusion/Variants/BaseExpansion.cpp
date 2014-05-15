/*
 * BaseExpansion.cpp
 *
 *  Created on: Jun 12, 2009
 *      Author: jmainpri
 */


#include "BaseExpansion.hpp"

#include "API/ConfigSpace/localpath.hpp"
#include "API/Roadmap/compco.hpp"
#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

BaseExpansion::BaseExpansion() :
    m_ExpansionNodeMethod(NEAREST_EXP_NODE_METH),
    m_MaxExpandNodeFailure(10),
    m_kNearestPercent(10),
    m_ExpansionDirectionMethod(GLOBAL_CS_EXP),
    m_IsDirSampleWithRlg(false),
    m_fromComp(NULL),
    m_toComp(NULL)
{ 
    cout << "no graph in expansion method" << endl;
}

BaseExpansion::BaseExpansion(Graph* ptrGraph) :

    m_ExpansionNodeMethod(NEAREST_EXP_NODE_METH),
    m_MaxExpandNodeFailure(10),
    m_kNearestPercent(10),
    m_ExpansionDirectionMethod(GLOBAL_CS_EXP),
    m_IsDirSampleWithRlg(false),
    m_Graph(ptrGraph),
    m_fromComp(NULL),
    m_toComp(NULL)
{
    cout << "BaseExpansion::step() = " << step() << endl;
}

BaseExpansion::~BaseExpansion()
{

}

/**
 * Returns a distance define by the environement
 */
double BaseExpansion::step()
{
    return ( ENV.getDouble(Env::dmax) * ENV.getDouble(Env::extensionStep) );
}

double BaseExpansion::positionAlongPath( LocalPath& path, double param )
{
    return path.getParamMax() <= 0. ? 1. : MIN( 1., param / path.getParamMax() );
}

/**
 * Return the path parameter that is the closest to step()
 */
double BaseExpansion::pathDelta( LocalPath& path )
{
    return path.getParamMax() <= 0. ? 1 : MIN( 1., step() / path.getParamMax() );;
}

/**
 * Returns the localpath of length equal or inferior to step
 */
pathPtr_t BaseExpansion::getExtensiontPath( confPtr_t qi, confPtr_t qf, double max_param )
{
    bool is_connect = ENV.getExpansionMethod() == Env::Connect;
    LocalPath directionLocalpath( qi, qf );

    double delta;

    if( max_param < 0 )
    {
        delta = is_connect ? 1. : pathDelta( directionLocalpath ) ;
    }
    else {
        double param_delta = positionAlongPath( directionLocalpath, max_param ) ;
        delta = MIN( pathDelta( directionLocalpath ), param_delta ); // TODO check if that is ok for RRT*
        // delta = param_delta;
    }

    return pathPtr_t( new LocalPath( directionLocalpath, delta , is_connect ) );
}

/**
 * Expnasion controls that the exploring the CSpace
 */
bool BaseExpansion::expandControl( LocalPath& path, Node& compNode )
{
    double radius=0;

    if(!ENV.getBool(Env::useRefiRadius))
    {
        if( ENV.getExpansionMethod() == Env::Extend || ENV.getExpansionMethod() == Env::costConnect )
        {
            radius = step();
        }
        //		if(ENV.getExpansionMethod() == Env::costConnect)
        //		{
        //			radius = compNode.getComp()->sumLengthEdges / (compNode.getComp()->nnode-1);
        //		}
    }
    else
    {
        radius = ENV.getDouble(Env::refiRadius)*ENV.getDouble(Env::dmax);
    }

    ConnectedComponent* compco = compNode.getConnectedComponent();

    double ratio = double(compco->getCompcoStruct()->nbRefinNodes) / double(compco->getNumberOfNodes());

    if( ENV.getBool(Env::printRadius) ){
        cout << "radius = " << radius << endl;
        cout << "path.length() = " << path.getParamMax() << endl;
        cout << "compco->getCompcoStruct()->nbRefinNodes = " << compco->getCompcoStruct()->nbRefinNodes << endl;
        cout << "compco->getNumberOfNodes() = " << compco->getNumberOfNodes() << endl;
        cout << "ratio of RNODES = " << ratio << endl;
        cout << endl;
    }

    if( path.getParamMax() <= radius )
    {
        if( ratio > 0.10 )
        {
            return(false);
        }
        else
        {
            compco->getCompcoStruct()->nbRefinNodes++;
        }
    }
    return(true);
}

/**
 * Function to be called when
 * an expansion fails
 */
void BaseExpansion::expansionFailed(Node& node) 
{
    if(m_ExpansionNodeMethod == RANDOM_IN_SHELL_METH)
    {
        p3d_SetNGood(0);
    }

    node.getNodeStruct()->n_fail_extend++;

    if((ENV.getBool(Env::discardNodes)) &&
            (node.getNodeStruct() != m_Graph->getGraphStruct()->search_start) &&
            (node.getNodeStruct() != m_Graph->getGraphStruct()->search_goal) &&
            (node.getNodeStruct()->n_fail_extend > m_MaxExpandNodeFailure))
    {
        node.getNodeStruct()->IsDiscarded = true;
        update_parent_nfails(node.getNodeStruct());

        m_Graph->getGraphStruct()->n_consec_pb_level ++;
        m_Graph->getGraphStruct()->n_consec_fail_pb_level ++;
    }
}

//((pathDelta < 0.03 || (pathDelta > 0.8 && path.length() < step))))
//      ((pathDelta < 0.1 || (
//  path.length() < step && pathDelta > 0.8 && !(expMethod == Env::nExtend &&
//						 extendStep != 1)))))// && p3d_GetIsExpandControl() && !directionNode && expandControl(N.mN)))

/**
 * Gives successive co
 */
bool BaseExpansion::nextStep(LocalPath& path, Node* directionNode, double& delta, shared_ptr<LocalPath>& newPath, Env::expansionMethod method)
{

    if( method == Env::Connect )
    {
        //cout << "create path satisfying connect method" << endl;
        newPath = shared_ptr<LocalPath>(new LocalPath( path, delta ));

        if( delta == 0.)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        confPtr_t ptrEnd;
        delta = pathDelta( path );

        if( delta == 1. && directionNode)
        {
            ptrEnd = directionNode->getConfiguration();
        }
        else
        {
            ptrEnd = path.configAtParam( delta * path.getParamMax() );
        }
        ptrEnd->setConstraints();
        newPath = shared_ptr<LocalPath>(new LocalPath(path.getBegin(),ptrEnd));
    }

    return(newPath->isValid());
}

/**
 * Gives successive co
 */
bool BaseExpansion::nextStep(LocalPath& path, confPtr_t& directionConfig, double& pathDelta, shared_ptr<LocalPath>& newPath, Env::expansionMethod method)
{

    if(method == Env::Connect)
    {
        cout << "Error : Method Doesn't work with connect" << endl;
    }
    else
    {
        pathDelta = path.getParamMax() <= 0. ? 1. : MIN(1., this->step() / path.getParamMax());

        newPath = shared_ptr<LocalPath>(
                    new LocalPath(path.getBegin(),
                                  pathDelta == 1. && directionConfig ?
                                      directionConfig :
                                      path.configAtParam(pathDelta * path.getP3dLocalpathStruct()->range_param)));
    }

    return(newPath->isValid());
}

/**
 * Function that adds a node to the graph
 */
Node* BaseExpansion::addNode(Node* currentNode, LocalPath& path, double pathDelta, Node* directionNode, int& nbCreatedNodes)
{
    if ((pathDelta == 1. && directionNode))
    {
        //cout << "MergeComp" << endl;
        m_Graph->linkNodeAndMerge( currentNode, directionNode, false );
        return (directionNode);
    }
    else
    {
        //cout << "insertNode" << endl;
        Node* newNode = m_Graph->insertNode( currentNode, path );
        nbCreatedNodes++;

        return (newNode);
    }
}
