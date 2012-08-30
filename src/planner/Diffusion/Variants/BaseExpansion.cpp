/*
 * BaseExpansion.cpp
 *
 *  Created on: Jun 12, 2009
 *      Author: jmainpri
 */


#include "BaseExpansion.hpp"

#include "API/planningAPI.hpp"

#include "Roadmap/compco.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

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
{}

BaseExpansion::~BaseExpansion(){}

/**
 * Returns a distance define by the environement
 */
double BaseExpansion::step()
{
	return(p3d_get_env_dmax() * ENV.getDouble(Env::extensionStep));
}

double BaseExpansion::positionAlongPath(LocalPath& path, double param)
{
	return					path.getParamMax() <= 0. ? 1. : 
	MIN(1., param / path.getParamMax() );
}

/**
 * Return the path parameter that is 
 * the closest to step()
 */
double BaseExpansion::pathDelta(LocalPath& path)
{
	double pathDeta =	path.getParamMax() == 0. ? 1 : 
	MIN(1., step() /  path.getParamMax() );	
	
	return pathDeta;
}

/**
 * Returns the localpath of length
 * equal or inferior to step
 */
LocalPath BaseExpansion::getExtensiontPath( confPtr_t qi, confPtr_t qf )
{
	LocalPath directionLocalpath(qi,qf);
	
	double pathDelta = directionLocalpath.getParamMax() == 0. ? 1 
	: MIN(1., step() / directionLocalpath.getParamMax() );
	
	LocalPath extensionLocalpath( directionLocalpath, pathDelta , false );
	return extensionLocalpath;
}

/**
 * Expnasion controls that
 * the exploring the CSpace
 */
bool BaseExpansion::expandControl( LocalPath& path, Node& compNode )
{
	double radius=0;
	
	if(!ENV.getBool(Env::useRefiRadius))
	{
		if( ENV.getExpansionMethod() == Env::Extend || ENV.getExpansionMethod() == Env::costConnect)
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
		radius = ENV.getDouble(Env::refiRadius)*p3d_get_env_dmax();
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
bool BaseExpansion::nextStep(LocalPath& path,
														 Node* directionNode,
														 double& pathDelta,
														 shared_ptr<LocalPath>& newPath,
														 Env::expansionMethod method)
{
	
	if( method == Env::Connect )
	{
		//cout << "create path satisfying connect method" << endl;
		newPath = shared_ptr<LocalPath>(new LocalPath(path, pathDelta));
		
		if(pathDelta == 0.)
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
		pathDelta = path.getParamMax() == 0. ? 1. : MIN(1., step() / path.getParamMax());
		shared_ptr<Configuration> ptrEnd;
		
		if(pathDelta == 1. && directionNode)
		{
			ptrEnd = directionNode->getConfiguration();
		}
		else
		{
			ptrEnd = path.configAtParam(pathDelta * path.getLocalpathStruct()->range_param);
		}
		ptrEnd->setConstraints();
		newPath = shared_ptr<LocalPath>(new LocalPath(path.getBegin(),ptrEnd));
	}
	
	return(newPath->isValid());
}

/**
 * Gives successive co
 */
bool BaseExpansion::nextStep(LocalPath& path,
														 shared_ptr<Configuration>& directionConfig,
														 double& pathDelta,
														 shared_ptr<LocalPath>& newPath,
														 Env::expansionMethod method)
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
																									path.configAtParam(pathDelta * path.getLocalpathStruct()->range_param)));
	}
	
	return(newPath->isValid());
}

/**
 * Function that adds a node to the graph
 */
Node* BaseExpansion::addNode(Node* currentNode, 
														 LocalPath& path, 
														 double pathDelta,
														 Node* directionNode, 
														 int& nbCreatedNodes)
{
	if ((pathDelta == 1. && directionNode))
	{
		//cout << "MergeComp" << endl;
		m_Graph->linkNodeAndMerge( currentNode, directionNode );
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
