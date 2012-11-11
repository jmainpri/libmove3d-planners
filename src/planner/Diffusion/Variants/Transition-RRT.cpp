/*
 * RRT-Transition.cpp
 *
 *  Created on: Jul 27, 2009
 *      Author: jmainpri
 */

#include "Transition-RRT.hpp"
#include "planner/Diffusion/Variants/Transition-RRT.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/compco.hpp"
#include "API/Roadmap/graph.hpp"

#include "API/ConfigSpace/localpath.hpp"

#include "planEnvironment.hpp"
#include "cost_space.hpp"

#ifdef HRI_COSTSPACE
#include "HRI_costspace/HRICS_Workspace.hpp"
#endif

#include "Localpath-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

using namespace Eigen;

TransitionExpansion::TransitionExpansion() :
RRTExpansion()
{
}

TransitionExpansion::TransitionExpansion(Graph* ptrGraph) :
RRTExpansion(ptrGraph)
{
}

TransitionExpansion::~TransitionExpansion()
{
}

void TransitionExpansion::setInitAndGoalNodes( Node* init, Node* goal )
{
  m_initNode = init;
  m_goalNode = goal;
}

shared_ptr<Configuration> TransitionExpansion::computeGradient(shared_ptr<Configuration> q)
{	
	vector< pair<double,shared_ptr<Configuration> > > vect(8);
	
	for (unsigned int i=0; i<vect.size(); i++) 
	{
		shared_ptr<Configuration> ptrQ(new Configuration(*q));
		ptrQ->setCostAsNotTested();
		
		vect[i].first = 0.0;
		vect[i].second = ptrQ;
	}
	
	unsigned int i=0;
	
	(*vect[i].second)(6) = (*q)(6) + step();
	(*vect[i].second)(7) = (*q)(7);
	
	vect[i].first = vect[i].second->cost();
	
	i++;
	
	(*vect[i].second)(6) = (*q)(6);
	(*vect[i].second)(7) = (*q)(7) + step();
	
	vect[i].first = vect[i].second->cost();
	
	i++;
	
	(*vect[i].second)(6) = (*q)(6) - step();
	(*vect[i].second)(7) = (*q)(7);
	
	vect[i].first = vect[i].second->cost();
	
	i++;
	
	(*vect[i].second)(6) = (*q)(6);
	(*vect[i].second)(7) = (*q)(7) - step();
	
	vect[i].first = vect[i].second->cost();	
	
	i++;
	
	(*vect[i].second)(6) = (*q)(6) + step();
	(*vect[i].second)(7) = (*q)(7) + step();
	
	vect[i].first = vect[i].second->cost();
	
	i++;
	
	(*vect[i].second)(6) = (*q)(6) - step();
	(*vect[i].second)(7) = (*q)(7) - step();
	
	vect[i].first = vect[i].second->cost();
	
	i++;
	
	
	(*vect[i].second)(6) = (*q)(6) + step();
	(*vect[i].second)(7) = (*q)(7) - step();
	
	vect[i].first = vect[i].second->cost();
	
	i++;
	
	(*vect[i].second)(6) = (*q)(6) - step();
	(*vect[i].second)(7) = (*q)(7) + step();
	
	vect[i].first = vect[i].second->cost();
	
	sort(vect.begin(),vect.end());
	
	//	for (unsigned int i=0; i<vect.size(); i++) 
	//	{
	//		//shared_ptr<Configuration> ptrQ(new Configuration(*q));
	//		cout << "Cost : " << i << " = " << vect[i].first << endl;
	//		cout << "Cost : " << i << " = " << vect[i].second->cost() << endl;
	//		vect[i].second->print();
	//		//vect[i].second = ptrQ;
	//	}
	//	cout << endl;
	
	//	cout << "q = " << endl;
	//	q->print();
	
	//	cout << "vect[0].second = " << endl;
	//	vect[0].second->print();
	
	Eigen::VectorXd direction = vect[0].second->getEigenVector() - q->getEigenVector();
	
	shared_ptr<Configuration> q_grad(new Configuration(vect[0].second->getRobot()));
	
	//	cout << "q_grad = " << endl;
	//	q_grad->print();
	
	direction.normalize();
	q_grad->setFromEigenVector(direction);
	
	return q_grad;
}

bool TransitionExpansion::mixWithGradient(Node* expansionNode,shared_ptr<Configuration> directionConfig)
{	
	shared_ptr<Configuration> q_grad = computeGradient(expansionNode->getConfiguration());
	shared_ptr<Configuration> q_goal = getToComp()->getConfiguration();
	
	Eigen::VectorXd ExpansionVector = expansionNode->getConfiguration()->getEigenVector();
	
	Eigen::VectorXd Voronoi = ( directionConfig->getEigenVector() - ExpansionVector ).normalized() ;
	Eigen::VectorXd Gradiant = q_grad->getEigenVector().normalized();
	
	
	//		cout << "Voronoi : " << endl;
	//		cout << Voronoi << endl;
	//		cout << "Gradiant : " << endl;
	//		cout << Gradiant << endl;
	//		cout << "-----------------" << endl;
	
	//		double Norm = Voronoi.norm();
	const unsigned int Discretization = 10;
	const double delta = (1/(double)Discretization);
	double k=0.0;
	
	for (unsigned int i=0; i<Discretization; i++) 
	{
		Eigen::VectorXd NewDirection = k*Voronoi + (1-k)*Gradiant;
		NewDirection.normalize();
		NewDirection *= 1.1*step();
		
		//			cout << "k = " << k << endl;
		//			cout << NewDirection << endl;
		//			cout << "-----------------" << endl;
		
		Eigen::VectorXd q_rand = NewDirection+ExpansionVector;
		
		//		cout << "k = " << k << endl;
		//		cout << "q_rand = " << endl << q_rand << endl;
		//		cout << "-----------------" << endl;
		
		directionConfig->setFromEigenVector(q_rand);
		
		if ((!directionConfig->isOutOfBounds()) && 
				(directionConfig->dist(*q_goal) < expansionNode->getConfiguration()->dist(*q_goal)))
		{
			break;
		}
		
		if (i>=(Discretization-1)) 
		{
			//			cout << "return false;" << endl;
			return false;
		}
		//			cout << "delta = " << delta << endl;
		k += delta;
	}
	
	//cout << "Direction = " << endl;
	//	directionConfig->print();
	//	cout << "Return true" << endl;
	return true;
}

/**
 * Gets the direction
 */
shared_ptr<Configuration> TransitionExpansion::getExpansionDirection(
																																		 Node* expandComp, Node* goalComp, bool samplePassive,
																																		 Node*& directionNode)
{
	shared_ptr<Configuration> q;
	
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
		q = m_Graph->getRobot()->shoot(samplePassive);
	}
	return q;
}


/**
 * CostTestSucceeded
 * Transition Test function to validate the feasability of the motion
 * from the current config with the current cost in function
 * of the previous config and cost
 * this test is currently based on the Metropolis Criterion
 * also referred as the Boltzmann probability when applied to
 * statistical physics or molecular modeling
 * The temperature parameter is adaptively tuned  in function of the
 * failures and successes during the search process.
 * This adaptation can be local to each node or applied globaly to
 * the entire graph.
 * @param[In] G: pointer to the robot graph
 * @param[In] previousNodePt: pointer to the previous node
 * @param[In] currentConfig: current confguration
 * @param[In] PreviousCost: Previous cost (i.e. cost
 * of the previous config)
 * @param[In] CurrentCost: Current node cost
 * @param[In] IsRaisingCost: Give the direction of the
 * cost. TRUE if its easy to succeed test for increasing costs
 * (i.e from goal to init) FALSE otherwise.
 * @return: TRUE if the test succeeded, FALSE otherwise
 */

int Nb_succes = 0;

bool TransitionExpansion::costTestSucceeded(Node* previousNode, shared_ptr<
                                            Configuration> currentConfig, double currentCost)
{
  //	p3d_node* previousNodePt(previousNode->getNodeStruct());
	double ThresholdVal;
	double dist;
	bool success(false);
  //	configPt previousConfig = previousNodePt->q;
	double temperature;
	// TODO : for Urmson transition
	// double cVertex, cOpt, cMax;
	// double minThreshold = 0.05;
	
  //new simplified test for down hill slopes
  if (currentCost <= previousNode->cost() )
  {
    return true;
  }
  
  //  GlobalNbDown =0;
  // previousNodePt->NbDown =0;
  
  // In principle, the distance are not necessarly
  // reversible for non-holonomic robots
  dist = p3d_dist_q1_q2(m_Graph->getRobot()->getRobotStruct(),
                        currentConfig->getConfigStruct(), previousNode->getConfiguration()->getConfigStruct() );
  // dist = p3d_dist_q1_q2(mR, previousConfig, currentConfig);
  
  // get the value of the auto adaptive temperature.
  temperature = p3d_GetIsLocalCostAdapt() ? previousNode->getNodeStruct()->temp
  : previousNode->getConnectedComponent()->getCompcoStruct()->temperature;
  
  if (p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH)
  {
    temperature = 0.001 * ENV.getDouble(Env::alpha) * ENV.getInt(
                                                                 Env::maxCostOptimFailures);
  }
  
  //Metropolis criterion (ie Boltzman probability)
  //    ThresholdVal = exp((PreviousCost-CurrentCost)/(temperature*dist));
  ThresholdVal = exp((previousNode->cost() - currentCost) / temperature);
  
  //    success = ThresholdVal > 0.5;
  success = p3d_random(0., 1.) < ThresholdVal;
  
  //  p3d_EvaluateExpandDiffic(previousNodePt->comp, success);
  //  cout << "No transition method selected" << endl;
  
  if (ENV.getBool(Env::printTemp))
  {
    cout << temperature << "\t" << previousNode->cost() << "\t"
    << currentCost << endl;
    
    if (success)
    {
      cout << "-----------------------------------------------" << endl;
      cout << "Nb_succes = " << Nb_succes++ << endl;
      cout << "-----------------------------------------------" << endl;
    }
	}
	
	if (previousNode->equalCompco(m_initNode))
	{
		ENV.setDouble(Env::temperatureStart, temperature);
	}
	else
	{
		ENV.setDouble(Env::temperatureGoal, temperature);
	}
	
	return success;
}

bool TransitionExpansion::costTestSucceededConf(
																								shared_ptr<Configuration>& previousConfig,
																								shared_ptr<Configuration>& currentConfig)
{
	
	double previousCost = previousConfig->cost();
	double currentCost = currentConfig->cost();
	//new simplified test for down hill slopes
	if (currentCost <= previousCost)
	{
		return true;
	}
	// ATTENTION HERE!!!
	else
	{
		return false;
	}
	
	double temperature = 0;
	
	// Metropolis criterion (ie Boltzman probability)
	// ThresholdVal = exp((PreviousCost-CurrentCost)/(temperature*dist));
	double ThresholdVal = exp((previousCost - currentCost) / temperature);
	
	// success = ThresholdVal > 0.5;
	bool success = p3d_random(0., 1.) < ThresholdVal;
	
	if (ENV.getBool(Env::printTemp))
	{
		cout << temperature << "\t" << previousCost << "\t" << currentCost << endl;
		
	}
	
	return success;
}

bool TransitionExpansion::expandToGoal(Node* expansionNode, confPtr_t directionConfig)
{
	bool extensionSucceeded(true);
	double param(0);
	//double temperature = expansionNode->getCompcoStruct()->temperature;
	double extensionCost(0.);
	
	confPtr_t fromConfig = expansionNode->getConfiguration();
	confPtr_t toConfig;
	
	LocalPath directionLocalPath( fromConfig, directionConfig );
  
	double expansionCost = fromConfig->cost();
	
	double paramMax = directionLocalPath.getParamMax();
	
	// Perform extension toward directionConfig
	// Additional nodes creation in the nExtend case, but without checking for expansion control
	for (int i = 1; param < paramMax; i++)
	{
		param = ((double) i) * step();
		
		if (param > paramMax)
			toConfig = directionConfig;
		else
			toConfig = directionLocalPath.configAtParam(param);
		
		if (ENV.getBool(Env::isCostSpace))
		{
			extensionCost = toConfig->cost();
			
			if (!(expansionCost >= extensionCost))
				return false;
		}
		else
			return true;

		expansionCost = extensionCost;
		fromConfig = toConfig;
	}
	
	return extensionSucceeded;
}


/**
 * Expands the tree in a direction checking the cost
 * with every step
 */
bool TransitionExpansion::expandCostConnect(Node& expansionNode, shared_ptr<
                                            Configuration> directionConfig, Node* directionNode, bool toGoal)
{
	bool failed(false);
	
	int nbCreatedNodes(0);
	
	double praramAlongDirection( step() );
	
	shared_ptr<Configuration> fromConfig = expansionNode.getConfiguration();
	shared_ptr<Configuration> toConfig;
	
	double extensionCost(0.);
	double prevCost(0.);
	
	bool firstIteration(true);
	bool upHill(false);
	
	unsigned int nb_of_extend = 0;
	
	LocalPath directionPath( fromConfig , directionConfig );
	
	// Perform extension toward directionConfig
	// Additional nodes creation in the nExtend case, but without 
	// checking for expansion control
	while (!failed && ( praramAlongDirection < directionPath.getParamMax() ) )
	{
		// Takes one configuration along the path
		// Tests if the new small portion is valid
		toConfig = directionPath.configAtParam( praramAlongDirection );
		LocalPath extensionLP( fromConfig , toConfig );
		failed = (!extensionLP.isValid()); nb_of_extend++;
		
		// Transition test for cost spaces, increase temperature in case of failure
		// Only for first iteration
		if (firstIteration && !failed)
		{
			// Expansion Control
			if ( ENV.getBool(Env::expandControl) && !this->expandControl(directionPath, expansionNode) )
			{
				failed = true;
			}
			
			// Compute the cost of the extremal configuration
			fromConfig->setCostAsNotTested();
			toConfig->setCostAsNotTested();
			
			prevCost						= fromConfig->cost();
			extensionCost				= toConfig->cost();
			
			// Cost Test Control
			if ((!failed) && ( prevCost < extensionCost ))
			{
				upHill = true;
				
				if (!costTestSucceeded(&expansionNode, toConfig, extensionCost))
				{
					adjustTemperature(false, &expansionNode);
					failed = true;
				}
				else
				{
					adjustTemperature(true, &expansionNode);
				}
			}
		}
		
		if( upHill && (nb_of_extend != 1) )
		{
			failed = true;
		}
		
		// Check if it's going down hill for next iterations
		if ( failed )
		{
			if ( nb_of_extend == 1 )
			{
				this->expansionFailed( expansionNode );
			}
			break;
		}
		else 
		{
			if ( nb_of_extend == 1 )
			{
				nbCreatedNodes++;
				
				if ( upHill )
				{
					failed = true;
				}
			}
			if ( !failed ) 
			{
				praramAlongDirection += extensionLP.getParamMax();
				fromConfig = toConfig;
			}
		}
	}
	
	// if at least one extention has succeded
	double posAlongDirection =  positionAlongPath ( directionPath , praramAlongDirection );
	
  //	cout << "nbCreatedNodes = " << nbCreatedNodes << endl;
  //	cout << "posAlongDirection = " << posAlongDirection << endl;
	
	if (nbCreatedNodes == 1 && (!toGoal || (toGoal && posAlongDirection >= 1.0)))
	{	
		extensionCost = fromConfig->cost();
		
		//		ATTENTION
		//		TODO Ajouter un champs dans composante connexete
		//		expansionNode.getCompcoStruct()->sumLengthEdges += length;
		
		LocalPath extentionPath(expansionNode.getConfiguration(),
														toConfig );
		
		nbCreatedNodes = 0;
		
		if ( toGoal ) 
		{
			this->addNode(&expansionNode,  directionPath,  posAlongDirection, directionNode,  nbCreatedNodes);
		}
		else 
		{
			this->addNode(&expansionNode,  extentionPath,  0, NULL,  nbCreatedNodes);
		}
	}
	
	if (nbCreatedNodes > 1)
	{
		cout	<< "ERRROR nbCreatedNodes in CostConnectMethod, nbCreatedNodes = "
		<< nbCreatedNodes << endl;
	}
	
	if (ENV.getBool(Env::isCostSpace) && (p3d_GetCostMethodChoice()
																				== MAXIMAL_THRESHOLD))
	{
		p3d_updateCostThreshold();
	}
	
	return nbCreatedNodes;
}

void TransitionExpansion::adjustTemperature(bool accepted, Node* node)
{
	if (accepted)
	{
		node->setTemp(node->getTemp() / 2.0);
		node->getConnectedComponent()->getCompcoStruct()->temperature /= 2.0;
	}
	else
	{
		double factor =
		exp(log(2.) / ENV.getDouble(Env::temperatureRate));
		
		node->setTemp(node->getTemp() * factor );
		node->getConnectedComponent()->getCompcoStruct()->temperature *= factor ;
	}
	
	if (node->equalCompco(m_initNode))
	{
		ENV.setDouble(Env::temperatureStart,
									node->getConnectedComponent()->getCompcoStruct()->temperature);
	}
	else
	{
		ENV.setDouble(Env::temperatureGoal,
									node->getConnectedComponent()->getCompcoStruct()->temperature);
	}
}

bool TransitionExpansion::transitionTest(Node& fromNode,
                                         LocalPath& extensionLocalpath)
{
	// Transition test for cost spaces, increase temperature in case of failure
	double extensionCost = extensionLocalpath.getEnd()->cost();
	
	if ( costTestSucceeded(&fromNode, extensionLocalpath.getEnd(), extensionCost) )
	{
		return true;
	}
	else
	{
		//		cout << "Failed : Cost invalid" << endl;
		int nbCostFail = ENV.getInt(Env::nbCostTransFailed);
		nbCostFail++;
		ENV.setInt(Env::nbCostTransFailed, nbCostFail);
		
		if (ENV.getBool(Env::printCostFail))
			cout << "nbCostFail = " << nbCostFail << endl;
		
		return false;
	}
}

//! extendExpandProcess
//! @param expansion node is the start node allready in the graph
//! @param directionConfig is the direction of expansion
//! @return the number of created nodes
int TransitionExpansion::extendExpandProcess(Node* expansionNode,
																						 shared_ptr<Configuration> directionConfig,
																						 Node* directionNode)
{
  //cout << "Expansion node cost = " <<  expansionNode->cost() << endl;
	//cout << "Expansion node cost = " <<  expansionNode->getNodeStruct()->cost << endl;
	
  bool valid(true);
	bool failed(false);
	int nbCreatedNodes(0);
	
	double extensionCost;
	
	Node* fromNode = expansionNode;
	Node* extensionNode = NULL;
	
	// Perform extension toward directionConfig
	// Additional nodes creation in the nExtend case, but without checking for expansion control
	LocalPath directionLocalpath(fromNode->getConfiguration(), directionConfig);
	
	if ( ENV.getBool(Env::tRrtComputeGradient) )
	{
		if ( directionLocalpath.configAtParam(step())->cost() > fromNode->getConfiguration()->cost() ) 
		{
			
			if( !mixWithGradient(expansionNode,directionConfig) )
			{
				return false;
			}
		}
	}
	
	double pathDelta = directionLocalpath.getParamMax() == 0. ? 1.
	: MIN(1., step() / directionLocalpath.getParamMax() );
	
	LocalPath extensionLocalpath(directionLocalpath.getBegin(), pathDelta == 1.
															 && directionNode ? directionNode->getConfiguration()
															 : directionLocalpath.configAtParam(pathDelta
																																	* directionLocalpath.getParamMax()));
	
	// Expansion control
	// Discards potential nodes that are to close to the graph
	if (ENV.getBool(Env::expandControl) && !expandControl(directionLocalpath,*expansionNode))
	{
		//		cout << "Failed expandControl test in " << __func__ << endl;
		return 0;
	}
  
	extensionCost = extensionLocalpath.getEnd()->cost();
	
	// Transition test and collision check
	//
	if (ENV.getBool(Env::costBeforeColl))
	{
		if (ENV.getBool(Env::isCostSpace))
		{
			if (!transitionTest(*fromNode, extensionLocalpath))
			{
				failed = true;
				//cout << "Failed transition test in " << __func__ << endl;
			}
		}
		if (!failed)
		{
			if (!extensionLocalpath.isValid())
			{
        valid = false;
				failed = true;
			}
		}
	}
	else
	{
		if (!extensionLocalpath.isValid())
		{
      valid = false;
			failed = true;
		}
		if (!failed)
		{
			if (ENV.getBool(Env::isCostSpace))
			{
				if (!transitionTest(*fromNode, extensionLocalpath))
				{
					failed = true;
				}
			}
		}
	}
  
	// Add node to graph if everything succeeded
	if (!failed)
	{
		extensionNode = addNode(fromNode, extensionLocalpath, 
														pathDelta, directionNode, 
														nbCreatedNodes);
		
		if ( ( directionNode != NULL )&&( extensionNode == directionNode ))
		{
			// Components were merged
			cout << "Connected in Transition" << __func__ << endl;
			return 0;
		}
    
		if ( extensionCost > expansionNode->getConfiguration()->cost())
		{
      //cout << "extensionCost = " << extensionCost << " , "
			adjustTemperature(true, extensionNode);
		}
	}
	else
	{
    if(valid)
    {
      adjustTemperature(false, fromNode);
    }
		this->expansionFailed(*expansionNode);
	}
	
	//	if (ENV.getBool(Env::isCostSpace) && ENV.getInt(Env::costMethodChoice)
	//			== MAXIMAL_THRESHOLD)
	//	{
	//		p3d_updateCostThreshold();
	//	}
	
	return nbCreatedNodes;
}

/** 
 * expandProcess
 * This method takes a node and expands it toward a direction configuration
 * it differs wether the method used is (Connect or Extend)
 */
int TransitionExpansion::expandProcess(Node* expansionNode,
																			 shared_ptr<Configuration> directionConfig,
																			 Node* directionNode,
																			 Env::expansionMethod method)
{
	switch (method) 
	{
		case Env::Connect:
		case Env::costConnect:
			return expandCostConnect( *expansionNode, directionConfig, directionNode, false );
			
			
		case Env::Extend:
			return extendExpandProcess( expansionNode, directionConfig, directionNode );
			
		default:
			cerr << "Error : expand process not omplemented" << endl;
			return 0;
	} 
}

TransitionRRT::TransitionRRT(Robot* R, Graph* G) :
RRT(R,G)
{
	cout << "TransitionRRT::TransitionRRT" << endl;
}

TransitionRRT::~TransitionRRT()
{
	
}

int TransitionRRT::init()
{
	int added = TreePlanner::init();
	
	_expan = new TransitionExpansion(this->getActivGraph());
	_expan->setDirectionMethod(NAVIGATION_BEFORE_MANIPULATION);
	static_cast<TransitionExpansion*>(_expan)->setInitAndGoalNodes( _Start, _Goal );
	
	//    p3d_InitSpaceCostParam(this->getActivGraph()->getGraphStruct(),
	//                           this->getStart()->getNodeStruct(),
	//                           this->getGoal()->getNodeStruct());
	
	this->getInit()->getNodeStruct()->temp = ENV.getDouble(Env::initialTemperature);
	this->getInit()->getConnectedComponent()->getCompcoStruct()->temperature = ENV.getDouble(Env::initialTemperature);
	this->getInit()->getNodeStruct()->nbFailedTemp = 0;
	
	p3d_SetGlobalNumberOfFail(0);
	
	//  GlobalNbDown = 0;
	//  Ns->NbDown = 0;
	
	setNodeCost(this->getInit());
  p3d_SetCostThreshold( this->getInit()->cost() );
  p3d_SetInitCostThreshold( p3d_GetNodeCost(this->getInit()->getNodeStruct()) );
	
	if ( ENV.getBool(Env::expandToGoal) && (this->getGoal() != NULL))
	{
		this->getGoal()->getNodeStruct()->temp	= ENV.getDouble(Env::initialTemperature);
		this->getInit()->getNodeStruct()->temp = ENV.getDouble(Env::initialTemperature);
		this->getGoal()->getConnectedComponent()->getCompcoStruct()->temperature = ENV.getDouble(Env::initialTemperature);
		this->getGoal()->getNodeStruct()->nbFailedTemp = 0;
		//    Ng->NbDown = 0;
		
		setNodeCost(this->getGoal());
		p3d_SetCostThreshold(MAX(p3d_GetNodeCost(this->getInit()->getNodeStruct()), 
														 p3d_GetNodeCost(this->getGoal()->getNodeStruct()) ));
		p3d_SetAverQsQgCost((  this->getActivGraph()->getGraphStruct()->search_start->cost
												 + this->getActivGraph()->getGraphStruct()->search_goal->cost) / 2.);
	}
	else
	{
		p3d_SetCostThreshold(this->getInit()->cost() );
		p3d_SetInitCostThreshold( this->getInit()->cost() );
		p3d_SetAverQsQgCost( this->getActivGraph()->getGraphStruct()->search_start->cost);
	}
	
	return added;
}


void TransitionRRT::setNodeCost(Node* node)
{
	global_costSpace->setNodeCost(node,
																node->parent() );
}

/**
 * costConnectNodeToComp
 * Try to connect a node to a given component
 * taking into account the fact that the space
 * is a cost space
 * @return: TRUE if the node and the componant have
 * been connected.
 */
bool TransitionRRT::connectNodeToCompco(Node* node, Node* compNode)
{
  double minumFinalCostGap = ENV.getDouble(Env::minimalFinalExpansionGap);
  
//	int SavedIsMaxDis = FALSE;
	int nbCreatedNodes=0;
  
	LocalPath path( node->getConfiguration(), compNode->getConfiguration() );
	
	if(!ENV.getBool(Env::costBeforeColl))
	{
		if( path.isValid() )
		{
			if( path.getParamMax() <= _expan->step() )
			{
				int nbCreatedNodes=0;
				
				_expan->addNode(node,path,1.0,compNode,nbCreatedNodes);
				cout << "Path Valid Connected" << endl;
				return true;
			}
			
			if( ENV.getBool(Env::costExpandToGoal) &&
				 (path.getParamMax() <= (minumFinalCostGap*_expan->step())) &&
				 _expan->expandToGoal( node, compNode->getConfiguration() ))
			{
				int nbCreatedNodes=0;
				
				_expan->addNode(node,path,1.0,compNode,nbCreatedNodes);
				cout << "attempting connect " << node->getConfiguration()->cost() << 
        " to " << compNode->getConfiguration()->cost() << endl;
				return true;
			}
		}
		return false;
	}
	else
	{
		if( path.getParamMax() <= _expan->step() )
		{
//			cout << "path.getParamMax() <= _expan->step()" << endl;
//			cout << "path.getParamMax() = " << path.getParamMax() << endl;
//			cout << "_expan->step()     = " << _expan->step() << endl;
			
			if ( path.getParamMax() == 0.0 ) {
				node->print();
				compNode->print();
			}
			
			if( path.isValid() )
			{
				_expan->addNode(node,path,1.0,compNode,nbCreatedNodes);
				cout << "Path Valid Connected" << endl;
				return true;
			}
			else
			{
				return false;
			}
		}
		
		// Warning 
		// The expansion to goal happens if only if the gap is inferior to some
		// multiple of the expansion step
		if( ENV.getBool(Env::costExpandToGoal) &&
			 (path.getParamMax() <= (minumFinalCostGap*_expan->step())) &&
			 _expan->expandToGoal(node,compNode->getConfiguration() ))
		{
			if( path.isValid() )
			{
				_expan->addNode(node,path,1.0,compNode,nbCreatedNodes);
				cout << "attempting connect " << node->getConfiguration()->cost() << " to " << compNode->getConfiguration()->cost() << endl;
				return true;
			}
			else
			{
				return false;
			}
		}
		return(false);
	}
}


