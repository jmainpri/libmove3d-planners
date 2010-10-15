#include "ManhattanLike-RRT.hpp"

#include "planningAPI.hpp"

#include "P3d-pkg.h"
#include "Bio-pkg.h"
#include "Collision-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

#define ML_DEBUG 0

ManhattanLikeRRT::ManhattanLikeRRT(Robot* R, Graph* G) :
  RRT(R,G)
{
  std::cout << "ManhattanLikeRRT Constructor" << std::endl;
}

int ManhattanLikeRRT::selectNewJntInList(vector<p3d_jnt*>& joints,
					 vector<p3d_jnt*>& oldJoints, vector<p3d_jnt*>& newJoints)
{
  for(uint i(0); i < joints.size(); i++)
  {
    bool found(false);
    for(uint j(0); j < oldJoints.size(); j++)
    {
      if(oldJoints[j] == joints[i])
      {
	found = true;
	break;
      }
    }
    if(!found)
    {
      newJoints.push_back(joints[i]);
      oldJoints.push_back(joints[i]);
    }
  }
  return(newJoints.size() > 0);
}

int ManhattanLikeRRT::getCollidingPassiveJntList(Robot* R, Configuration& qinv,
						 vector<p3d_jnt*>& joints)
{
  p3d_poly* polys[2];

  // BIO
  if(p3d_col_get_mode() == p3d_col_mode_bio)
  {
#ifdef BIO
    p3d_jnt** passiveJoints = NULL;
    int nJoints = 0;
    bio_get_list_of_passive_joints_involved_in_collision(
      R->getRobotStruct(), qinv.getConfigStruct(), &nJoints, &passiveJoints);
    for(int i(0); i < nJoints; i++)
    {
      joints.push_back(passiveJoints[i]);
    }
    MY_FREE(passiveJoints, p3d_jnt*, nJoints);
#endif
  }
  // not BIO
  else
  {
    R->setAndUpdateWithoutConstraints(qinv);
    if(p3d_col_test() <= 0)
    {
      cout << "No collision detected" << endl;
      return(false);
    }

    // NOTE: KCD only retuns the first collision pair !!!
    // NOTE: ONLY THE PASSIVE JOINT INVOLVED IN THE COLLISION IS RETURNED
    //       BUT ALL THE PARENT JOINTS SHOULD BE ALSO CONSIDERED ???
    p3d_col_get_report(0,&polys[0],&polys[1]);
    for(uint i(0); i < 2; i++)
      if(polys[i]->p3d_objPt->jnt != NULL)
	if(!p3d_jnt_get_is_active_for_planner(polys[i]->p3d_objPt->jnt))
	  joints.push_back(polys[i]->p3d_objPt->jnt);
  }
  return(joints.size() > 0);
}

void ManhattanLikeRRT::shoot_jnt_list_and_copy_into_conf(Configuration& qrand,
							 vector<p3d_jnt*>& joints)
{
  //double perturb = 0.1; // NOTE: THIS SHOULD BE A PARAMETER
  double perturb = 0.3; // NOTE: THIS SHOULD BE A PARAMETER

  // NOTE : the random shoot should be centered at q_inv !!!
  //        (doesn't matter in the case of "circular" joints)

  for(uint i(0); i < joints.size(); i++)
  {
    p3d_jnt* joint(joints[i]);
    for(int j(0); j < joint->dof_equiv_nbr; j++)
    {
      double vmin, vmax;
      double val, rval;
      p3d_jnt_get_dof_rand_bounds(joint, j, &vmin, &vmax);
      int k(joint->index_dof + j);
      if(!p3d_jnt_is_dof_circular(joint, j))
	val = p3d_random(vmin,vmax);
      else
      {
	double midrange = (vmax-vmin) / 2.0;
	// perturbation factor
	midrange *= perturb;
	rval = p3d_random(-midrange,midrange);
	val = qrand.getConfigStruct()[k] + rval;
	val = MAX(MIN(val, vmax), vmin);
      }
      qrand.getConfigStruct()[k] = val;
    }
  }
}

bool ManhattanLikeRRT::getCurrentInvalidConf(Configuration& q) {
#ifdef BIO
  return(p3d_col_get_mode() == p3d_col_mode_bio ? 
	 bio_get_current_q_inv(_Robot->getRobotStruct(), q.getConfigStruct()) :
	 p3d_get_current_q_inv(_Robot->getRobotStruct(), q.getConfigStruct()));
#endif
	cout << "Warning : BIO Not compiled in " << __func__ <<  endl;
	return false;
}

int ManhattanLikeRRT::passiveExpandProcess(Node* expansionNode, 
					   int NbActiveNodesCreated,
					   Node* directionNode)
{
  int totalCreatedNodes(0);
  bool expansionSucceeded(true);

  // Don't perfom passive expansion if this parameter is false
  // and the active expansion didn't create any node.
  if (!ENV.getBool(Env::isPasExtWhenAct) && 
      NbActiveNodesCreated == 0)
  {
    return 0;
  }
  
  Configuration invalConf(_Robot);
  vector<p3d_jnt*> oldJoints;
  // Keep iterating while a collision is encountered during the connect expansion,
  // unless the last passive expansion failed.
  while (expansionSucceeded && this->getCurrentInvalidConf(invalConf))
  {
    vector<p3d_jnt*> joints;
    vector<p3d_jnt*> newJoints;
    expansionSucceeded = false;
    
    // Get the colliding passive dofs and select those that have not been expanded yet.
    if (getCollidingPassiveJntList(_Robot, invalConf, joints) &&
      selectNewJntInList(joints, oldJoints, newJoints))
    {
      // Create a copy of the expansion configuration.
      // This configuration will become the new target configuration,
      // after its passive dofs have been changed.
      shared_ptr<Configuration> newRandConf = expansionNode->getConfiguration()->copy();
      
      for (int i = 0;
	   i < ENV.getInt(Env::MaxPassiveExpand) && !expansionSucceeded;
	   i++)
      {
	shoot_jnt_list_and_copy_into_conf(*newRandConf, newJoints);
	int nbCreatedNodes = _expan->expandProcess(expansionNode,
						   newRandConf, directionNode,
						   Env::Connect);
	expansionSucceeded = nbCreatedNodes > 0;
	if (expansionSucceeded)
	{
	  totalCreatedNodes += nbCreatedNodes;
	  // If the expansion succeeded, update the expansion node for the next iteration
	  expansionNode = _Graph->getLastNode();
	  if (ML_DEBUG) { cout << "Expanded passive parameters at try " << i + 1 << endl; }
	}
      }
    }
  }
  return (totalCreatedNodes);
}

bool ManhattanLikeRRT::manhattanSamplePassive()
{
  return (ENV.getDouble(Env::manhatRatio) < p3d_random(0., 1.));
}

int ManhattanLikeRRT::expandOneStep(Node* fromComp,Node* toComp)
{
  Node* directionNode(NULL);
  Node* expansionNode(NULL);
  shared_ptr<Configuration> directionConfig;

  // get direction
  directionConfig = _expan->getExpansionDirection(fromComp,toComp,
						  false,directionNode);
  // get node for expansion toward direction
  expansionNode = _expan->getExpansionNode(fromComp,directionConfig,
					   ACTIVE_CONFIG_DIST);
  // copy passive dofs
  expansionNode->getConfiguration()->copyPassive(*directionConfig);
  // expand the active dofs
  int nbCreatedNodes = _expan->expandProcess(expansionNode,
					     directionConfig, directionNode, ENV.getExpansionMethod());

  // expand the passive dofs
  nbCreatedNodes += this->passiveExpandProcess(nbCreatedNodes == 0 ? 
					       expansionNode :
					       _Graph->getLastNode(),
					       nbCreatedNodes,
					       directionNode);
  return (nbCreatedNodes);
}
