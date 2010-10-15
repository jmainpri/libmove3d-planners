/*
 * ManhattanLike-RRT.hpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#ifndef MANHATTANLIKERRT_HPP_
#define MANHATTANLIKERRT_HPP_

#include "RRT.hpp"

/**
@ingroup Diffusion
*/
class ManhattanLikeRRT : public RRT {

public:
	/** Constructor from a WorkSpace object
	 * @param WS the WorkSpace
	 */
        ManhattanLikeRRT(Robot* R, Graph* G);

	/**
	 * Destructor
	 */
  virtual ~ManhattanLikeRRT() {}


	/**
	 * expansion of one Node from one Component to an other
	 * In the ML case
	 * @param fromComp la composante connexe de départ
	 * @param toComp la composante connexe d'arrivée
	 * @return le nombre de Node créés
	 */
	int expandOneStep(Node* fromComp, Node* toComp);

  bool getCurrentInvalidConf(Configuration& q);

	/**
	 * expansion des joints passifs dans le cas ML_RRT
	 * @param expansionNode
	 * @param NbActiveNodesCreated le nombre de Node créés lors de l'expansion de joints actifs
	 * @param directionNode la direction de l'expansion
	 * @return le nombre de Node Créés
	 */
	int passiveExpandProcess(Node* expansionNode, int NbActiveNodesCreated,
			Node* directionNode);

	/**
	 * choisie si l'expansion sera de type Manhattan
	 * @return l'expansion sera de type Manhattan
	 */
	bool manhattanSamplePassive();

	int selectNewJntInList(std::vector<p3d_jnt*>& joints,
			std::vector<p3d_jnt*>& oldJoints, std::vector<p3d_jnt*>& newJoints);

	int getCollidingPassiveJntList(Robot* R, Configuration& qinv,
			std::vector<p3d_jnt*>& joints);

	void shoot_jnt_list_and_copy_into_conf(Configuration& qrand,
			std::vector<p3d_jnt*>& joints);

};

#endif /* MANHATTANLIKERRT_HPP_ */
