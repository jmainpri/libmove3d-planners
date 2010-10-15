/*
 * testModel.hpp
 *
 *  Created on: Jul 8, 2009
 *      Author: jmainpri
 */

#ifndef TESTMODEL_HPP_
#define TESTMODEL_HPP_

#ifndef ROBOT_HPP
class Robot;
#endif

/**
  * @defgroup USER_APPLI User Applications
  */

/**
  * @ingroup USER_APPLI
  * Does multiple querries for a Robot such as Collision, Localpath validity and cost computations
  * to get the Model behavior regarding thos computation primitives
  */
class TestModel{

public:

	TestModel();

	int nbOfColisionsPerSeconds();
	int nbOfVoxelCCPerSeconds();
	int nbOfCostPerSeconds();
	int nbOfLocalPathsPerSeconds();

	void distEnv();

	void runAllTests();

private:
	Robot* modelRobot;

	int nbColisionTest;
	int nbLocalPathTest;

};




#endif /* TESTMODEL_HPP_ */
