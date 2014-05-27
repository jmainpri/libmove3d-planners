/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
/*
 * testModel.hpp
 *
 *  Created on: Jul 8, 2009
 *      Author: jmainpri
 */

#ifndef TESTMODEL_HPP_
#define TESTMODEL_HPP_

#ifndef ROBOT_HPP
namespace Move3D { class Robot; }
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
    Move3D::Robot* modelRobot;

	int nbColisionTest;
	int nbLocalPathTest;

};




#endif /* TESTMODEL_HPP_ */
