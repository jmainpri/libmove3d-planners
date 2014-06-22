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
#ifndef LOCALPATH_VALID_TEST_HPP
#define LOCALPATH_VALID_TEST_HPP

#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/localpath.hpp"

namespace Move3D {

/**
 * @ingroup CONFIG_SPACE
 * \brief Tests the localPath using a colision checker
 */
class LocalPathValidTest : public LocalPath {

public:
	/**
	 * Costructors
	 */
//	LocalPathValidTest();
	LocalPathValidTest(const LocalPath& LP);
	
	/**
	 * Destructor
	 */
	~LocalPathValidTest();
	
	/**
	 * Tests the
	 */
	bool testIsValid();
	
	/**
	 *
	 */
	void setClassicTest(bool isClassic);
	
	/**
	 *
	 */
	unsigned int getNbCollisionTest();
	
	/**
	 *
	 */
    confPtr_t getLastValidConfiguration();
	
private:
	
	bool testClassic();
	bool testDichotomic();
	
	bool changePositionRobot(double l) ;
	bool changePositionRobotWithoutCntrt(double l);
	bool invalidJointLimits();
	
	/**
	 * Variables
	 */
	bool mIsClassicVsDicho;
	
	bool mDoSelf;
	bool mDoStatics;
	bool mDoOthers;
	
	bool mMicrocollisionAvoidance;
	
	MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> mLastValidConfiguration;
	
	unsigned int mNbTest;

};

}

#endif
 
