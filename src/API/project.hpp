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
#ifndef WORKSPACE_HPP
#define WORKSPACE_HPP

#include "API/scene.hpp"

/**
  * @ingroup NEW_CPP_MODULE
  * @defgroup CPP_API C++ Planning API
  * @brief Implements in C++ an interface to the more low level functionalities
  */

/**
 * @ingroup CPP_API
 * \brief Classe représentant l'espace de travail de l'application
 * @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */
namespace Move3D
{

class Project {

public:
	/**
	 * Class constructor
	 */
	Project(Scene* sc);

	/**
	 * Class Destructor
	 */
	~Project();

	/**
	 * obtient l'Scene actif
	 * @return l'Scene actif
	 */
	Scene* getActiveScene();
	
	/**
	  * modifie l'Scene actif
	  * @param name le nom du nouvel Scene actif
	  */
	void setActiveScene(std::string name);

	/**
	 * insert un nouvel Scene au vecteur des Scene
	 * @param E le nouvel Scene
	 */
	void insertScene(Scene* E);

private:
        std::vector<Scene*> m_Scenes;/*!< le vecteur des Scene chargés*/
        std::string m_activeScene;/*!< le nom de l'Scene actif*/
};

extern Project* global_Project;

}

#endif
