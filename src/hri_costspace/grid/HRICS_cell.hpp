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
#ifndef HRICELL_H
#define HRICELL_H

#include "API/Grids/ThreeDGrid.hpp"

#include "HRICS_grid.hpp"

#include <libmove3d/include/Graphic-pkg.h>

/**
 @ingroup HRICS
 @brief Cell for the HRICS AStar
 */
namespace HRICS
{
	class Cell : public Move3D::ThreeDCell
	{
		
	public:
		Cell();
		Cell(int i, Eigen::Vector3i pos , Eigen::Vector3d corner, HRICS::Grid* grid);
		
		~Cell() { }
		
		double getCost();
		//        double getHRICostSpace();
		void setBlankCost() { _CostIsComputed = false; this->resetExplorationStatus(); }
		
		Eigen::Vector3i getCoord() { return _Coord; }
		
		bool getOpen() { return _Open; }
		void setOpen() { _Open = true; }
		
		bool getClosed() { return _Closed; }
		void setClosed() { _Closed = true; }
		
		void resetExplorationStatus();
		
		GLint getDisplayList() { return m_list; }
		void createDisplaylist();
		
		bool getIsCostComputed() { return _CostIsComputed; }
		
		void setGradient(const Eigen::Vector3d& grad) { m_GradientDirection = grad; } 
		Eigen::Vector3d getGradient() { return m_GradientDirection; }
		
		void draw();
		
	private:
		
		Eigen::Vector3i _Coord;
		
		double* _v0; double* _v1; double* _v2; double* _v3;
		double* _v4; double* _v5; double* _v6; double* _v7;
		
		bool _Open;
		bool _Closed;
		
		bool _CostIsComputed;
		double _Cost;
		
		GLint m_list;
		
		Eigen::Vector3d m_GradientDirection;
		
	};
	//Contenu du namespace
}


#endif // HRICELL_H
