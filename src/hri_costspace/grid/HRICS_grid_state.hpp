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
#ifndef HRIGRIDSTATE_HPP
#define HRIGRIDSTATE_HPP

#include "HRICS_grid.hpp"
#include "HRICS_cell.hpp"

#include "API/Search/AStar/AStar.hpp"

/**
 @ingroup HRICS
 @brief Cell for the HRICS AStar
 */
namespace HRICS
{
    class State : public Move3D::State
	{
	public:
		State() {}
		State( Eigen::Vector3i cell, Grid* grid);
		State( Cell* cell , Grid* grid);
		
        std::vector<Move3D::State*> getSuccessors();
		
		bool isLeaf();		/* leaf control for an admissible heuristic function; the test of h==0*/
        bool equal(Move3D::State* other);
		
		void setClosed(std::vector<State*>& closedStates,std::vector<State*>& openStates);
		bool isColsed(std::vector<State*>& closedStates);
		
		void setOpen(std::vector<State*>& openStates);
		bool isOpen(std::vector<State*>& openStates);
		
		void reset();
		
		void print();
		
		Cell* getCell() { return _Cell; }
		
	protected:
        double computeLength(Move3D::State *parent);       /* g */
        double computeHeuristic(Move3D::State *parent = NULL ,Move3D::State* goal = NULL);    /* h */
		
	private:
		Cell* _Cell;
		Grid* _Grid;
	};
}

#endif // HRIGRIDSTATE_HPP
