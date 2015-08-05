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
#include "HRICS_grid_state.hpp"
#include "HRICS_grid.hpp"

using namespace std;
using namespace HRICS;

// import most common Eigen types 
//using namespace Eigen;
using namespace Eigen;

State::State( Vector3i cell , Grid* grid) :
        _Grid(grid)
{
    _Cell = dynamic_cast<Cell*>(grid->getCell(cell));
}

State::State( Cell* cell , Grid* grid) :
        _Cell(cell),
        _Grid(grid)
{

}


vector<Move3D::State*> State::getSuccessors()
{
    vector<Move3D::State*> newStates;
//    newStates.reserve(26);

    for(int i=0;i<26;i++)
    {
        Cell* neigh = dynamic_cast<Cell*>(_Grid->getNeighbour( _Cell->getCoord(), i));
        if( neigh != NULL )
        {
            _Grid->isVirtualObjectPathValid(dynamic_cast<Cell*>(_Cell),neigh);
            newStates.push_back( new State(neigh,_Grid));
        }
    }

    return newStates;
}

bool State::isLeaf()
{
    return false;
}

bool State::equal(Move3D::State* other)
{
    //bool equal(false);
    State* state = dynamic_cast<State*>(other);
    Vector3i pos = _Cell->getCoord();
    for(int i=0;i<3;i++)
    {
        if( pos[i] != state->_Cell->getCoord()[i])
        {
            //            cout << "State::equal false" << endl;
            return false;
        }
    }

    //    cout << "State::equal true" << endl;
    return true;
}

void State::setClosed(std::vector<State*>& closedStates,std::vector<State*>& openStates)
{
    //    cout << "State :: set Closed" <<endl;
    _Cell->setClosed();
}

bool State::isColsed(std::vector<State*>& closedStates)
{
    //    cout << "State :: get Closed" <<endl;
    return _Cell->getClosed();
}

void State::setOpen(std::vector<State*>& openStates)
{
    //     cout << "State :: set open" <<endl;
    _Cell->setOpen();
}


bool State::isOpen(std::vector<State*>& openStates)
{
    //    cout << "State :: get open" <<endl;
    return _Cell->getOpen();
}

void State::reset()
{
    _Cell->resetExplorationStatus();
}

void State::print()
{

}

double State::computeLength(Move3D::State *parent)
{
    State* preced = dynamic_cast<State*>(parent);

    Vector3d pos1 = _Cell->getCenter();
    Vector3d pos2 = preced->_Cell->getCenter();

    double dist = ( pos1 - pos2 ).norm();

//    double cost1 = preced->_Cell->getCost();
    double cost2 = _Cell->getCost();
    double g = preced->g() + /*cost1 +*/ cost2 + dist;

//    cout << "dist = " << dist << endl;
//    cout << "g = " << g << endl;
    return g;
}

double State::computeHeuristic(Move3D::State *parent,Move3D::State* goal)
{
    State* state = dynamic_cast<State*>(goal);

    Vector3d pos1 = state->_Cell->getCenter();
    Vector3d pos2 = _Cell->getCenter();

    double dist=0;

    for(int i=0;i<3;i++)
    {
        dist += (pos1[i]-pos2[i])*(pos1[i]-pos2[i]);;
    }
    return dist;
}
