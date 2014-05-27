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
 *
 * State.h
 *
 * This is the header file of a pure virtual base class for the
 * state class that will be written by the
 * programmer for his/her own purpose
 * */

#ifndef STATE_H
#define STATE_H

#include <vector>

namespace Move3D
{
/**
   * @ingroup C++ Planning API
   */
class State
{
public:
    State();
    double computeCost(State *parent, State* goal); /* f (f=g+h) */
    
    double f() const;				/* get functions for the variables */
    double g() const;
    double h() const;
    
    // Test that the state is valid
    virtual bool isValid() { return true; }
    
    /* states after branching is returned and the number of
     non-NULL states in the returned array is saved in the variable nodes_n */
    virtual std::vector<State*> getSuccessors(State* s);
    virtual bool isLeaf();		/* leaf control for an admissible heuristic function; the test of h==0*/
    virtual bool equal(State* other);
    
    virtual void setClosed(std::vector<State*>& closedStates,std::vector<State*>& openStates);
    virtual bool isColsed(std::vector<State*>& closedStates);
    
    virtual void setOpen(std::vector<State*>& openStates);
    virtual bool isOpen(std::vector<State*>& openStates);
    
    virtual void reset() {}
    
    virtual void print() {}
    
protected:
    virtual double computeLength(State *parent);       /* g */
    virtual double computeHeuristic(State *parent,State* goal);    /* h */
    
private:
    /* f, g, h values */
    double _f,_g,_h;
};
}

#endif

