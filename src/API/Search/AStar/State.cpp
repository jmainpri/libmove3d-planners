/*
 *
 * State_Base.cpp
 *
 * This is a virtual base class for the 
 * state class that will be written by the
 * programmer for his/her own purpose
 *
 * The derivation from this class is required
 * and the name of the new class should be 
 * "State" with the necessary functions defined.
 *
 * */

#include <iostream>
#include <limits>
#include "State.hpp"

using namespace std;
using namespace API;

State::State() : _f(0), _g(0), _h(0) {}


double State::computeCost(State *parent, State* goal)
{ 
    _g = computeLength(parent);
    _h = computeHeuristic(parent,goal);

     return _f = ( _g + _h );
}


bool State::isLeaf()
{
        return (0==_h);
}

bool State::equal(State* other)
{
    cout << "equal(State* other) not implemented" << endl;
    return false;
}

double State::f() const {return _f;}
double State::g() const {return _g;}
double State::h() const {return _h;}

vector<State*> State::getSuccessors()
{
    vector<State*> successors;
    cout << "State::getSuccessors() Not implemented" << endl;
    return successors;
}

double State::computeLength(State *parent)
{
    return 0;
}

double State::computeHeuristic(State *parent = NULL ,State* goal = NULL)
{
    return 0;
}

bool State::isColsed(vector<State*>& closedStates)
{
    bool isClosed=false;
    for(unsigned i=0;i<closedStates.size();i++)
    {
        if(this->equal(closedStates[i]))
        {
           isClosed = true;
//           this->print();
           break;
        }
    }
//    cout << "State = " << this << " is " << isClosed << endl;
    return isClosed;
}

void State::setClosed(std::vector<State*>& closedStates,std::vector<State*>& openStates)
{
    for (vector<State*>::iterator it = openStates.begin(); it!=openStates.end(); ++it)
    {
        if((*it)->equal(this))
        {
            openStates.erase(it);
            break;
        }
    }

    closedStates.push_back(this);
}

bool State::isOpen(vector<State*>& openStates)
{
    bool isOpen=false;
    for(unsigned i=0;i<openStates.size();i++)
    {
        if(this->equal(openStates[i]))
        {
           isOpen = true;
//           this->print();
           break;
        }
    }
//    cout << "State = " << this << " is " << isClosed << endl;
    return isOpen;
}


void State::setOpen(std::vector<State*>& openStates)
{
    openStates.push_back(this);
}



