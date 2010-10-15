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

namespace API
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

        /* states after branching is returned and the number of
           non-NULL states in the returned array is saved in the variable nodes_n */
        virtual std::vector<State*> getSuccessors();
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

