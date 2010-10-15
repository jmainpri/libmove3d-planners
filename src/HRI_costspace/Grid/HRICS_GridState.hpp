#ifndef HRIGRIDSTATE_HPP
#define HRIGRIDSTATE_HPP

#include "API/planningAPI.hpp"
#include "HRI_costspace/Grid/HRICS_Grid.hpp"
#include "HRI_costspace/Grid/HRICS_Cell.hpp"

/**
 @ingroup HRICS
 @brief Cell for the HRICS AStar
 */
namespace HRICS
{
	class State : public API::State
	{
	public:
		State() {}
		State( Eigen::Vector3i cell, Grid* grid);
		State( Cell* cell , Grid* grid);
		
		std::vector<API::State*> getSuccessors();
		
		bool isLeaf();		/* leaf control for an admissible heuristic function; the test of h==0*/
		bool equal(API::State* other);
		
		void setClosed(std::vector<State*>& closedStates,std::vector<State*>& openStates);
		bool isColsed(std::vector<State*>& closedStates);
		
		void setOpen(std::vector<State*>& openStates);
		bool isOpen(std::vector<State*>& openStates);
		
		void reset();
		
		void print();
		
		Cell* getCell() { return _Cell; }
		
	protected:
		double computeLength(API::State *parent);       /* g */
		double computeHeuristic(API::State *parent = NULL ,API::State* goal = NULL);    /* h */
		
	private:
		Cell* _Cell;
		Grid* _Grid;
	};
}

#endif // HRIGRIDSTATE_HPP
