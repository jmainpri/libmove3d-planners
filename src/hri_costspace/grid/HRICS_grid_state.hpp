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
