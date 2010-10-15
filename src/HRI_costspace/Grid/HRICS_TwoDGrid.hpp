#ifndef HRICS_TWODGRID_HPP
#define HRICS_TWODGRID_HPP

#include "planningAPI.hpp"
#include "API/Grids/gridsAPI.hpp"

namespace HRICS
{	
	/**
	 @ingroup HRICS
	 @brief Plannar HRI Grid
	 */
	class PlanGrid : public API::TwoDGrid
	{
	public:
		PlanGrid();
		PlanGrid(double pace, std::vector<double> envSize);
		
		void setRobot(Robot* R) { mRobot = R; }
		Robot* getRobot() { return mRobot; }
		
		void writeToOBPlane();
		
		
		API::TwoDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y );
		
		void draw();
		
		void setRobotToStoredConfig();
		
	private:
		Robot* mRobot;
	};
	
	/**
	 @ingroup HRICS
	 @brief Plannar HRI Cell
	 */
	class PlanCell : public API::TwoDCell
	{
		
	public:
		PlanCell();
		PlanCell(int i, Eigen::Vector2i coord, Eigen::Vector2d corner, PlanGrid* grid);
		
		~PlanCell() { }
		
		double getCost(); /* { std::cout << " Warning not implemented"  << std::endl; }*/
		
		void setBlankCost() { mCostIsComputed = false; }
		
		Eigen::Vector2i getCoord() { return _Coord; }
		
		bool getOpen() { return _Open; }
		void setOpen() { _Open = true; }
		
		bool getClosed() { return _Closed; }
		void setClosed() { _Closed = true; }
		
		void resetExplorationStatus() { _Open = false; _Closed = false; }
		//        void draw();
		
	private:
		
		Eigen::Vector2i _Coord;
		//        double* _v0; double* _v1; double* _v2; double* _v3;
		//        double* _v4; double* _v5; double* _v6; double* _v7;
		
		bool _Open;
		bool _Closed;
		
		bool mCostIsComputed;
		double mCost;
		
	};
	
	/**
	 @ingroup HRICS
	 @brief Plannar HRI State
	 */
	class PlanState : public API::State
	{
	public:
		PlanState() {}
		PlanState( Eigen::Vector2i cell, PlanGrid* grid);
		PlanState( PlanCell* cell , PlanGrid* grid);
		
		std::vector<API::State*> getSuccessors();
		
		bool isLeaf();		/* leaf control for an admissible heuristic function; the test of h==0*/
		bool equal(API::State* other);
		
		void setClosed(std::vector<PlanState*>& closedStates,std::vector<PlanState*>& openStates);
		bool isColsed(std::vector<PlanState*>& closedStates);
		
		void setOpen(std::vector<PlanState*>& openStates);
		bool isOpen(std::vector<PlanState*>& openStates);
		
		void reset();
		
		void print();
		
		PlanCell* getCell() { return _Cell; }
		
	protected:
		double computeLength(API::State *parent);       /* g */
		double computeHeuristic(API::State *parent = NULL ,API::State* goal = NULL);    /* h */
		
	private:
		PlanGrid* _Grid;
		PlanCell* _Cell;
	};
}

#endif // HRICS_TWODGRID_HPP
