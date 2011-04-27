#ifndef HRICS_EnvGRID_HPP
#define HRICS_EnvGRID_HPP

#include "API/planningAPI.hpp"
#include "API/Grids/gridsAPI.hpp"

namespace HRICS
{	
	class EnvCell;
	/**
	 @ingroup HRICS
	 @brief Plannar HRI Grid
	 */
	class EnvGrid : public API::TwoDGrid
	{
	public:
		EnvGrid();
		EnvGrid(double pace, std::vector<double> envSize, bool isHumanCentered);

		bool isHumanCentered(){ return _isHumanCentered; }
		
		void setRobot(Robot* R) { mRobot = R; }
		Robot* getRobot() { return mRobot; }

		void setHuman(Robot* R) { mHuman = R; }
		Robot* getHuman() { return mHuman; }
		
		API::TwoDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y );
		
		void draw();

		void setCellsToblankCost();

		std::vector<EnvCell*> getSortedCells();

		void recomputeCostRobotOnly();
		
	private:
		Robot* mRobot;
		Robot* mHuman;

		// this is for differenciation between grids.
		bool _isHumanCentered;
	};
	
	/**
	 @ingroup HRICS
	 @brief Plannar HRI Cell
	 */
	class EnvCell : public API::TwoDCell
	{
		
	public:
		EnvCell();
		EnvCell(int i, Eigen::Vector2i coord, Eigen::Vector2d corner, EnvGrid* grid);
		
		~EnvCell() { }
		
		double getCost(); /* { std::cout << " Warning not implemented"  << std::endl; }*/

		void setCost(double value) {mCost = value; }

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
//		//        double* _v0; double* _v1; double* _v2; double* _v3;
//		//        double* _v4; double* _v5; double* _v6; double* _v7;
		
		bool _Open;
		bool _Closed;
		
		bool mCostIsComputed;
		double mCost;
		
	};


	/**
	 @ingroup HRICS
	 @brief Plannar HRI State
	 */
	class EnvState : public API::State
	{
	public:
		EnvState() {}
		EnvState( Eigen::Vector2i cell, EnvGrid* grid);
		EnvState( EnvCell* cell , EnvGrid* grid);

		std::vector<API::State*> getSuccessors();

		bool isLeaf();		/* leaf control for an admissible heuristic function; the test of h==0*/
		bool equal(API::State* other);

		void setClosed(std::vector<EnvState*>& closedStates,std::vector<EnvState*>& openStates);
		bool isColsed(std::vector<EnvState*>& closedStates);

		void setOpen(std::vector<EnvState*>& openStates);
		bool isOpen(std::vector<EnvState*>& openStates);

		void reset();

		void print();

		EnvCell* getCell() { return _Cell; }

	protected:
		double computeLength(API::State *parent);       /* g */
		double computeHeuristic(API::State *parent = NULL ,API::State* goal = NULL);    /* h */

	private:
		EnvGrid* _Grid;
		EnvCell* _Cell;
	};
}



#endif // HRICS_TWODGRID_HPP
