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
		EnvGrid(double pace, std::vector<double> envSize, bool isHumanCentered, Robot* robot, Robot* human);

		void init(std::pair<double,double> minMax);

		void initGrid();

		void computeHumanRobotReacheability(std::pair<double,double> minMax);

		bool isHumanCentered(){ return _isHumanCentered; }
		
		void setRobot(Robot* R) { mRobot = R; }
		Robot* getRobot() { return mRobot; }

		void setHuman(Robot* R) { mHuman = R; }
		Robot* getHuman() { return mHuman; }

		Robot* getHumanCylinder() {return humCyl;}
		Robot* getRobotCylinder() {return robotCyl;}
		
		API::TwoDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y );

		void computeDistances(EnvCell* cell, bool isHuman);
		
		void draw();

		void setCellsToblankCost();

		std::vector<EnvCell*> getSortedCells();

		void recomputeCostRobotOnly();

		double getNbCellX() {return _nbCellsX;}
		double getNbCellY() {return _nbCellsY;}

		void initAllCellState();
		void initAllReachability();

		double getHumanMaxDist() {return m_humanMaxDist;}
		double getRobotMaxDist() { return m_robotMaxDist;}

		std::vector<std::pair<double,EnvCell*> > getSortedGrid();
		void setAsNotSorted() {gridIsSorted = false;}
		
	private:
		Robot* mRobot;
		Robot* mHuman;

		Robot* humCyl;
		Robot* robotCyl;

		// this is for differenciation between grids.
		bool _isHumanCentered;

		double m_humanMaxDist;
		double m_robotMaxDist;

		std::vector<EnvCell*> m_HumanAccessible;
		std::vector<EnvCell*> m_RobotAccessible;

		std::vector<std::pair<double,EnvCell*> > sortedGrid;
		bool gridIsSorted;
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

		void computeReach();

		void setCost(double value) {mCost = value; }

		void setHumanDist(double value) {m_humanDist = value; }
		double getHumanDist() {return m_humanDist; }
		void setHumanDistIsComputed() {m_humanDistIsComputed = true;}
		bool isHumanDistComputed() {return m_humanDistIsComputed;}

		void setRobotDist(double value) {m_robotDist = value; }
		double getRobotDist() {return m_robotDist; }
		void setRobotDistIsComputed() {m_robotDistIsComputed = true;}
		bool isRobotDistComputed() {return m_robotDistIsComputed;}

		void setBlankCost() { mCostIsComputed = false; }
		
		Eigen::Vector2i getCoord() { return _Coord; }
		
		bool getOpen() { return _Open; }
		void setOpen() { _Open = true; }
		
		bool getClosed() { return _Closed; }
		void setClosed() { _Closed = true; }
		
		void resetExplorationStatus() { _Open = false; _Closed = false; }
		void resetReacheability();
		//        void draw();
		bool isNotAccessible;

		bool isHumAccessible() {return m_isHumanAccessible;}
		void setHumAccessible(bool value) {m_isHumanAccessible = value;}
		bool isRobAccessible() {return m_isRobotAccessible;}
		void setRobAccessible(bool value) {m_isRobotAccessible = value;}

		std::vector<EnvCell*> getNeighbors(bool isHuman);

		double computeDist(EnvCell* neighCell);

		std::vector<EnvCell*> getHumanTraj() {return humanTraj;}
		void setHumanTraj(std::vector<EnvCell*> _humanTraj) {humanTraj = _humanTraj; }

		std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > getHumanVectorTraj() {return humanVectorTraj;}
		void setHumanVectorTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > _humanVectorTraj) {humanVectorTraj = _humanVectorTraj; }

		std::vector<EnvCell*> getRobotTraj() {return robotTraj;}
		void setRobotTraj(std::vector<EnvCell*> _robotTraj) {robotTraj = _robotTraj; }

		std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > getRobotVectorTraj() {return robotVectorTraj;}
		void setRobotVectorTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > _RobotVectorTraj) {robotVectorTraj = _RobotVectorTraj; }

		void addPoint(double Rz);

		std::vector<double> getRandomVector(){return randomVectorPoint;}

		void setHumanRobotReacheable(std::vector<EnvCell*> value) {initHumanRobotReacheable = value;}
		std::vector<EnvCell*> getHumanRobotReacheable() {return initHumanRobotReacheable;}
		void addToHumanRobotReacheable(EnvCell* cell) {initHumanRobotReacheable.push_back(cell);}
		void clearHumanRobotReacheable() {initHumanRobotReacheable.clear();}

		void setCurrentHumanRobotReacheable(std::vector<EnvCell*> value) {currentHumanRobotReacheable = value;}
		std::vector<EnvCell*> getCurrentHumanRobotReacheable() {return currentHumanRobotReacheable;}
		void addToCurrentHumanRobotReacheable(EnvCell* cell) {currentHumanRobotReacheable.push_back(cell);}
		void clearCurrentHumanRobotReacheable() {currentHumanRobotReacheable.clear();}

		std::pair<double,EnvCell*> getRobotBestPos(){return robotBestPos;}
		void setRobotBestPos(std::pair<double,EnvCell*> value){robotBestPos = value;}

	private:
		
		Eigen::Vector2i _Coord;
//		//        double* _v0; double* _v1; double* _v2; double* _v3;
//		//        double* _v4; double* _v5; double* _v6; double* _v7;
		
		bool _Open;
		bool _Closed;
		
		bool mCostIsComputed;
		double mCost;

		bool m_isHumanAccessible;
		bool m_isRobotAccessible;

		bool m_reachComputed;

		bool m_humanDistIsComputed;
		double m_humanDist;
		std::vector<EnvCell*> humanTraj;
		std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > humanVectorTraj;

		bool m_robotDistIsComputed;
		double m_robotDist;
		std::vector<EnvCell*> robotTraj;
		std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > robotVectorTraj;

		std::vector<double> randomVectorPoint;

		std::vector<EnvCell*> initHumanRobotReacheable;
		std::vector<EnvCell*> currentHumanRobotReacheable;
		std::pair<double,EnvCell*> robotBestPos;

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
