/*
 *  GridCollisionChecker.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 07/06/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef GRID_COLLISION_CHECKER_H_
#define GRID_COLLISION_CHECKER_H_

#include "planningAPI.hpp"
#include "Grids/gridsAPI.hpp"
#include "CellCollisionChecker.h"
#include "PointsOnBodies.h"

#ifndef _P3D_H
typedef struct obj;
#endif

class GridCollisionChecker : public API::ThreeDGrid
{
public:
    //constructors and destructors
    GridCollisionChecker();
    virtual ~GridCollisionChecker();
	
    //setters and getters
    inline int getNbCellsOverX(void){return _nbCellsX;}
    inline int getNbCellsOverY(void){return _nbCellsY;}
    inline int getNbCellsOverZ(void){return _nbCellsZ;}
	
    //functions
    void init(void);
	
	std::vector<CellCollisionChecker*> getCellListForObject(obj* obj, const Eigen::Transform3d& Trans);
	std::vector<CellCollisionChecker*> getOccupiedCells() { return m_OccupationCells; }
    void updateRobotOccupationCells(Robot* rob);
	
	std::vector<CellCollisionChecker*> computeOccupiedCells(LocalPath& path);
	bool areCellsValid(std::vector<CellCollisionChecker*> cells);
	
    void unvalidObjectCells(obj* obj);
    void draw();
    bool collisionCheck();	
protected:
	API::ThreeDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z );
	
private:
    //The position of the origin of the grid regarding th eorigin of the world
    int m_nbMaxCells; //the number of cell along the longest axis of the environment
	
	std::vector<CellCollisionChecker*> m_OccupationCells;
	Robot* m_Robot;	
	
	BodySurfaceSampler* m_sampler;
};

extern GridCollisionChecker* global_GridCollisionChecker;

#endif
