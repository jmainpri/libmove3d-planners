//  Created by Jim Mainprice on 23/04/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.

#include "AStarPlanner.hpp"

#include "API/ConfigSpace/configuration.hpp"
#include "Grid/HRICS_Grid.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "planner/TrajectoryOptim/plannarTrajectorySmoothing.hpp"
#include "planEnvironment.hpp"

#include "Graphic-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"

//std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > path_to_draw;

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE
using namespace Eigen;

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

PlanGrid::PlanGrid(Robot* R, double pace, vector<double> env_size) : API::TwoDGrid( pace, env_size ), robot_(R)
{
    createAllCells();
    cout << "Number total of cells = " << _nbCellsX*_nbCellsY << endl;
}

API::TwoDCell* PlanGrid::createNewCell(unsigned int index,unsigned  int x,unsigned  int y )
{
    Vector2i coord;
    coord[0] = x;
    coord[1] = y;

    API::TwoDCell* newCell;

    if (index == 0) {
        newCell = new PlanCell( 0, coord, _originCorner , this );
    }
    else {
        newCell = new PlanCell( index, coord, computeCellCorner(x,y) , this );
    }

    return newCell;
}

void PlanGrid::reset()
{
    for (int i=0;i<int(_cells.size());i++)
    {
        PlanCell* cell = dynamic_cast<PlanCell*>(_cells[i]);

        if( cell != NULL ){
            cell->resetExplorationStatus();
            cell->resetCost();
            cell->resetIsValid();
        }
    }
}

std::pair<double,double> PlanGrid::getMinMaxCost()
{
    double max = std::numeric_limits<double>::min();
    double min = std::numeric_limits<double>::max();

    for( int i=0;i<int(_cells.size());i++)
    {
        double cost = dynamic_cast<PlanCell*>(_cells[i])->getCost();

        if( cost > max ) max = cost;
        if( cost < min ) min = cost;
    }

    std::pair<double,double> result;
    result.first = min;
    result.second = max;
    return result;
}

void PlanGrid::draw()
{
    if( robot_ == 0x00 )
    {
        std::cout << "Error : PlanGrid::draw() => No Robot "  << std::endl;
    }

//    cout << "Draw plan grid" << endl;

    double colorvector[4];
    colorvector[0] = 0.0;  //red
    colorvector[1] = 0.0;  //green
    colorvector[2] = 0.0;  //blue
    colorvector[3] = 1.0;  //alpha

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    // glDisable(GL_LIGHTING);
    // glDisable(GL_LIGHT0);
    // glEnable(GL_CULL_FACE);

    std::pair<double,double> min_max = getMinMaxCost();
    double c_min = min_max.first;
    double c_max = min_max.second;

    // How to display better
    if( c_max > 100)
        c_max = 100;

    // cout << "c_min : " << c_min << " , c_max : " << c_max << endl;

    const double height = 0.0;
    // cout << "Drawing 2D Grid"  << endl;

    for (unsigned int x=0;x<_nbCellsX;++x)
    {
        for (unsigned int y=0;y<_nbCellsY;++y)
        {
            PlanCell* Cell = dynamic_cast<PlanCell*>(getCell(x,y));

            if (Cell == NULL || !Cell->isValid() ) {
                continue;
            }

            // double colorRation = ENV.getDouble(Env::colorThreshold1)- Cell->getCost();
            double cost = Cell->getCost();
            // GroundColorMix(color,colorRation*ENV.getDouble(Env::colorThreshold2)*1000,0,1);
            // GroundColorMixGreenToRed(colorvector,ENV.getDouble(Env::colorThreshold1)*colorRation/200);
            // GroundColorMixGreenToRed( colorvector, ENV.getDouble(Env::colorThreshold1)*colorRation/10 );

            GroundColorMixGreenToRed( colorvector, (cost-c_min)/(c_max-c_min) );

            g3d_set_color( Any, colorvector );

            // cout << "Cost of cell (" << x << " , " << y << ") = " << colorRation;
            // cout << " , colorvector : (" << colorvector[0] << " , " << colorvector[1] << " , " << colorvector[2] << " , " << colorvector[3] << ")" << endl;

            Vector2d center = Cell->getCenter();

            if( x==0 && y==0 ) {
                g3d_draw_solid_sphere(center[0], center[1], height, _cellSize[0]/5, 20);
            }
            g3d_draw_rectangle(center[0]-_cellSize[0]/2, center[1]-_cellSize[1]/2, height, _cellSize[0], _cellSize[1]);
        }
    }
    // glDisable(GL_CULL_FACE);

    glDisable(GL_BLEND);
    // glEnable(GL_LIGHTING);
    // glEnable(GL_LIGHT0);
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

PlanCell::PlanCell(int i, Vector2i coord, Vector2d corner, PlanGrid* grid) :
    API::TwoDCell(i,corner,grid),
    coord_(coord),
    open_(false),
    closed_(false),
    cost_is_computed_(false),
    cost_(0.0),
    is_cell_tested_(false),
    is_valid_(false)
{
}

confPtr_t PlanCell::setRobotAtCenter()
{
    Robot* rob = dynamic_cast<PlanGrid*>(_grid)->getRobot();
    Vector2d center = getCenter();
    confPtr_t q = rob->getCurrentPos();
    (*q)[6] = center[0];
    (*q)[7] = center[1];
    return q;
}

double PlanCell::getCost()
{
    if (ENV.getBool(Env::isCostSpace))
    {
        if( cost_is_computed_ /*&& (!ENV.getBool(Env::RecomputeCellCost))*/ )
        {
            return cost_;
        }

        confPtr_t q = setRobotAtCenter();
        cost_ = q->cost();
        cost_is_computed_ = true;
        return cost_;
    }
    else
    {
        return 1;
    }
}

bool PlanCell::isValid()
{
    if( is_cell_tested_ ) {
        return is_valid_;
    }

    is_valid_ = false;

    confPtr_t q = setRobotAtCenter();

    if ( dynamic_cast<PlanGrid*>(_grid)->getRobot()->setAndUpdate(*q) )
    {
        is_valid_ = !q->isInCollision();

        //    if( !mIsValid ) {
        //      p3d_print_col_pair();
        //    }
        //    else {
        //      cout << "Valid cell for robot : " << q->getRobot()->getName() << endl;
        //    }
    }

    is_cell_tested_ = true;
    return is_valid_;
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

PlanState::PlanState( Vector2i cell , PlanGrid* grid) : grid_(grid)
{
    cell_ = dynamic_cast<PlanCell*>( grid->getCell(cell) );
}

PlanState::PlanState( PlanCell* cell , PlanGrid* grid ) : grid_(grid), cell_(cell)
{

}

vector<API::State*> PlanState::getSuccessors(API::State* s)
{
    vector<API::State*> newStates;
    // newStates.reserve(26);

    vector<int> remove(3);
    remove[0]=-1; remove[1]=-1; remove[2]=-1;

    Vector2i coord2 = cell_->getCoord();

    if(s)
    {
        Vector2i coord1 = dynamic_cast<PlanState*>(s)->cell_->getCoord();

        Vector2i coord = coord1 - coord2;

        int dir = (coord[0]+1) + (coord[1]+1)*3;

        switch (dir)
        {
        case 0: remove[0]=0; remove[1]=1; remove[2]=3; break;
        case 1: remove[0]=1; remove[1]=0; remove[2]=2; break;
        case 2: remove[0]=2; remove[1]=1; remove[2]=5; break;
        case 3: remove[0]=3; remove[1]=6; remove[2]=0; break;
        case 4: remove[0]=4; remove[1]=4; remove[2]=4; break;
        case 5: remove[0]=5; remove[1]=8; remove[2]=2; break;
        case 6: remove[0]=6; remove[1]=3; remove[2]=7; break;
        case 7: remove[0]=7; remove[1]=6; remove[2]=8; break;
        case 8: remove[0]=8; remove[1]=7; remove[2]=5; break;
        };
    }

    for(int i=0;i<8;i++)
    {
        if( i == remove[0] || i == remove[1] || i == remove[2] ){
            continue;
        }

        PlanCell* neigh = dynamic_cast<PlanCell*>( grid_->getNeighbour(coord2,i) );
        if( neigh != NULL )
        {
            newStates.push_back(new PlanState( neigh, grid_ ));
        }
    }

    return newStates;
}

bool PlanState::isLeaf()
{
    return false;
}

bool PlanState::isValid()
{
    return cell_->isValid();
}

bool PlanState::equal(API::State* other)
{
    // bool equal(false);
    PlanState* state = dynamic_cast<PlanState*>(other);

    if( cell_ != state->cell_ )
    {
        // cout << "PlanState::equal false" << endl;
        return false;
    }

    // cout << "State::equal true" << endl;
    return true;
}

void PlanState::setClosed(std::vector<PlanState*>& closedStates,std::vector<PlanState*>& openStates)
{
    //    cout << "State :: set Closed" <<endl;
    cell_->setClosed();
}

bool PlanState::isColsed(std::vector<PlanState*>& closedStates)
{
    //    cout << "State :: get Closed" <<endl;
    return cell_->getClosed();
}

void PlanState::setOpen(std::vector<PlanState*>& openStates)
{
    //     cout << "State :: set open" <<endl;
    cell_->setOpen();
}


bool PlanState::isOpen(std::vector<PlanState*>& openStates)
{
    //    cout << "State :: get open" <<endl;
    return cell_->getOpen();
}

void PlanState::reset()
{
    cell_->resetExplorationStatus();
}

void PlanState::print()
{

}

double PlanState::computeLength(API::State *parent)
{
    PlanState* preced = dynamic_cast<PlanState*>(parent);

    Vector2d pos1 = cell_->getCenter();
    Vector2d pos2 = preced->cell_->getCenter();

    double g;
    double dist = ( pos1 - pos2 ).norm();

    if( ENV.getBool(Env::isCostSpace) ) {
        g = preced->g() + cell_->getCost()*dist;
    }
    else {
        g = preced->g() + dist;
    }
    return g;
}

double PlanState::computeHeuristic( API::State *parent, API::State* goal )
{
    if( !ENV.getBool(Env::isCostSpace) )
    {
        PlanState* state = dynamic_cast<PlanState*>(goal);
        Vector2d posGoal = state->cell_->getCenter();
        Vector2d posThis = cell_->getCenter();
        return ( posGoal - posThis ).norm();
    }
    else {
        return 0.0;
    }
}

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

AStarPlanner::AStarPlanner(Robot* R) : Planner(R,NULL)
{
    pace_ = 1.0;

    // double diagonal = std::sqrt( std::pow(m_envSize[1]-m_envSize[0], 2 ) + std::pow(m_envSize[3]-m_envSize[2] , 2 ) );
    // double pace = 0.20;
//    init();
}

AStarPlanner::~AStarPlanner()
{
    delete grid_;
}

unsigned int AStarPlanner::init()
{
    env_size_ = global_Project->getActiveScene()->getBounds();
    env_size_.resize(4);

    cout << "pace : " << pace_ << " meters" << endl;

    grid_ = new PlanGrid( _Robot,/*ENV.getDouble(Env::PlanCellSize)*/ pace_, env_size_ );
    API_activeGrid = grid_;

    return 1;
}

void AStarPlanner::reset()
{
    grid_->reset();
}

bool AStarPlanner::solveAStar( PlanState* start, PlanState* goal )
{
    bool path_exists=true;
    path_.clear();

    // Change the way AStar is computed to go down
    if( start->getCell()->getCost() < goal->getCell()->getCost() )
    {
        API::AStar* search = new API::AStar(start);
        vector<API::State*> path = search->solve(goal);

        if(path.size() == 0 )
        {
            path_.clear();
            cell_path_.clear();
            path_exists = false;
            return path_exists;
        }

        for (unsigned int i=0;i<path.size();i++)
        {
            API::TwoDCell* cell = dynamic_cast<PlanState*>(path[i])->getCell();
            path_.push_back( cell->getCenter() );
            cell_path_.push_back( cell );
        }
    }
    else
    {
        API::AStar* search = new API::AStar(goal);
        vector<API::State*> path = search->solve(start);

        if(path.size() == 0 )
        {
            path_.clear();
            cell_path_.clear();
            path_exists = false;
            return path_exists;
        }

        for (int i=path.size()-1;i>=0;i--)
        {
            PlanCell* cell = dynamic_cast<PlanState*>(path[i])->getCell();
            path_.push_back( cell->getCenter() );
            cell_path_.push_back( cell );
        }
    }

    return path_exists;
}

bool AStarPlanner::computeAStarIn2DGrid( Vector2d source, Vector2d target )
{
    PlanCell* startCell = dynamic_cast<PlanCell*>(grid_->getCell(source));
    if( startCell == NULL )
    {
        cout << "start (" << source.transpose() << ") not in grid" << endl;
        return false;
    }

    PlanCell* goalCell = dynamic_cast<PlanCell*>(grid_->getCell(target));
    if( goalCell == NULL )
    {
        cout << "goal (" << target.transpose() << ") not in grid" << endl;
        return false;
    }

    Vector2i startCoord = startCell->getCoord();
    cout << "Start Pos = (" << source[0] << " , " << source[1] << ")" << endl;
    cout << "Start Coord = (" << startCoord[0] << " , " << startCoord[1] << ")" << endl;

    Vector2i goalCoord = goalCell->getCoord();
    cout << "Goal Pos = (" << target[0] << " , " << target[1] << ")" << endl;
    cout << "Goal Coord = (" << goalCoord[0] << " , " << goalCoord[1] << ")" << endl;

    if( startCoord == goalCoord )
    {
        cout << " no planning as cells are identical" << endl;
        return false;
    }

    PlanState* start    = new PlanState( startCell, grid_ );
    PlanState* goal     = new PlanState( goalCell,  grid_ );
    if ( start == NULL || goal == NULL ) {
        cout << "Start or goal == NULL" << endl;
        return false;
    }

    if( solveAStar( start, goal ) )
    {
        double SumOfCost= 0.0;
        for(int i=0; i< int(path_.size()); i++ )
        {
            //cout << "Cell "<< i <<" = " << endl << path_[i] << endl;
            SumOfCost +=  dynamic_cast<PlanCell*>(cell_path_[i])->getCost();
        }
        cout << " SumOfCost = "  << SumOfCost << endl;
        return true;
    }
    else {
        return false;
    }
}

API::Trajectory* AStarPlanner::computeRobotTrajectory( confPtr_t source, confPtr_t target )
{
    confPtr_t q = _Robot->getCurrentPos();
    (*q)[6] =0;
    (*q)[7] =0;

    Vector2d x1,x2;

    x1[0]=(*source)[6];
    x1[1]=(*source)[7];

    x2[0]=(*target)[6];
    x2[1]=(*target)[7];

    _Robot->setAndUpdate(*source);

    if( computeAStarIn2DGrid( x1, x2 ) )
    {
        API::Trajectory* traj = new API::Trajectory(_Robot);

        traj->push_back( source );

        for(int i=0;i<int(path_.size());i++)
        {
            confPtr_t q = _Robot->getCurrentPos();
            (*q)[6] = path_[i][0];
            (*q)[7] = path_[i][1];

            traj->push_back( q );
        }
        traj->push_back( target );

        traj->replaceP3dTraj();
        _Robot->setAndUpdate(*q);
        return traj;
    }
    else
    {
        _Robot->setAndUpdate(*q);
        return NULL;
    }
}

void AStarPlanner::draw()
{
    for(int i=0;i<int(path_.size())-1;i++)
    {
        glLineWidth(3.);
        g3d_drawOneLine(path_[i][0], path_[i][1], 0.4,
                        path_[i+1][0], path_[i+1][1], 0.4, Yellow, NULL);
        glLineWidth(1.);
    }
}

void AStarPlanner::allow_smoothing(bool state)
{
    PlanEnv->setBool(PlanParam::env_createTrajs,state);
}
