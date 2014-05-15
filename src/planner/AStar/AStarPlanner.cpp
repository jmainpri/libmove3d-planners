//  Created by Jim Mainprice on 23/04/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.

#include "AStarPlanner.hpp"

#include "API/project.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Search/AStar/AStar.hpp"

#include "hri_costspace/grid/HRICS_grid.hpp"

#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "planner/TrajectoryOptim/plannarTrajectorySmoothing.hpp"
#include "planner/planEnvironment.hpp"

#include "Graphic-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"

//std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > path_to_draw;

using namespace Move3D;
using namespace std;
using namespace Eigen;

MOVE3D_USING_SHARED_PTR_NAMESPACE

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

PlanGrid::PlanGrid(Robot* R, double pace, vector<double> env_size, bool print_cost ) : Move3D::TwoDGrid( pace, env_size ), robot_(R), print_cost_(print_cost), use_given_bounds_(false)
{
    createAllCells();
    cout << "Number total of cells = " << _nbCellsX*_nbCellsY << endl;
}

Move3D::TwoDCell* PlanGrid::createNewCell(unsigned int index,unsigned  int x,unsigned  int y )
{
    Vector2i coord;
    coord[0] = x;
    coord[1] = y;

    Move3D::TwoDCell* newCell;

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

    if( !use_given_bounds_ ) {
        std::pair<double,double> min_max = getMinMaxCost();
        min_cost_ = min_max.first;
        max_cost_ = min_max.second;
    }

    // How to display better
    if( max_cost_ > 100 )
        max_cost_ = 100;

    if( print_cost_ )
        cout << "c_min : " << min_cost_ << " , c_max : " << max_cost_ << endl;

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

            GroundColorMixGreenToRed( colorvector, (cost-min_cost_)/(max_cost_-min_cost_) );

            g3d_set_color( Any, colorvector );

            // cout << "Cost of cell (" << x << " , " << y << ") = " << cost << endl;
            //cout << " , colorvector : (" << colorvector[0] << " , " << colorvector[1] << " , " << colorvector[2] << " , " << colorvector[3] << ")" << endl;

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
    Move3D::TwoDCell(i,corner,grid),
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
        //g3d_draw_allwin_active();
        cost_is_computed_ = true;
        return cost_;
    }
    else
    {
        if( cost_is_computed_ )
            return cost_;
        else
            return 0.0;
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

vector<Move3D::State*> PlanState::getSuccessors(Move3D::State* s)
{
    vector<Move3D::State*> newStates;
    // newStates.reserve(26);

    vector<int> remove(3);
    remove[0]=-1; remove[1]=-1; remove[2]=-1;

    Vector2i coord2 = cell_->getCoord();

    if(s)
    {
        Vector2i coord1 = dynamic_cast<PlanState*>(s)->cell_->getCoord();

        Vector2i coord = coord1 - coord2;

        int dir = (coord[0]+1) + (coord[1]+1)*3;

        // Remove directions that go back
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
//        if( i == remove[0] || i == remove[1] || i == remove[2] ){
//            continue;
//        }

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

bool PlanState::equal(Move3D::State* other)
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

double PlanState::computeLength(Move3D::State *parent)
{
    PlanState* preced = dynamic_cast<PlanState*>(parent);

    double g;

    Vector2d pos1 = cell_->getCenter();
    Vector2d pos2 = preced->cell_->getCenter();


    if( ENV.getBool(Env::isCostSpace) )
    {
        confPtr_t q1 = grid_->getRobot()->getNewConfig();
        confPtr_t q2 = grid_->getRobot()->getNewConfig();
        (*q1)[6] = pos1[0]; (*q2)[6] = pos2[0];
        (*q1)[7] = pos1[1]; (*q2)[7] = pos2[1];
        LocalPath path( q2, q1 );
        g = preced->g() + path.cost();
    }
    else {
        g = preced->g() + ( pos1 - pos2 ).norm();
        cell_->setCost( g );
    }
    return g;
}

double PlanState::computeHeuristic( Move3D::State *parent, Move3D::State* goal )
{
//    return 0.0;

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
    cout << "Create planner for robot : " << R->getName() << endl;
    pace_ = 1.0;

    // double diagonal = std::sqrt( std::pow(m_envSize[1]-m_envSize[0], 2 ) + std::pow(m_envSize[3]-m_envSize[2] , 2 ) );
    // double pace = 0.20;
//    init();
}

AStarPlanner::~AStarPlanner()
{
//    if( grid_ == API_activeGrid )
//        API_activeGrid = NULL;

//    delete grid_;
}

unsigned int AStarPlanner::init()
{
    env_size_ = global_Project->getActiveScene()->getBounds();
    env_size_.resize(4);

    cout << "pace : " << pace_ << " meters" << endl;

    grid_ = new PlanGrid( _Robot,/*ENV.getDouble(Env::PlanCellSize)*/ pace_, env_size_ );

    if( API_activeGrid != NULL )
    {
        delete API_activeGrid;
    }
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
        Move3D::AStar* search = new Move3D::AStar(start);
        vector<Move3D::State*> path = search->solve(goal);

        if(path.size() == 0 )
        {
            path_.clear();
            cell_path_.clear();
            path_exists = false;
            return path_exists;
        }

        for (unsigned int i=0;i<path.size();i++)
        {
            Move3D::TwoDCell* cell = dynamic_cast<PlanState*>(path[i])->getCell();
            path_.push_back( cell->getCenter() );
            cell_path_.push_back( cell );
        }
    }
    else
    {
        Move3D::AStar* search = new Move3D::AStar(goal);
        vector<Move3D::State*> path = search->solve(start);

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

Move3D::Trajectory* AStarPlanner::computeRobotTrajectory( confPtr_t source, confPtr_t target )
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
        Move3D::Trajectory* traj = new Move3D::Trajectory(_Robot);

        traj->push_back( source );

        for(int i=0;i<int(path_.size());i++)
        {
            confPtr_t q = _Robot->getCurrentPos();
            (*q)[6] = path_[i][0];
            (*q)[7] = path_[i][1];

            traj->push_back( q );
        }
        traj->push_back( target );
        traj->computeSubPortionIntergralCost( traj->getCourbe() );
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
