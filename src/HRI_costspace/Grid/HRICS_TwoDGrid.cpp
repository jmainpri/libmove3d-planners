#include "HRICS_TwoDGrid.hpp"

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

#include "P3d-pkg.h"
#include "Graphic-pkg.h"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

//---------------------------------------------------------------------------
// Grid
//---------------------------------------------------------------------------
PlanGrid::PlanGrid(Robot* R, double pace, vector<double> envSize) : API::TwoDGrid(pace,envSize), mRobot(R)
{
  createAllCells();
  cout << "Number total of cells = " << _nbCellsX*_nbCellsY << endl;
}

/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
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

//----------------------------------------
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

//----------------------------------------
void PlanGrid::draw()
{
  if( mRobot == 0x00 )
  {
    std::cout << "Error : PlanGrid::draw() => No Robot "  << std::endl;
  }
	
  double colorvector[4];
  colorvector[0] = 0.0;  //red
  colorvector[1] = 0.0;  //green
  colorvector[2] = 0.0;  //blue
  colorvector[3] = 1.0;  //alpha
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //glBlendFunc(GL_SRC_ALPHA, GL_ONE);
  
  //  glDisable(GL_LIGHTING);
  //  glDisable(GL_LIGHT0);
  //  glEnable(GL_CULL_FACE);
  
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
      double colorRation = Cell->getCost();
      // GroundColorMix(color,colorRation*ENV.getDouble(Env::colorThreshold2)*1000,0,1);
      GroundColorMixGreenToRed(colorvector,ENV.getDouble(Env::colorThreshold1)*colorRation/200);
      
      g3d_set_color( Any, colorvector );
      
      //      cout << "Cost of cell (" << x << " , " << y << ") = " << colorRation;
      //      cout << " , colorvector : (" 
      //      << colorvector[0] << " , " << colorvector[1] << " , " 
      //      << colorvector[2] << " , " << colorvector[3] << ")" << endl;
      
      Vector2d center = Cell->getCenter();
      
      if( x==0 && y==0 ) {
        g3d_draw_solid_sphere(center[0], center[1], height, _cellSize[0]/5, 20);
      }
      g3d_draw_rectangle(center[0]-_cellSize[0]/2, center[1]-_cellSize[1]/2, height, _cellSize[0], _cellSize[1]);
    }
  }
  //  glDisable(GL_CULL_FACE);
  
  glDisable(GL_BLEND);
  //  glEnable(GL_LIGHTING);
  //  glEnable(GL_LIGHT0);
}

//----------------------------------------
//! Write a Cost Tab to an ObPlane format, this output has
//! to go through a script to be used as a Cost Map
void PlanGrid::writeToOBPlane() {
  
  FILE* fd = fopen("OB_Plane.macro", "w");
  
  fprintf(fd, "p3d_beg_desc P3D_OBSTACLE\n\n\n");
  fprintf(fd, "\tp3d_add_desc_poly polyhedre1\n");
  
  const double xMin=-25;
  const double xMax=25;
  
  const double yMin=-25;
  const double yMax=25;
  
  for (unsigned int a=0; a<_nbCellsX; a++) {
    for (unsigned int b=0; b<_nbCellsY; b++) {
      PlanCell* Cell = dynamic_cast<PlanCell*>(this->getCell(a,b));
      fprintf(fd, "\t\tp3d_add_desc_vert %.9f %.9f %.9f\n",
              (xMax-xMin)*(((double)a)/((double)(_nbCellsX-1)))+xMin,
              (yMax-yMin)*(((double)b)/((double)(_nbCellsY-1)))+yMin,
              Cell->getCost());
    }
  }
  
  fprintf(fd, "\n");
  
  for (unsigned int a=0; a<_nbCellsX-1; a++) {
    for (unsigned int b=0; b<_nbCellsY-1; b++) {
      fprintf(fd, "\t\tp3d_add_desc_face %d %d %d \n",
              b*_nbCellsX + a +1,
              (b+1)*_nbCellsX + a +1,
              b*_nbCellsX + (a+1)+1);
      
      fprintf(fd, "\t\tp3d_add_desc_face %d %d %d \n",
              (b+1)*_nbCellsX + a+1,
              (b+1)*_nbCellsX + (a+1)+1,
              b*_nbCellsX + (a+1)+1);
    }
  }
  
  fprintf(fd, "    p3d_end_desc_poly\n");
  fprintf(fd,
          "    p3d_set_prim_pos_by_mat polyhedre1 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n");
  fprintf(fd, "p3d_end_desc\n\n");
  fprintf(fd, "p3d_set_obst_poly_color 1 Any 0.8 0.8 0.8\n");
  
  fclose(fd);
  
  cout << "Save cost tab to ObPlane format"<<endl;
}


//---------------------------------------------------------------------------
// Cell
//---------------------------------------------------------------------------

PlanCell::PlanCell(int i, Vector2i coord, Vector2d corner, PlanGrid* grid) :
API::TwoDCell(i,corner,grid),
_Coord(coord),
_Open(false),
_Closed(false),
mCostIsComputed(false),
mCost(0.0),
mIsCellTested(false),
mIsValid(false)
{
}

//----------------------------------------
confPtr_t PlanCell::setRobotAtCenter()
{
  Robot* rob = dynamic_cast<PlanGrid*>(_grid)->getRobot();
  Vector2d center = getCenter();
  confPtr_t q = rob->getCurrentPos();
  (*q)[6] = center[0];
  (*q)[7] = center[1];
  return q;
}

//----------------------------------------
double PlanCell::getCost()
{
    if (ENV.getBool(Env::isCostSpace))
    {
        if(mCostIsComputed /*&& (!ENV.getBool(Env::RecomputeCellCost))*/)
        {
            return mCost;
        }

        confPtr_t q = setRobotAtCenter();
        mCost = q->cost();
        mCostIsComputed = true;
        return mCost;
    }
    else
    {
        return 1;
    }
}

//----------------------------------------
bool PlanCell::isValid()
{
  if(mIsCellTested)
  {
    return mIsValid;
  }
  
  confPtr_t q = setRobotAtCenter();
  mIsValid = !q->isInCollision();
  mIsCellTested = true;
  return mIsValid;
}

//---------------------------------------------------------------------------
// State
//---------------------------------------------------------------------------
PlanState::PlanState( Vector2i cell , PlanGrid* grid) :
_Grid(grid)
{
  _Cell = dynamic_cast<PlanCell*>(grid->getCell(cell));
}

PlanState::PlanState( PlanCell* cell , PlanGrid* grid ) :
_Grid(grid),
_Cell(cell)
{
  
}

vector<API::State*> PlanState::getSuccessors(API::State* s)
{
    vector<API::State*> newStates;
    // newStates.reserve(26);

    vector<int> remove(3);
    remove[0]=-1; remove[1]=-1; remove[2]=-1;

    Vector2i coord2 = _Cell->getCoord();

    if(s)
    {
        Vector2i coord1 = dynamic_cast<PlanState*>(s)->_Cell->getCoord();

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
        if( i == remove[0] || i == remove[1] || i == remove[2]  ){
            continue;
        }

      PlanCell* neigh = dynamic_cast<PlanCell*>(_Grid->getNeighbour(coord2,i));
      if( neigh != NULL )
      {
          newStates.push_back(new PlanState(neigh,_Grid));
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
  return _Cell->isValid();
}

bool PlanState::equal(API::State* other)
{
  // bool equal(false);
  PlanState* state = dynamic_cast<PlanState*>(other);
  
  if( _Cell != state->_Cell )
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
  _Cell->setClosed();
}

bool PlanState::isColsed(std::vector<PlanState*>& closedStates)
{
  //    cout << "State :: get Closed" <<endl;
  return _Cell->getClosed();
}

void PlanState::setOpen(std::vector<PlanState*>& openStates)
{
  //     cout << "State :: set open" <<endl;
  _Cell->setOpen();
}


bool PlanState::isOpen(std::vector<PlanState*>& openStates)
{
  //    cout << "State :: get open" <<endl;
  return _Cell->getOpen();
}

void PlanState::reset()
{
  _Cell->resetExplorationStatus();
}

void PlanState::print()
{
  
}

double PlanState::computeLength(API::State *parent)
{
  PlanState* preced = dynamic_cast<PlanState*>(parent);
  
  Vector2d pos1 = _Cell->getCenter();
  Vector2d pos2 = preced->_Cell->getCenter();
  
  double dist = ( pos1 - pos2 ).norm();
  
//  double cost1 = preced->_Cell->getCost();
//  double cost2 = _Cell->getCost();
  double g = preced->g() + /*cost1 + cost2 */ dist;
  
  //    cout << "dist = " << dist << endl;
  //    cout << "g = " << g << endl;
  return g;
}

double PlanState::computeHeuristic( API::State *parent, API::State* goal )
{
//  PlanState* state = dynamic_cast<PlanState*>(goal);
//  
//  Vector2d posGoal = state->_Cell->getCenter();
//  Vector2d posThis = _Cell->getCenter();
//  
//  double dist=0;
//  dist += ( posGoal - posThis ).norm();
  return 0.0;
}
