#include "HRICS_rrtPlanExpansion.hpp"
#include "../HRICS_ConfigSpace.hpp"
#include "API/Grids/PointCloud.hpp"

#include "Planner-pkg.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

HRICS_rrtPlanExpansion::HRICS_rrtPlanExpansion() :
        TransitionExpansion(),
        mForward(true),
        mBiasing(false)
{
    this->init();
}

HRICS_rrtPlanExpansion::HRICS_rrtPlanExpansion(Graph* ptrGraph) :
        TransitionExpansion(ptrGraph),
        mForward(true),
        mBiasing(false)
{
    this->init();
}

/**
  * Computes a box for the free flyer
  * and the index of the Object Dof
  */
void HRICS_rrtPlanExpansion::init()
{
    cout << "Init Box Jido" << endl;
//    double box[] = {-1.3,1.3,-1.3,1.3,0,1.5};
    shared_ptr<Configuration> qInit = mGraph->getRobot()->getInitialPosition();

//    _Box = new double[6];
//
//    _Box[0] = box[0] + qInit->getConfigStruct()[6];
//    _Box[1] = box[1] + qInit->getConfigStruct()[6];
//    _Box[2] = box[2] + qInit->getConfigStruct()[7];
//    _Box[3] = box[3] + qInit->getConfigStruct()[7];
//    _Box[4] = box[4];
//    _Box[5] = box[5];

//    mIndexObjectDof = mGraph->getRobot()->getObjectDof();

}

/**
  * Sets the Cell path, First and Last Cell
  */
void HRICS_rrtPlanExpansion::setCellPath(vector<API::TwoDCell*> cellPath)
{
    m2DCellPath = cellPath;
    mLastForward = cellPath.at(0);
    mLastBackward = cellPath.back();
}

/**
  * Main Function for the 2D Biased from A*
  * Cost Map RRT
  */
//int Direction=0;
shared_ptr<Configuration> HRICS_rrtPlanExpansion::getExpansionDirection(
        Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode)
{
    shared_ptr<Configuration> q;

    mBiasing = ENV.getBool(Env::isGoalBiased) && p3d_random(0.,1.) <= ENV.getDouble(Env::Bias);

    if( mBiasing )
    {
        q = getConfigurationInNextCell(expandComp);
    }
    else
    {
//        if(ENV.getBool(Env::isInverseKinematics))
//        {
//            q = mGraph->getRobot()->shootFreeFlyer(_Box);
//        }
//        else
//        {
            q = mGraph->getRobot()->shoot(samplePassive);
//        }
    }

    if(ENV.getBool(Env::drawPoints))
    {
        if(PointsToDraw==NULL)
        {
            PointsToDraw = new PointCloud();
        }
        Vector3d randomPoint;
        randomPoint[0] = q->at(6);
        randomPoint[1] = q->at(7);
        randomPoint[2] = -0.40;
        PointsToDraw->push_back(randomPoint);
    }

    //    cout << ++Direction << " New Direction (Biased = " << biasing << ") " << endl;
    return q;
}

shared_ptr<Configuration> HRICS_rrtPlanExpansion::getConfigurationInNextCell(Node* CompcoNode)
{

    API::TwoDCell* farthestCell;

    // Get the farthest cell explored depending on
    // the way the tree explores
    if( CompcoNode->equalCompco( mGraph->getStart() ) )
    {
        mForward = true;
        farthestCell = mLastForward;
    }
    else
    {
        mForward = false;
        farthestCell = mLastBackward;
    }

    int cellId;
    // Get Id of Next cell on the 2D Path
    for(int i=0; i<(int)m2DCellPath.size(); i++)
    {
        if(m2DCellPath[i] == farthestCell )
        {
            if( mForward  )
            {
                i++;
                if( i == (int)m2DCellPath.size() )
                {
                    i = m2DCellPath.size()-1;
                }
                cellId = i;
                break;
            }
            else
            {
                i--;
                if( i == -1 )
                {
                    i = 0;
                }
                cellId = i;
                break;
            }
        }
    }

//    shared_ptr<Configuration> q = mGraph->getRobot()->shoot(false);

    // Get a random config in the cell
//    randomPoint = _2DCellPath[cellId]->getRandomPoint();
    mBiasedPlanCell = m2DCellPath[cellId];

    shared_ptr<Configuration> q(new Configuration(mGraph->getRobot()));

//    Vector2d corner = mBiasedPlanCell->getCorner();
//    Vector2d cellSize = mBiasedPlanCell->getCellSize();
//    double biasedBox[4];
//    biasedBox[0] = corner[0];
//    biasedBox[1] = corner[0] + cellSize[0];
//    biasedBox[2] = corner[1];
//    biasedBox[3] = corner[1] + cellSize[1];
//    p3d_ShootInCell( mGraph->getRobot()->getRobotStruct() , q->getConfigStruct() , biasedBox , 0 );

    Vector2d center = mBiasedPlanCell->getCenter();
    double CellCenter[2];
    CellCenter[0] = center[0];
    CellCenter[1] = center[1];
	
#ifdef P3D_PLANNER
    p3d_ShootAroundPoint( mGraph->getRobot()->getRobotStruct() , q->getConfigStruct(), CellCenter , 0);
#else
	printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif

    return q;
}

/**
  * Return true if the cell is on the path
  * and after the first cell depending on the order (forward or backwards)
  */
bool HRICS_rrtPlanExpansion::on2DPathAndAfter(API::TwoDCell* cell)
{
    // Is cell on path
    bool cellOnPath;

    for(unsigned int i=0;i<m2DCellPath.size();i++)
    {
        if(cell == m2DCellPath[i])
        {
            cellOnPath = true;
            break;
        }
    }

    if( cellOnPath == false )
    {
//        cout << "Not on path" << endl;
        return false;
    }

    //    cout << "Cell on Path" << endl;
    //    for(unsigned int i=0;i<_2DCellPath.size();i++)
    //    {
    //        cout << i << " => " << _2DCellPath[i] << endl;
    //    }

    //    cout << "Cell = " << cell << endl;
    //    cout << "Last = " << _LastForward << endl;

    // Looks if it is the one further away
    // forward and backwards
    if(mForward)
    {
        for(unsigned int i=0;i<m2DCellPath.size();i++)
        {
            if( m2DCellPath[i] == mLastForward )
            {
                //                cout << "After" << endl;
                return true;
            }
            if( m2DCellPath[i] == cell )
            {
                //                cout << "Before" << endl;
                return false;
            }
        }

    }
    else
    {
        for(unsigned int i=m2DCellPath.size()-1;i>=0;i--)
        {
            if( m2DCellPath[i] == mLastBackward )
            {
                return true;
            }
            if( m2DCellPath[i] == cell )
            {
                return false;
            }
        }
    }
	
	return false;
}

/**
  * Adds a node and checks
  * if it explores the path
  */
Node* HRICS_rrtPlanExpansion::addNode(Node* currentNode, LocalPath& path, double pathDelta,
                                  Node* directionNode, int& nbCreatedNodes)
{
    Node* newNode = BaseExpansion::addNode(
            currentNode,path,pathDelta,directionNode,nbCreatedNodes);

    //    cout << "New Node " << endl;

//    cout << "New node Biased = " << _biasing << endl;

//    newNode->getConfiguration()->print();

    Vector2d pos;

    pos[0] = currentNode->getConfiguration()->at(6);
    pos[1] = currentNode->getConfiguration()->at(7);

    API::TwoDCell* cell = m2DGrid->getCell(pos);

    if( currentNode->equalCompco( mGraph->getStart() ) )
    {
        if(mLastForward != cell)
        {
            if( on2DPathAndAfter( cell ) )
            {
                mLastForward = cell;
            }
        }
        mForward = true;
    }
    else
    {
        if(mLastBackward != cell)
        {
            if( on2DPathAndAfter( cell ) )
            {
                mLastBackward = cell;
            }
        }
        mForward = false;
    }

    return newNode;
}
