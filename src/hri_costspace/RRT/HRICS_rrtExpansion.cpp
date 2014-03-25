#include "HRICS_rrtExpansion.hpp"
#include "../HRICS_Workspace.hpp"
#include "API/Grids/PointCloud.hpp"
#include "API/Roadmap/graph.hpp"

#include "Planner-pkg.h"

using namespace std;
using namespace HRICS;
using namespace Eigen;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

HRICS_rrtExpansion::HRICS_rrtExpansion() :
    TransitionExpansion(),
    _biasing(false)
{
    this->init();
}

HRICS_rrtExpansion::HRICS_rrtExpansion(Graph* ptrGraph) :
    TransitionExpansion(ptrGraph),
    _biasing(false)
{
    this->init();
}

/**
  * Computes a box for the free flyer
  * and the index of the Object Dof
  */
void HRICS_rrtExpansion::init()
{
    cout << "Init Box Jido" << endl;
    double box[] = {-1.3,1.3,-1.3,1.3,0,1.5};
    confPtr_t qInit = m_Graph->getRobot()->getInitPos();

    _Box = new double[6];

    _Box[0] = box[0] + qInit->getConfigStruct()[6];
    _Box[1] = box[1] + qInit->getConfigStruct()[6];
    _Box[2] = box[2] + qInit->getConfigStruct()[7];
    _Box[3] = box[3] + qInit->getConfigStruct()[7];
    _Box[4] = box[4];
    _Box[5] = box[5];

#ifdef LIGHT_PLANNER
    mIndexObjectDof = m_Graph->getRobot()->getObjectDof();
#endif
}

/**
  * Sets the Cell path, First and Last Cell
  */
void HRICS_rrtExpansion::setCellPath(vector<Move3D::ThreeDCell*> cellPath)
{
    _3DCellPath = cellPath;
    _LastForward = cellPath.at(0);
    _LastBackward = cellPath.back();
}

/**
  *
  */
int Direction=0;
shared_ptr<Configuration> HRICS_rrtExpansion::getExpansionDirection(
        Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode)
{
    shared_ptr<Configuration> q;

    _biasing = ENV.getBool(Env::isGoalBiased) && p3d_random(0.,1.) <= ENV.getDouble(Env::Bias);

    if( _biasing )
    {
        q = getConfigurationInNextCell(expandComp);
    }
    else
    {
        //        if(ENV.getBool(Env::isInverseKinematics))
        //        {
        //            q = m_Graph->getRobot()->shootFreeFlyer(_Box);
        //        }
        //        else
        //        {
        q = m_Graph->getRobot()->shoot(samplePassive);
        //        }
    }

    if(ENV.getBool(Env::drawPoints))
    {
        if(PointsToDraw==NULL)
        {
            PointsToDraw = new PointCloud();
        }
        Vector3d randomPoint;
        randomPoint[0] = q->getConfigStruct()[mIndexObjectDof+0];
        randomPoint[1] = q->getConfigStruct()[mIndexObjectDof+1];
        randomPoint[2] = q->getConfigStruct()[mIndexObjectDof+2];
        PointsToDraw->push_back(randomPoint);
    }

    //    cout << ++Direction << " New Direction (Biased = " << biasing << ") " << endl;
    return q;
}

Move3D::ThreeDCell* BiasedCell=NULL;

shared_ptr<Configuration> HRICS_rrtExpansion::getConfigurationInNextCell(Node* CompcoNode)
{
    Move3D::ThreeDCell* farthestCell=NULL;

    // Get the farthest cell explored depending on
    // the way the tree explores
    //WARNING BROKEN
    //    if( CompcoNode->equalCompco( m_Graph->getStart() ) )
    //    {
    //        _forward = true;
    //        farthestCell = _LastForward;
    //    }
    //    else
    //    {
    //        _forward = false;
    //        farthestCell = _LastBackward;
    //    }

    //    Vector3d randomPoint;
    int cellId=0;

    // Get Id of Next cell on the 3D Path
    for(int i=0; i<int(_3DCellPath.size()); i++)
    {
        if(_3DCellPath[i] == farthestCell )
        {
            if( _forward  )
            {
                i++;
                if( i == (int)_3DCellPath.size() )
                {
                    i = _3DCellPath.size()-1;
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

    //    shared_ptr<Configuration> q = m_Graph->getRobot()->shoot(false);

    // Get a random config in the cell
    //    randomPoint = _3DCellPath[cellId]->getRandomPoint();
    BiasedCell = _3DCellPath[cellId];

    //     if(PointsToDraw==NULL)
    //     {
    //         PointsToDraw = new Points();
    //     }
    //
    //    PointsToDraw->push_back(randomPoint);
    //    g3d_draw_allwin_active();

    //    Matrix3d mat = Matrix3d::Identity();
    //    randomPoint =2*mat*randomPoint;

    shared_ptr<Configuration> q(new Configuration(m_Graph->getRobot()));

    Vector3d corner = BiasedCell->getCorner();
    Vector3d cellSize = BiasedCell->getCellSize();

    double biasedBox[6];

    biasedBox[0] = corner[0];
    biasedBox[1] = corner[0] + cellSize[0];
    biasedBox[2] = corner[1];
    biasedBox[3] = corner[1] + cellSize[1];
    biasedBox[4] = corner[2];
    biasedBox[5] = corner[2] + cellSize[2];

    p3d_FreeFlyerShoot( m_Graph->getRobot()->getP3dRobotStruct() , q->getConfigStruct() , biasedBox );

    //    q->getConfigStruct()[VIRTUAL_OBJECT_DOF+0] = randomPoint[0];
    //    q->getConfigStruct()[VIRTUAL_OBJECT_DOF+1] = randomPoint[1];
    //    q->getConfigStruct()[VIRTUAL_OBJECT_DOF+2] = randomPoint[2];

    return q;
}

/**
  * Return true if the cell is on the path
  * and after the first cell depending on the order (forward or backwards)
  */
bool HRICS_rrtExpansion::on3DPathAndAfter(Move3D::ThreeDCell* cell)
{
    // Is cell on path
    bool cellOnPath;

    for(unsigned int i=0;i<_3DCellPath.size();i++)
    {
        if(cell == _3DCellPath[i])
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

    // Looks if it is the one further away
    // forward and backwards
    if(_forward)
    {
        for(unsigned int i=0;i<_3DCellPath.size();i++)
        {
            if( _3DCellPath[i] == _LastForward )
            {
                //                cout << "After" << endl;
                return true;
            }
            if( _3DCellPath[i] == cell )
            {
                //                cout << "Before" << endl;
                return false;
            }
        }

    }
    else
    {
        for(unsigned int i=_3DCellPath.size()-1;i>=0;i--)
        {
            if( _3DCellPath[i] == _LastBackward )
            {
                return true;
            }
            if( _3DCellPath[i] == cell )
            {
                return false;
            }
        }
    }

    cout << "Warning: HRICS_rrtExpansion::on3DPathAndAfter" << endl;
    return false;
}

/**
  * Adds a node and checks
  * if it explores the path
  */
Node* HRICS_rrtExpansion::addNode(Node* currentNode, LocalPath& path, double pathDelta,
                                  Node* directionNode, int& nbCreatedNodes)
{
    Node* newNode = BaseExpansion::addNode(
                currentNode,path,pathDelta,directionNode,nbCreatedNodes);

    //    cout << "New Node " << endl;

    //    cout << "New node Biased = " << _biasing << endl;

    //    newNode->getConfiguration()->print();

    Vector3d pos;

    pos[0] = currentNode->getNodeStruct()->q[mIndexObjectDof+0];
    pos[1] = currentNode->getNodeStruct()->q[mIndexObjectDof+1];
    pos[2] = currentNode->getNodeStruct()->q[mIndexObjectDof+2];

    // TODO WARNING BROKEN
    //  Move3D::ThreeDCell* cell = _3DGrid->getCell(pos);
    //    _forward = (currentNode->equal(m_Graph->getStart()));
    //
    //    if( _forward )
    //    {
    //        if(_LastForward != cell)
    //        {
    //            if( on3DPathAndAfter( cell ) )
    //            {
    //                _LastForward = cell;
    //            }
    //        }
    //    }
    //    else
    //    {
    //        if(_LastForward != cell)
    //        {
    //            if( on3DPathAndAfter( cell ) )
    //            {
    //                _LastBackward = cell;
    //            }
    //        }
    //    }

    return newNode;
}
