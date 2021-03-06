/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#include "HRICS_rrtPlanExpansion.hpp"
#include "../HRICS_config_space.hpp"

#include "API/Grids/PointCloud.hpp"
#include "API/Roadmap/graph.hpp"

#include "Planner-pkg.h"

using namespace std;
using namespace HRICS;
using namespace Eigen;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

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
    confPtr_t qInit = m_Graph->getRobot()->getInitPos();

    //    _Box = new double[6];
    //
    //    _Box[0] = box[0] + qInit->getConfigStruct()[6];
    //    _Box[1] = box[1] + qInit->getConfigStruct()[6];
    //    _Box[2] = box[2] + qInit->getConfigStruct()[7];
    //    _Box[3] = box[3] + qInit->getConfigStruct()[7];
    //    _Box[4] = box[4];
    //    _Box[5] = box[5];

    //    mIndexObjectDof = m_Graph->getRobot()->getObjectDof();

}

/**
  * Sets the Cell path, First and Last Cell
  */
void HRICS_rrtPlanExpansion::setCellPath(vector<Move3D::TwoDCell*> cellPath)
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
confPtr_t HRICS_rrtPlanExpansion::getExpansionDirection(
        Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode)
{
    confPtr_t q;

    mBiasing = ENV.getBool(Env::isGoalBiased) && p3d_random(0.,1.) <= ENV.getDouble(Env::Bias);

    if( mBiasing )
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
        randomPoint[0] = q->at(6);
        randomPoint[1] = q->at(7);
        randomPoint[2] = -0.40;
        PointsToDraw->push_back(randomPoint);
    }

    //    cout << ++Direction << " New Direction (Biased = " << biasing << ") " << endl;
    return q;
}

confPtr_t HRICS_rrtPlanExpansion::getConfigurationInNextCell(Node* CompcoNode)
{
    Move3D::TwoDCell* farthestCell=NULL;

    // Get the farthest cell explored depending on
    // the way the tree explores
    // WARNING BROKEN
    //    if( CompcoNode->equalCompco( m_Graph->getStart() ) )
    //    {
    //        mForward = true;
    //        farthestCell = mLastForward;
    //    }
    //    else
    //    {
    //        mForward = false;
    //        farthestCell = mLastBackward;
    //    }

    int cellId=0;
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

    //    confPtr_t q = m_Graph->getRobot()->shoot(false);

    // Get a random config in the cell
    //    randomPoint = _2DCellPath[cellId]->getRandomPoint();
    mBiasedPlanCell = m2DCellPath[cellId];

    confPtr_t q(new Configuration(m_Graph->getRobot()));

    //    Vector2d corner = mBiasedPlanCell->getCorner();
    //    Vector2d cellSize = mBiasedPlanCell->getCellSize();
    //    double biasedBox[4];
    //    biasedBox[0] = corner[0];
    //    biasedBox[1] = corner[0] + cellSize[0];
    //    biasedBox[2] = corner[1];
    //    biasedBox[3] = corner[1] + cellSize[1];
    //    p3d_ShootInCell( m_Graph->getRobot()->getP3dRobotStruct() , q->getConfigStruct() , biasedBox , 0 );

    Vector2d center = mBiasedPlanCell->getCenter();
    double CellCenter[2];
    CellCenter[0] = center[0];
    CellCenter[1] = center[1];

#ifdef P3D_PLANNER
    p3d_ShootAroundPoint( m_Graph->getRobot()->getP3dRobotStruct() , q->getConfigStruct(), CellCenter , 0);
#else
    printf("P3D_PLANNER not compiled in %s in %s",__PRETTY_FUNCTION__,__FILE__);
#endif

    return q;
}

/**
  * Return true if the cell is on the path
  * and after the first cell depending on the order (forward or backwards)
  */
bool HRICS_rrtPlanExpansion::on2DPathAndAfter(Move3D::TwoDCell* cell)
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

    // TODO WARNING BROKEN
    //    Move3D::TwoDCell* cell = m2DGrid->getCell(pos);
    //
    //    if( currentNode->equalCompco( m_Graph->getStart() ) )
    //    {
    //        if(mLastForward != cell)
    //        {
    //            if( on2DPathAndAfter( cell ) )
    //            {
    //                mLastForward = cell;
    //            }
    //        }
    //        mForward = true;
    //    }
    //    else
    //    {
    //        if(mLastBackward != cell)
    //        {
    //            if( on2DPathAndAfter( cell ) )
    //            {
    //                mLastBackward = cell;
    //            }
    //        }
    //        mForward = false;
    //    }

    return newNode;
}
