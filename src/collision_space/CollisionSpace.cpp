/*
 *  CollisionSpace.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 07/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "CollisionSpace.hpp"
#include "CollisionSpaceCell.hpp"
#include "BodySurfaceSampler.hpp"

#include "planner/planEnvironment.hpp"

#include "P3d-pkg.h"
#include "Graphic-pkg.h"

#include <Eigen/Core>

CollisionSpace* global_collisionSpace = NULL;

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

CollisionSpaceCell::CollisionSpaceCell(int i, const Eigen::Vector3d& corner, API::ThreeDGrid* grid) :
    API::ThreeDCell(i, corner, grid)
{
    m_Valid = true;
    m_Visited = false;
    m_Occupied = false;

    m_ClosestPoint[0] = -1;
    m_ClosestPoint[1] = -1;
    m_ClosestPoint[2] = -1;

    _cellSize = grid->getCellSize();

    // Max radius
    m_DistanceSquare = 10000;

    m_Location = grid->getCellCoord( this );
}

void CollisionSpaceCell::draw( int color, int width )
{
    Eigen::Vector3d corner = getCorner();

    //cout << "Draw CollisionSpaceCell ( " << color << " )" << endl;
    g3d_draw_simple_box(corner[0], corner[0] + _cellSize[0],
                        corner[1], corner[1] + _cellSize[1],
                        corner[2], corner[2] + _cellSize[2], color, 0, width);
}

void CollisionSpaceCell::drawStatic()
{
    if( !m_Valid )
    {
        draw(Red, 1);
    }
}

void CollisionSpaceCell::draw()
{
    if( !m_Valid )
    {
        //drawStatic()
    }
    else if (m_Occupied)
    {
        draw(Green,1);
    }
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------

CollisionSpace::CollisionSpace( Robot* rob, double pace, const vector<double>& env_size ) :
    API::ThreeDGrid( pace , env_size ), m_sampler(NULL)
{
    //    m_nbMaxCells = ENV.getInt(Env::nbCells);

    //    double cellSize = (XYZ_ENV->box.x2 - XYZ_ENV->box.x1);

    //    cellSize = MAX(XYZ_ENV->box.y2 - XYZ_ENV->box.y1, cellSize);
    //    cellSize = MAX(XYZ_ENV->box.z2 - XYZ_ENV->box.z1, cellSize);
    //    cellSize /= m_nbMaxCells;

    //    _originCorner[0] = XYZ_ENV->box.x1;
    //    _originCorner[1] = XYZ_ENV->box.y1;
    //    _originCorner[2] = XYZ_ENV->box.z1;

    //    _nbCellsX = (XYZ_ENV->box.x2 - XYZ_ENV->box.x1)/cellSize;
    //    _nbCellsY = (XYZ_ENV->box.y2 - XYZ_ENV->box.y1)/cellSize;
    //    _nbCellsZ = (XYZ_ENV->box.z2 - XYZ_ENV->box.z1)/cellSize;

    //    _originCorner[0] = env_size[0];
    //    _originCorner[1] = env_size[2];
    //    _originCorner[2] = env_size[4];

    //    Eigen::Vector3d topCorner;
    //    topCorner[0] = env_size[1];
    //    topCorner[1] = env_size[3];
    //    topCorner[2] = env_size[4];

    //    double cellSize = ( topCorner[0] - _originCorner[0] );

    //    cellSize = max( topCorner[1] - _originCorner[1], cellSize );
    //    cellSize = max( topCorner[2] - _originCorner[2], cellSize );
    //    cellSize /= m_nbMaxCells;

    //    _nbCellsX = (XYZ_ENV->box.x2 - _originCorner[1])/cellSize;
    //    _nbCellsY = (XYZ_ENV->box.y2 - _originCorner[1])/cellSize;
    //    _nbCellsZ = (XYZ_ENV->box.z2 - _originCorner[1])/cellSize;

    //    //_nbCells = _nbCellsX * _nbCellsY * _nbCellsZ;
    //    _cellSize[0] = _cellSize[1] = _cellSize[2] = cellSize;

    this->createAllCells();

    //unvalid cells for each robot except current one ?
    m_size = max( max(  _cellSize[0]*_nbCellsX , _cellSize[1]*_nbCellsY ), _cellSize[2]*_nbCellsZ  );

    //Build the meshes env edges
    if(XYZ_ENV->o[0]->pol[0]->poly->the_edges == NULL)
    {
        for(int i = 0; i < XYZ_ENV->no; i++)
        {
            p3d_obj * obj = XYZ_ENV->o[i];

            for(int j = 0; j < obj->np; j++)
            {
                poly_build_edges(obj->pol[j]->poly);
            }
        }
    }

    //m_Robot = global_Project->getActiveScene()->getActiveRobot();
    m_Robot = rob;

    init();
}

CollisionSpace::~CollisionSpace()
{
    delete m_sampler;
}

API::ThreeDCell* CollisionSpace::createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z )
{
    return new CollisionSpaceCell( index, computeCellCorner(x,y,z) , this );
}

void CollisionSpace::init()
{
    m_sampler = new BodySurfaceSampler( _cellSize[0]*0.25 );
    m_sampler->sampleStaticObjectsSurface();
    m_sampler->sampleAllRobotsBodiesSurface();

    int maxSq = (  _nbCellsX*_nbCellsX + _nbCellsY*_nbCellsY + _nbCellsZ*_nbCellsZ ) ;

    double resolution = _cellSize[0];

    // create a sqrt table:
    m_SqrtTable.resize( maxSq + 1 );
    for (int i=0; i<=maxSq; ++i)
    {
        m_SqrtTable[i] = sqrt(double(i))*resolution;
    }

    cout << "Max m_SqrtTable : " << m_SqrtTable.back() << endl;

    m_invTwiceResolution = 1.0/(2.0*resolution);
    initNeighborhoods();
}

void CollisionSpace::resetOccupationCells()
{
    for(unsigned int i = 0; i < getNumberOfCells(); i++)
    {
        dynamic_cast<CollisionSpaceCell*>(_cells[i])->setOccupied(false);
    }
}

//! Updates the cells occupied by the robot
//! All objects composing the robot are processed
//! A list of cells for each body is added
void CollisionSpace::updateRobotOccupationCells(Robot* rob)
{
    m_OccupationCells.clear();

    for( int i=0; i<int(rob->getNumberOfJoints()); i++)
    {
        p3d_obj* obj = rob->getJoint(i)->getJointStruct()->o;

        if( obj )
        {
            vector<CollisionSpaceCell*> objectCell = getOccupiedCellsForObject( obj, rob->getJoint(i)->getMatrixPos() );

            m_OccupationCells.insert( m_OccupationCells.end(), objectCell.begin(), objectCell.end() );
        }
    }

    for( int i=0; i<int(m_OccupationCells.size()); i++)
    {
        m_OccupationCells[i]->setOccupied(true);
    }
}

//! Return the occupied cells for a given object
//! Each point on the surface of the object leads to one cell
//! the cells allready added are marked as visied
vector<CollisionSpaceCell*> CollisionSpace::getOccupiedCellsForObject( p3d_obj* obj, const Eigen::Transform3d& T )
{
    vector<CollisionSpaceCell*> objectCells;

    PointCloud& cloud = m_sampler->getPointCloud( obj );

    for( int i=0; i<int(cloud.size()); i++)
    {
        CollisionSpaceCell* cell = dynamic_cast<CollisionSpaceCell*>(getCell(Eigen::Vector3d( T * cloud[i] )));

        if( cell == NULL )
            continue;

        if(!cell->isVisited())
        {
            cell->setVisited(true);
            objectCells.push_back(cell);
        }
    }

    for( int i=0; i<int(objectCells.size()); i++)
        objectCells[i]->setVisited(false);

    return objectCells;
}

bool CollisionSpace::areCellsValid(const vector<CollisionSpaceCell*>& cells)
{
    for (unsigned int i=0; i<cells.size(); i++)
    {
        if(!cells[i]->isValid())
        {
            return false;
        }
    }

    return true;
}

bool CollisionSpace::collisionCheck()
{
    updateRobotOccupationCells( m_Robot );

    for(unsigned int i=0; i<m_OccupationCells.size(); ++i)
    {
        if(!m_OccupationCells[i]->isValid())
        {
            return true;
        }
    }

    return false;
}

void CollisionSpace::initNeighborhoods()
{
    // first initialize the direction number mapping:
    m_DirectionNumberToDirection.resize(27);
    for (int dx=-1; dx<=1; ++dx)
    {
        for (int dy=-1; dy<=1; ++dy)
        {
            for (int dz=-1; dz<=1; ++dz)
            {
                int direction_number = getDirectionNumber(dx, dy, dz);
                vector<int> n_point(3);
                n_point[0] = dx;
                n_point[1] = dy;
                n_point[2] = dz;
                m_DirectionNumberToDirection[direction_number] = n_point;
            }
        }
    }

    m_Neighborhoods.resize(2);
    for (int n=0; n<2; n++)
    {
        m_Neighborhoods[n].resize(27);
        // source directions
        for (int dx=-1; dx<=1; ++dx)
        {
            for (int dy=-1; dy<=1; ++dy)
            {
                for (int dz=-1; dz<=1; ++dz)
                {
                    int direction_number = getDirectionNumber(dx, dy, dz);
                    // target directions:
                    for (int tdx=-1; tdx<=1; ++tdx)
                    {
                        for (int tdy=-1; tdy<=1; ++tdy)
                        {
                            for (int tdz=-1; tdz<=1; ++tdz)
                            {
                                if (tdx==0 && tdy==0 && tdz==0)
                                    continue;
                                if (n>=1)
                                {
                                    if ((abs(tdx) + abs(tdy) + abs(tdz))!=1)
                                        continue;
                                    if (dx*tdx<0 || dy*tdy<0 || dz*tdz <0)
                                        continue;
                                }
                                vector<int> n_point(3);
                                n_point[0] = tdx;
                                n_point[1] = tdy;
                                n_point[2] = tdz;
                                m_Neighborhoods[n][direction_number].push_back(n_point);
                            }
                        }
                    }
                }
            }
        }
    }
}

double CollisionSpace::addPointsToField(const std::vector<Eigen::Vector3d>& points)
{
    cout << "Add points to distance field" << endl;
    cout << "Propagate distance" << endl;

    double max_distance_sq = (  _nbCellsX*_nbCellsX
                                + _nbCellsY*_nbCellsY
                                + _nbCellsZ*_nbCellsZ );

    //cout << "max_distance_sq : " << max_distance_sq << endl;

    // initialize the bucket queue
    vector< vector<CollisionSpaceCell*> > bucket_queue;
    bucket_queue.resize( max_distance_sq+1 );
    bucket_queue[0].reserve(points.size());

    // first mark all the points as distance=0, and add them to the queue
    int initial_update_direction = getDirectionNumber(0,0,0);
    for (unsigned int i=0; i<points.size(); ++i)
    {
        Eigen::Vector3d point = points[i];
        CollisionSpaceCell* voxel = dynamic_cast<CollisionSpaceCell*>(this->getCell( point ));

        if (!voxel)
            continue;

        Eigen::Vector3i coord = getCellCoord(voxel);

        voxel->m_DistanceSquare = 0;
        voxel->m_ClosestPoint = coord;
        voxel->m_UpdateDirection = initial_update_direction;
        bucket_queue[0].push_back(voxel);
    }

    // now process the queue:
    cout << "process the Queue : " << bucket_queue.size() << endl;
    for (unsigned int i=0; i<bucket_queue.size(); ++i)
    {
        std::vector<CollisionSpaceCell*>::iterator list_it = bucket_queue[i].begin();
        while(list_it!=bucket_queue[i].end())
        {
            CollisionSpaceCell* vptr = *list_it;

            // Get the cell location in grid
            Eigen::Vector3i loc = getCellCoord(vptr);

            int D = i;
            if (D>1)
                D=1;
            // avoid a possible segfault situation:
            if (vptr->m_UpdateDirection<0 || vptr->m_UpdateDirection>26)
            {
                cout << "Invalid update direction detected: " << vptr->m_UpdateDirection << endl;
                ++list_it;
                continue;
            }

            // select the neighborhood list based on the update direction:
            std::vector<std::vector<int> >* neighborhood = &m_Neighborhoods[D][vptr->m_UpdateDirection];

            // Look in the neighbouring cells
            // and update distance
            for (unsigned int n=0; n< neighborhood->size(); n++)
            {
                Eigen::Vector3i direction;
                direction[0] = (*neighborhood)[n][0];
                direction[1] = (*neighborhood)[n][1];
                direction[2] = (*neighborhood)[n][2];

                Eigen::Vector3i neigh_loc = loc + direction;

                CollisionSpaceCell* neighbour = dynamic_cast<CollisionSpaceCell*>(getCell(neigh_loc));

                if (!neighbour)
                    continue;

                //        cout << "loc = " << endl << loc << endl;
                //        cout << "vptr->m_ClosestPoint = " << endl << vptr->m_ClosestPoint << endl;

                double new_distance_sq_float = ( vptr->m_ClosestPoint - neigh_loc ).squaredNorm();

                int new_distance_sq = new_distance_sq_float;
                if (new_distance_sq > max_distance_sq)
                {
                    cout << "new_distance_sq : " << new_distance_sq << endl;
                    continue;
                }

                //        cout << "new_distance_sq_float = " << new_distance_sq_float << endl;
                //        cout << "---------------------------------------" << endl;
                if (new_distance_sq < neighbour->m_DistanceSquare)
                {
                    //         cout << "loc = " <<  loc << endl;
                    //cout << "vptr->m_ClosestPoint = " << endl << vptr->m_ClosestPoint << endl;
                    //cout << "new_distance_sq_float = " << new_distance_sq_float << endl;
                    //cout << "new_distance_sq = " << new_distance_sq << endl;
                    // update the neighboring voxel
                    neighbour->m_DistanceSquare = new_distance_sq;
                    neighbour->m_ClosestPoint = vptr->m_ClosestPoint;
                    neighbour->m_UpdateDirection = getDirectionNumber(direction[0], direction[1], direction[2]);

                    // and put it in the queue:
                    bucket_queue[new_distance_sq].push_back(neighbour);
                }
            }
            ++list_it;
        }
        bucket_queue[i].clear();
    }
    cout << "All points have been added!!!" << endl;
    return 0.0;
}

void CollisionSpace::addRobotBody(Joint* jnt)
{
    vector<Eigen::Vector3d> points;

    p3d_obj* obj = jnt->getJointStruct()->o;

    if ( obj == NULL )
        return;

    PointCloud& cloud = m_sampler->getPointCloud( obj );

    for (unsigned int j=0; j<cloud.size(); j++)
    { points.push_back( jnt->getMatrixPos()*cloud[j] ); }

    addPointsToField(points);
}

void CollisionSpace::addRobot(Robot* rob)
{
    for (unsigned int joint_id=0;
         joint_id<rob->getNumberOfJoints(); joint_id++)
    {
        addRobotBody( rob->getJoint( joint_id ) );
    }
}

void CollisionSpace::addAllPointsToField()
{
    vector<Eigen::Vector3d> points;

    // Add Static Obstacles
    for (int i=0; i<XYZ_ENV->no; i++)
    {
        PointCloud& cloud = m_sampler->getPointCloud(XYZ_ENV->o[i]);

        for (unsigned int j=0; j<cloud.size(); j++)
        { points.push_back( cloud[j] ); }
    }

    // Add Moving Obstacles
    Scene* sc = global_Project->getActiveScene();
    for (unsigned int i=0; i<sc->getNumberOfRobots(); i++)
    {
        Robot* mov_obst = sc->getRobot(i);

        // The robot is not a robot or a human
        if (  mov_obst->getName().find( "ROBOT" ) == string::npos &&
              mov_obst->getName().find( "HUMAN" ) == string::npos )
        {
            Joint* jnt = mov_obst->getJoint(1);

            if( jnt->getJointStruct()->o == NULL)
                continue;

            PointCloud& cloud = m_sampler->getPointCloud( jnt->getJointStruct()->o );

            for ( int j=0; j<int(cloud.size()); j++)
            {
                points.push_back( jnt->getMatrixPos()*cloud[j] );
            }
        }
    }

    addPointsToField(points);
}

int CollisionSpace::getDirectionNumber(int dx, int dy, int dz) const
{
    return (dx+1)*9 + (dy+1)*3 + dz+1;
}

double CollisionSpace::getDistanceFromCell(int x, int y, int z) const
{
    return getDistance( static_cast<CollisionSpaceCell*>(getCell(x,y,z)) );
}

double CollisionSpace::getDistance(CollisionSpaceCell* cell) const
{
    if ( cell->m_DistanceSquare >= (int)m_SqrtTable.size() )
    {
        //std::cout << "cell dist is : " << cell->m_DistanceSquare << std::endl;
        return 0;
    }
    return m_SqrtTable[cell->m_DistanceSquare];

    //  return sqrt(cell->m_DistanceSquare)*_cellSize[0];
}

//! Computes the finiate differenting gradient
//! 
double CollisionSpace::getDistanceGradient(const Eigen::Vector3d& point,Eigen::Vector3d& gradient) const
{
    CollisionSpaceCell* cell = static_cast<CollisionSpaceCell*>( getCell( point ) );

    if( !cell )
    {
        gradient = Eigen::Vector3d::Zero();
        return 0;
    }

    const Eigen::Vector3i& loc = cell->getLocation();

    unsigned int gx = loc(0);
    unsigned int gy = loc(1);
    unsigned int gz = loc(2);

    // if out of bounds, return 0 distance, and 0 gradient
    // we need extra padding of 1 to get gradients
    if (gx<1 || gy<1 || gz<1 ||
            gx>=getXNumberOfCells()-1 || gy>=getYNumberOfCells()-1 || gz>=getZNumberOfCells()-1)
    {
        gradient = Eigen::Vector3d::Zero();
        return 0;
    }

    gradient(0) = (getDistanceFromCell(gx+1,gy,gz) - getDistanceFromCell(gx-1,gy,gz))*m_invTwiceResolution;
    gradient(1) = (getDistanceFromCell(gx,gy+1,gz) - getDistanceFromCell(gx,gy-1,gz))*m_invTwiceResolution;
    gradient(2) = (getDistanceFromCell(gx,gy,gz+1) - getDistanceFromCell(gx,gy,gz-1))*m_invTwiceResolution;

    //return getDistanceFromCell(gx,gy,gz);
    return getDistance( cell );
}

//! The distance potential and the gradient is computed for a collision point
//! First the distance d of the collision point (sphere) to the closest obstacle is computed
//! Then 3 cases apear to compute the potential
//! - 0 if this distance is greater than the coll point clearance
//! - 
bool CollisionSpace::getCollisionPointPotentialGradient(const CollisionPoint& collision_point, 
                                                        const Eigen::Vector3d& collision_point_pos,
                                                        double& field_distance,
                                                        double& potential,
                                                        Eigen::Vector3d& gradient) const
{
    Eigen::Vector3d field_gradient;

    // Compute the distance gradient and distance to nearest obstacle
    field_distance = this->getDistanceGradient( collision_point_pos, field_gradient );

    //  field_gradient(0) += field_bias_x_;
    //  field_gradient(1) += field_bias_y_;
    //  field_gradient(2) += field_bias_z_;

    double d = field_distance - collision_point.getRadius();

    // three cases below:
    if (d >= collision_point.getClearance())
    {
        potential = 0.0;
        gradient.setZero();
    }
    else if (d >= 0.0)
    {
        double diff = (d - collision_point.getClearance());
        double gradient_magnitude = diff * collision_point.getInvClearance(); // (diff / clearance)
        potential = 0.5*gradient_magnitude*diff;
        gradient = gradient_magnitude * field_gradient;
    }
    else // if d < 0.0
    {
        gradient = field_gradient;
        potential = -d + 0.5 * collision_point.getClearance();
    }

    //  cout << "----------------------------" << endl;
    //  cout << "field_distance = " << field_distance << endl;
    //  cout << "collision_point.getRadius() = " << collision_point.getRadius() << endl;

    return (field_distance <= collision_point.getRadius()); // true if point is in collision
}


//! Goes through all points and computes whether the robot is colliding
//! Computes the transform of all point from the joint transform
//! Uses the potential gradient function
bool CollisionSpace::isRobotColliding(double& dist) const
{
    //cout << "Test collision space is robot colliding" << endl;
    bool isRobotColliding = false;

    double potential = 0.0;
    double max_potential = -numeric_limits<double>::max();

    double distance = 0.0 ;
    double min_distance = numeric_limits<double>::max();

    Eigen::Vector3d position,gradient;

    for( unsigned int i=0; i<m_Robot->getNumberOfJoints(); i++ )
    {
        Joint* jnt = m_Robot->getJoint(i);

        Eigen::Transform3d T = jnt->getMatrixPos();
        vector<CollisionPoint>& points = m_sampler->getCollisionPoints(jnt);

        //cout << "Joint(" << i << "), nbCollisionPoints : " << points.size() << endl;

        for( unsigned int j=0; j<points.size(); j++ )
        {
            position = T*points[j].getPosition();

            if( getCollisionPointPotentialGradient( points[j], position, distance, potential, gradient ) )
            {
                //        if ( j > 5 )
                //        {
                points[j].m_is_colliding = true;
                //        cout << "point : " << j << " with segment number : " << points[j].getSegmentNumber() ;
                //        cout << " in collision" << endl;
                //        }
                //        else {
                //          points[j].m_is_colliding = false;
                //        }

                // Hack!!!
                if ( points[j].getSegmentNumber() > 1 )
                {
                    isRobotColliding = true;
                }
            }
            else {
                points[j].m_is_colliding = false;
            }

            if( max_potential < potential )
                max_potential = potential;

            if( min_distance > distance )
                min_distance = distance;

        }
    }

    dist = min_distance;
    return isRobotColliding;
}

//! Draws a sphere in all voxels
//! the color of the sphere represent the distance 
//! to the nearest obstacle
void CollisionSpace::drawStaticVoxels()
{
    // Draw all cells
    for(unsigned int i=0; i < getNumberOfCells(); i++)
    {
        static_cast<CollisionSpaceCell*>( _cells[i] )->drawStatic();
    }
}

void CollisionSpace::drawGradient()
{
    //  Eigen::Vector3d unitX(1, 0, 0);
    //  Eigen::Vector3d unitY(0, 1, 0);
    //  Eigen::Vector3d unitZ(0, 0, 1);

    for (unsigned int n=0; n<getNumberOfCells(); n++)
    {
        CollisionSpaceCell* cell = static_cast<CollisionSpaceCell*>(_cells[n]);

        if (getDistance(cell) > PlanEnv->getDouble(PlanParam::distMinToDraw) )
            continue;

        Eigen::Vector3d point = cell->getCenter();
        Eigen::Vector3d gradient;

        bool isGradientValid = getDistanceGradient( point, gradient );
        if (!isGradientValid)
            continue;

        if (gradient != Eigen::Vector3d::Zero() ) {
            //cout << "gradient = " << endl << gradient << endl;
        }

        /**
     Eigen::Vector3d axis = //gradient.cross(unitX).length() > 0 ?
     gradient.cross(Eigen::Vector3d::UnitX()); // : unitY;
     double angle = -gradient.dot(Eigen::Vector3d::UnitX());
     
     Eigen::Transform3d t = Eigen::AngleAxisd(angle,axis) * Eigen::Translation3d(point);
     
     p3d_matrix4 mat;
     for (unsigned int i=0; i<4; i++) {
     for (unsigned int j=0;j<4; j++) {
     mat[i][j] =  t(i,j);
     //cout << t.matrix() << endl;
     }
     }
     g3d_draw_frame(mat, 0.10);
     */

        p3d_vector3 p1;
        Eigen::Vector3d p1Eigen = cell->getCenter();

        p1[0] = p1Eigen(0);
        p1[1] = p1Eigen(1);
        p1[2] = p1Eigen(2);

        p3d_vector3 p2;
        Eigen::Vector3d p2Eigen = 2*cell->getCellSize().minCoeff()*gradient + p1Eigen;

        p2[0] = p2Eigen(0);
        p2[1] = p2Eigen(1);
        p2[2] = p2Eigen(2);

        g3d_draw_arrow(p1,p2,0.1,0.7,0.0);
    }
}

void CollisionSpace::drawSquaredDist()
{
    double distToClosObst;
    CollisionSpaceCell* cell;
    for(unsigned int i=0; i < getNumberOfCells(); i++)
    {
        cell = static_cast<CollisionSpaceCell*>( _cells[i] );

        distToClosObst = getDistance(cell);

        //cout << "distToClosObst : " << distToClosObst << endl;

        if (distToClosObst < PlanEnv->getDouble(PlanParam::distMinToDraw) )
            cell->drawColorGradient(distToClosObst,0.0,1.0,true);
    }
}

void CollisionSpace::drawCollisionPoints()
{
    for( unsigned int i=0; i<m_Robot->getNumberOfJoints(); i++ )
    {
        Joint* jnt = m_Robot->getJoint(i);
        Eigen::Transform3d T = jnt->getMatrixPos();
        vector<CollisionPoint>& points = m_sampler->getCollisionPoints(jnt);

        for( unsigned int j=0; j<points.size(); j++ )
        {
            //      if (points[j].getSegmentNumber() < ENV.getDouble(Env::extensionStep))
            //        continue;

            if( points[j].m_is_colliding )
            {
                double color[4];

                color[1] = 0.0;       //green
                color[2] = 0.0;       //blue
                color[0] = 1.0;       //red
                color[3] = 0.7;       //transparency

                g3d_set_color(Any,color);
            }

            bool yellow = (!points[j].m_is_colliding);

            points[j].draw( T, yellow );
        }
    }
}

void CollisionSpace::draw()
{
    //cout << "Draw CollisionSpace" << endl;
    resetOccupationCells();

    updateRobotOccupationCells( m_Robot );

    // For all robot in the scene
    //  Scene* sc = global_Project->getActiveScene();
    //  for (unsigned int i=0; i< sc->getNumberOfRobots(); i++)
    //  {
    //    updateRobotOccupationCells( sc->getRobot(i) );
    //  }

    //  updateRobotOccupationCells( sc->getRobotByNameContaining("LOTR") );

    // Draw all cells
    for(unsigned int i=0; i<_cells.size(); i++)
    {
        static_cast<CollisionSpaceCell*>( _cells[i] )->draw();
    }
}