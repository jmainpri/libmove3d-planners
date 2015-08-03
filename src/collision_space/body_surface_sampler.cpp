/*
 *  BodySurfaceSampler.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 23/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "body_surface_sampler.hpp"

#include "API/project.hpp"
#include "planner/planEnvironment.hpp"

#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"

using namespace std;
using namespace Move3D;

BodySurfaceSampler::BodySurfaceSampler(double step, double clearance) :
    m_step(step),
    m_collision_clearance_default(clearance)
{

}

BodySurfaceSampler::~BodySurfaceSampler()
{
    //    cout << "Delete BodySurfaceSampler" << endl;
}

bool BodySurfaceSampler::isPointOverGround( const Eigen::Vector3d& point )
{
    if ( point[2] < 0 ) {
        return false;
    }

    return true;
}

bool BodySurfaceSampler::isPointInEnvironment( const Eigen::Vector3d& point )
{
    vector<double> bounds = global_Project->getActiveScene()->getBounds();

    if ( (point[0] < bounds[0]) || (point[0] > bounds[1]) ) {
        return false;
    }
    if ( (point[1] < bounds[2]) || (point[1] > bounds[3]) ) {
        return false;
    }
    if ( (point[2] < bounds[4]) || (point[2] > bounds[5]) ) {
        return false;
    }

    return true;
}

//! Sample the surface of an object
//! 
//! Discard points if they are not in the environment box
//! also dicard points if they are bellow 0 in z (bellow ground)
//! this enables a pading while keeping a ground object for the
//! calssical collision checker based algorithm
PointCloud& BodySurfaceSampler::sampleObjectSurface( p3d_obj* obj, bool isRobot )
{
    //  cout << "Sample Object : " << obj->name << endl;
    m_objectToPointCloudMap[obj] = PointCloud( 0.01 );

    for(int i = 0; i<obj->np; i++)
    {
        if(obj->pol[i]->TYPE != P3D_GRAPHIC)
        {
            p3d_polyhedre* poly = obj->pol[i]->poly;
            p3d_vector3 the_points[poly->nb_points];

            for(unsigned int j = 0; j < poly->nb_points; j++)
            {
                the_points[j][0] = poly->the_points[j][0];
                the_points[j][1] = poly->the_points[j][1];
                the_points[j][2] = poly->the_points[j][2];

                if (obj->type == P3D_OBSTACLE)
                {
                    //real point position
                    p3d_xformPoint(obj->pol[i]->pos0, poly->the_points[j], the_points[j]);
                }
                else
                {
                    p3d_matrix4 inv_pos, mat;
                    p3d_matInvertXform( obj->jnt->pos0, inv_pos );
                    p3d_matMultXform(inv_pos, obj->pol[i]->pos0, mat);
                    p3d_xformPoint(mat, poly->the_points[j], the_points[j]);
                }
            }

            for(unsigned int j = 0; j < poly->nb_faces; j++)
            {
                unsigned int nbPoints = 0;

                p3d_vector3* tmp = sample_triangle_surface(
                            the_points[poly->the_faces[j].the_indexs_points[0] - 1],
                            the_points[poly->the_faces[j].the_indexs_points[1] - 1],
                            the_points[poly->the_faces[j].the_indexs_points[2] - 1], m_step, &nbPoints);

                for(unsigned int k = 0; k < nbPoints; k++)
                {
                    Eigen::Vector3d point;
                    point(0) = tmp[k][0];
                    point(1) = tmp[k][1];
                    point(2) = tmp[k][2];

                    if( isRobot || ( isPointInEnvironment( point ) && isPointOverGround( point )) )
                    {
                        m_objectToPointCloudMap[obj].push_back( point );
                    }
                }
                free(tmp);
            }
        }
    }

    return m_objectToPointCloudMap[obj];
}

//! Returns a pointer to the point cloud associated to the given object
//!
PointCloud& BodySurfaceSampler::getPointCloud( obj* o )
{
    return m_objectToPointCloudMap[o];
}

//! Sample the
//!
void BodySurfaceSampler::sampleStaticObjectsSurface()
{
    // PointCloud points;
    for(int i = 0; i < XYZ_ENV->no; i++)
    {
        sampleObjectSurface( XYZ_ENV->o[i] );
    }
}

//! Compute point cloud for the static objects
//!
void BodySurfaceSampler::sampleRobotBodiesSurface( Robot* robot )
{
    // PointCloud points;
    for( int i=0; i<int(robot->getNumberOfJoints()); i++)
    {
        p3d_obj* obj = static_cast<p3d_jnt*>( robot->getJoint(i)->getP3dJointStruct() )->o;

        if( obj )
        {
            sampleObjectSurface( obj, true );
        }
    }
}

void BodySurfaceSampler::sampleAllRobotsBodiesSurface()
{
    Scene* sc = global_Project->getActiveScene();
    
    for( int i=0;i<int(sc->getNumberOfRobots()); i++)
    {
        sampleRobotBodiesSurface( sc->getRobot(i) );
    }
}

BoundingCylinder* BodySurfaceSampler::generateBoudingCylinder(p3d_obj* obj)
{
    if(!obj)
    {
        cout << "Problem object is NILL" << endl;
        return NULL;
    }

    double box[8][3];
    std::vector<Eigen::Vector3d> cuboide(8);

    double radius;
    Eigen::Vector3d p1,p2;

    if ( pqp_get_OBB_first_level(obj,box) )
    {
        for (unsigned int i=0; i<8; i++)
            for(unsigned int j=0; j<3; j++)
                cuboide[i][j] = box[i][j];

        p1 = 0.5*(cuboide[1]+cuboide[2]);
        p2 = 0.5*(cuboide[5]+cuboide[6]);

        radius = 0.5*( ( cuboide[1] - cuboide[2] ).norm() );
    }
    else
    {
        //cout << "No Bounding Box" << endl;
        return NULL;
    }

    return new BoundingCylinder( p1, p2, radius );
}

double BodySurfaceSampler::generateRobotBoudingCylinder( Robot* robot, const vector<Joint*>& activeJoints )
{
    double maxRadius= numeric_limits<double>::min();

    for(unsigned int i=0;i<activeJoints.size();i++)
    {
        p3d_obj* obj = static_cast<p3d_jnt*>( activeJoints[i]->getP3dJointStruct() )->o;

        if(obj)
        {
            BoundingCylinder* bc = generateBoudingCylinder(obj);

            if (bc)
            {
                m_objectToBoCylinderMap[obj] = bc;
                if ( maxRadius < bc->getRadius() )
                {
                    maxRadius = bc->getRadius();
                }
            }
        }
    }

    return maxRadius;
}


std::vector<CollisionPoint>& BodySurfaceSampler::getCollisionPoints(Joint* jnt)
{
    return m_jointToCollisionPoint[jnt];
}

//! Generates the collision point for a given link
//! Stores the segment number (the id of the joint for planning)
//! Also store the parent joints of the link
std::vector<CollisionPoint> BodySurfaceSampler::getLinksCollisionPoints(Joint* jnt, int segment_number, const std::vector<int>& parent_joints )
{
    std::vector<CollisionPoint> collision_points;

    if (m_objectToBoCylinderMap.empty())
        return collision_points;

    BoundingCylinder* bc = m_objectToBoCylinderMap[ static_cast<p3d_jnt*>( jnt->getP3dJointStruct() )->o];

    if (bc == NULL)
        return collision_points;

    double radius = bc->getRadius();
    double length = bc->getLength();

    Eigen::Vector3d p1 = bc->getPoint1();
    Eigen::Vector3d p2 = bc->getPoint2();

    Eigen::Vector3d p;

    double spacing = radius / PlanEnv->getDouble(PlanParam::ratioCollRadiusSpacing); // 2.0
    int num_points = ceil( length / spacing ) + 1;
    spacing = length / ( num_points - 1.0 );

    // cout << "segment id : " << segment_number << " , nb of coll points : " << num_points << endl;

    for (int i=0; i<num_points; ++i)
    {
        Eigen::Vector3d p = p1 + (double(i)/double(num_points))*( p2 - p1 );
        // cout << "p : " << p.transpose() << endl;
        cout << "clearance : " << m_collision_clearance_default << endl;
        collision_points.push_back(CollisionPoint(
                                       parent_joints, radius,
                                       m_collision_clearance_default, segment_number, p));
    }


    return collision_points;
}

//! Computes collision points for all bodies
void BodySurfaceSampler::generateAllRobotsBoundingCylinders()
{
    Scene* sc = global_Project->getActiveScene();
    Robot* rob;

    for(unsigned int i=0;i<sc->getNumberOfRobots(); i++)
    {
        rob = sc->getRobot(i);
        generateRobotBoudingCylinder( rob , rob->getAllJoints() );
    }
}

//! Computes the parent joints, the segment of the joint
//! and generate the collision point for the joint
//! supposing that the bounding cylinder has been computed before
std::vector<CollisionPoint> BodySurfaceSampler::generateJointCollisionPoints(Robot* robot, int id, 
                                                                             const std::vector<int>& active_joints,
                                                                             const std::vector<int>& planner_joints)
{
    const int joint = active_joints[id];
    Joint* jnt = robot->getJoint( joint );

    std::vector<int> parent_joints;
    parent_joints.clear();

    // Compute the parent joints of joint,
    // this doesn't handle trees, only handles chains
    for (unsigned int j=0; j<active_joints.size(); j++)
    {
        if ( active_joints[j] <= joint )
        {
            parent_joints.push_back( active_joints[j] );
            // cout << "parent joint : " << parent_joints[j] << endl;
        }
    }

    int segment=-1; // The segment is the index of the joint in the planner

    // TODO remove this is the copy of what happens at the init of
    // Chomp planning group it sets the index in the planning group
    // here it counts the number of DoFs for each planned joint to retreive the index
    for (unsigned int i=0; i<planner_joints.size(); i++)
    {
        for (unsigned int j=0; j<robot->getJoint( planner_joints[i] )->getNumberOfDof(); j++)
        {
            if( !robot->getJoint( planner_joints[i] )->isJointDofUser(j) )
                continue;

            double min,max;
            robot->getJoint( planner_joints[i] )->getDofBounds( j, min, max );
            if (min == max)
                continue;

            segment++; // segment starts at 0
        }

        if( joint == planner_joints[i] )
            break;
    }

    // Becarefull!!! the segment maybe wrong if
    // the size of the active joints set is greater than the planner joint set
//    if ( id > int(planner_joints.size()-1) )
//    {
//        cout << "segment : " << id << " is replaced by : " <<  planner_joints.size()-1 ;
//        cout << " in " << __PRETTY_FUNCTION__ << endl;
//        segment = planner_joints.size()-1;
//    }
//    else
//    {
//        segment = id;
//    }

//    cout << "segment : " << segment << endl;

    // Generate the collision points of the joint link
    std::vector<CollisionPoint> points = getLinksCollisionPoints( jnt, segment, parent_joints );

    // Stores the collision point in a map of the corresponding joint
    m_jointToCollisionPoint[jnt] = points;

    return points;
}

//! Generate the collision points for links associated to the active joints
//! It computes the parent joints of the joint
std::vector<CollisionPoint> BodySurfaceSampler::generateAllRobotCollisionPoints(Robot* robot)
{
    std::vector<CollisionPoint> all_points;
    all_points.clear();

    std::vector<int> active_joints, planner_joints;

    for (int i=0; i<int(robot->getNumberOfJoints()); i++)
    {
        active_joints.push_back( i );
        planner_joints.push_back( i );
    }

    for (int id=0; id<int(robot->getNumberOfJoints()); id++)
    {
        std::vector<CollisionPoint> points =
                generateJointCollisionPoints( robot, id, active_joints, planner_joints );

        for (int i=0; i<int(points.size()); i++)
        {
            all_points.push_back( points[i] );
        }
    }

    return all_points;
}

//! Generate the collision points for links associated to the active joints
//! It computes the parent joints of the joint
std::vector<CollisionPoint> BodySurfaceSampler::generateRobotCollisionPoints(Robot* robot, 
                                                                             const std::vector<int>& active_joints,                                                                             const std::vector<int>& planner_joints,                                                                             int id_first_active_joint )
{
    std::vector<CollisionPoint> all_points;
    all_points.clear();

    // If only one joint is active
    if(active_joints.size() == 1)
    {
        std::vector<CollisionPoint> points =
                generateJointCollisionPoints( robot, 0, active_joints, planner_joints );

        for (int i=0; i<int(points.size()); i++)
        {
            all_points.push_back( points[i] );
        }
    }

    // Else only do not account for the first joint
    for ( int id=id_first_active_joint; id<int(active_joints.size()); id++ )
    {
        std::vector<CollisionPoint> points =
                generateJointCollisionPoints( robot, id, active_joints, planner_joints );

        for (int i=0; i<int(points.size()); i++)
        {
            all_points.push_back( points[i] );
        }
    }

    return all_points;
}

//! Draw the sampled points
//! Sampled points are points on the surface of static obstacles
//! collision points are computed by bounding cylinders
void BodySurfaceSampler::draw()
{
    for(int i = 0; i < XYZ_ENV->no; i++)
    {
        m_objectToPointCloudMap[XYZ_ENV->o[i]].drawAllPoints();
    }

    Joint* jnt;
    Robot* rob;
    Scene* sc = global_Project->getActiveScene();

    for(unsigned int j=0; j<sc->getNumberOfRobots(); j++)
    {
        rob = sc->getRobot(j);

        for(unsigned int i=0; i<rob->getNumberOfJoints(); i++)
        {
            jnt = rob->getJoint(i);

            Eigen::Transform3d T = jnt->getMatrixPos();
            //      vector<CollisionPoint> points = m_jointToCollisionPoint[jnt];
            //
            //      for( unsigned int i=0; i<points.size(); i++ )
            //      {
            //        points[i].draw(t);
            //      }

            p3d_obj* obj = static_cast<p3d_jnt*>( jnt->getP3dJointStruct() )->o;

            if( obj )
            {
                m_objectToPointCloudMap[obj].drawAllPoints( T, NULL );

                if (rob->getName().find("PR2") == string::npos )
                    continue;

                BoundingCylinder* bc = m_objectToBoCylinderMap[obj];

                if (bc)
                {
                    bc->draw(T);
                }
            }
        }
    }
}
