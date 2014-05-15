#ifndef HRIDISTANCE_H
#define HRIDISTANCE_H

/*
 *  Distance.h
 *
 *
 *  Created by Jim Mainprice on 05/12/09.
 *  Copyright 2009 mainprice@gmail.com All rights reserved.
 *
 */

class p3d_poly;

/**
 @ingroup HRICS
 */
namespace HRICS
{
class Distance
{
public:
    //! Constructor that sets the Human and Robot structures
    Distance( Move3D::Robot* rob, std::vector<Move3D::Robot*> humans);
    ~Distance();

    //! Goes through the Human (p3d_rob) structure
    //! to find the safety zones
    void parseHumans();

    //! Changes dynamically the size of the zone shown
    //! in the OpenGl display
    void offSetPrim(p3d_poly* poly,double offset);


    //! Changes the collision context
    //! to compute distance only to the
    //! pritimives that represent the human
    void activateSafetyZonesMode();

    //! Changes the collision context
    //! to compute the CD with the finer representation of the Human
    void activateNormalMode();

    //! Compute the distance cost
    double computeCost(double distance);

    //! Get Workspace Cost using bounding balls
    double getWorkspaceCost( const Eigen::Vector3d& WSPoint );

    //! main function (deprecated)
    std::vector<double> getDistToZones();

    //! Returns vector for drawing
    std::vector<double> getVectorJim() {return vect_jim; }
    void setVector( std::vector<double> toDrawVector ) { vect_jim = toDrawVector; }

    //! Returnd Bounding Box Distance
    double computeBBDist(p3d_vector3 robot, p3d_vector3 human);

    //! Computes the cylinder associated
    //! with the human (body)
    bool computeCylinderZone(Eigen::Vector3d &p1, Eigen::Vector3d& p2);

    //! Point to line segment distance
    double pointToLineSegmentDistance(const Eigen::Vector3d& p,
                                      const Eigen::Vector3d& p1,
                                      const Eigen::Vector3d& p2,
                                      Eigen::Vector3d& closestPoint);

    //! Bounding Balls Distance
    //! computes the distance to a line and a sphere
    //! The Line is the body and the sphere is the head
    double computeBoundingBalls(const Eigen::Vector3d& WSPoint, p3d_vector3 robot, p3d_vector3 human);

    Eigen::Vector3d getColsestPointToHuman() { return mClosestPointToHuman; }
    void setSafeRadius(double radius) { _SafeRadius = radius; }
    
    //! Draws the interaction zone using OpenGL
    void drawInteractionZone() const;
    
    int isPointInInteractionZone(const Eigen::Vector3d& WSPoint);


private:
    Move3D::Robot* m_Robot;
    std::vector<Move3D::Robot*> _Humans;
    std::vector< std::vector<int> > _SafetyZonesBodyId;
    std::vector<double> _PenetrationDist;
    std::vector<double> vect_jim;
    double _SafeOffset;
    double _SafeRadius;
    double m_InteractionRadius;

    enum Computation
    {
        Balls,
        Boxes,
        Full,
    } m_Method;

    Eigen::Vector3d mClosestPointToHuman;
};
}

#endif
