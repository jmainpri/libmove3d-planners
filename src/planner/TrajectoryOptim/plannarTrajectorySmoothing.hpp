#ifndef PLANNARTRAJECTORYSMOOTHING_HPP
#define PLANNARTRAJECTORYSMOOTHING_HPP

#include "API/ConfigSpace/configuration.hpp"
MOVE3D_USING_SHARED_PTR_NAMESPACE

#include <Eigen/StdVector>

namespace Move3D {

class PlannarTrajectorySmoothing
{
public:
    PlannarTrajectorySmoothing( Move3D::Robot* robot );


    void initTraj( std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, Move3D::Robot* Cylinder );
    bool goToNextStep();


    // functions
    double computeDistOfTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, int begin, int end);

    double computeDistBetweenTrajAndPoint(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, Eigen::Vector2d p);

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > removeUnnecessaryPoint(
            std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > trajectory, double threshold);

    bool robotCanDoTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj,
                                                    Robot* BoundingBox, Robot* trajOfThisRobot, double dist);

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > smoothTrajectory(Robot* robot,
            std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > trajectory);

    Eigen::Vector2d getRandomPointInSegment(Eigen::Vector2d p1, Eigen::Vector2d p2, double errorT);

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > findShortCut(
            std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, int i, int j);


    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > findShortCut(
            std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, int i, Robot* cyl, Robot* robot);

    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > add3Dim(
            std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, Robot* robot, double epsilon);

    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > add3DimwithoutTrajChange(
            std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, Robot* robot, double epsilon);

    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > removeSamePoints(
            std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > traj, double epsilon);

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > get2DtrajFrom3Dtraj(
            std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > traj);



private:
    int _id;
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > _traj;
    Move3D::Robot* _cyl;
    Move3D::Robot* _Robot;
    shared_ptr<Configuration>  _curRobotConf;
    shared_ptr<Configuration>  _curCylConf;
    double _dist;


};

}

#endif // PLANNARTRAJECTORYSMOOTHING_HPP
