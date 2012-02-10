#ifndef PLANNARTRAJECTORYSMOOTHING_HPP
#define PLANNARTRAJECTORYSMOOTHING_HPP


#include "API/planningAPI.hpp"


using namespace std;
using namespace tr1;
using namespace Eigen;
#include <Eigen/StdVector>

class PlannarTrajectorySmoothing
{
public:
    PlannarTrajectorySmoothing(Robot* robot);


    void initTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, Robot* Cylinder);
    bool goToNextStep();


    // functions
    double computeDistFromTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, int begin, int end);

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

    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > removeSamePoints(
            std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > traj, double epsilon);

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > get2DtrajFrom3Dtraj(
            std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > traj);

private:
    int _id;
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > _traj;
    Robot* _cyl;
    Robot* _Robot;
    shared_ptr<Configuration>  _curRobotConf;
    shared_ptr<Configuration>  _curCylConf;
    double _dist;


};

#endif // PLANNARTRAJECTORYSMOOTHING_HPP
