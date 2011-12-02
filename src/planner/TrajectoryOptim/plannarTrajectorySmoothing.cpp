#include "plannarTrajectorySmoothing.hpp"
#include "P3d-pkg.h"

PlannarTrajectorySmoothing::PlannarTrajectorySmoothing(Robot* robot)
{
    _id = 0;
    _Robot = robot;
    _dist = 0;
}

void PlannarTrajectorySmoothing::initTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, Robot* Cylinder)
{
    _traj = traj;
    _id = 0;
    _cyl = Cylinder;
}

bool PlannarTrajectorySmoothing::goToNextStep()
{
    if (_id == 0)
    {
        shared_ptr<Configuration> q_cur_robot (_Robot->getCurrentPos());
        _curRobotConf = q_cur_robot;

        shared_ptr<Configuration> q_robot (_Robot->getCurrentPos());
        (*q_robot)[6] = 0;
        (*q_robot)[7] = 0;
        _Robot->setAndUpdate(*q_robot);

        shared_ptr<Configuration> q_cur (_cyl->getCurrentPos());
        _curCylConf = q_cur;
    }

    shared_ptr<Configuration> q (_cyl->getCurrentPos());


    if (_id >= (int)_traj.size() -1)
    {
        _Robot->setAndUpdate(*_curRobotConf);
        _cyl->setAndUpdate(*_curCylConf);
        return false;
    }


    double x1 = _traj.at(_id)[0];
    double y1 = _traj.at(_id)[1];
    double x2 = _traj.at(_id+1)[0];
    double y2 = _traj.at(_id+1)[1];

    double angle = atan2(y2-y1,x2-x1);
    double segment = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

    double xr = x1 + cos(angle)*_dist;
    double yr = y1 + sin(angle)*_dist;
    (*q)[6] = xr;
    (*q)[7] = yr;
    _cyl->setAndUpdate(*q);

    _dist += 0.2;
    if (_dist > segment)
    {
        _dist = 0;
        _id++;
    }

    return true;
}

double PlannarTrajectorySmoothing::computeDistFromTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, int begin, int end)
{

    if (end < (int)traj.size() && begin < (int)traj.size() && begin < end)
    {
        double dist=0;
        for (int i = begin; i < end;i++)
        {
            dist += sqrt(pow(traj.at(i)[1]-traj.at(i+1)[1],2) + pow(traj.at(i)[0]-traj.at(i+1)[0],2));
        }
        return dist;
    }
    return -1;
}


std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > PlannarTrajectorySmoothing::removeUnnecessaryPoint(
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > trajectory, double threshold)
{
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > tmp;


    tmp.clear();
    for(unsigned int i =0; i < trajectory.size() - 1; i++)
    {
        if (i>0)
        {
//            double threshold = 0.03;
            if (trajectory.at(i+1)[0] - trajectory.at(i-1)[0] != 0)
            {
                double pente =  (trajectory.at(i+1)[1] - trajectory.at(i-1)[1])/(trajectory.at(i+1)[0] - trajectory.at(i-1)[0]);
                double cst = trajectory.at(i+1)[1] - pente*trajectory.at(i+1)[0];

                if (fabs(trajectory.at(i)[1] - pente*trajectory.at(i)[0] - cst) > threshold)
                {
                    tmp.push_back(trajectory.at(i));
//                    cout << "cell to be keeped \n" << trajectory.at(i) << endl;
                }
            }
            else
            {
                if (trajectory.at(i+1)[0] - trajectory.at(i)[0] > threshold)
                {
                    tmp.push_back(trajectory.at(i));
//                    cout << "cell to be keeped \n" << trajectory.at(i) << endl;
                }
            }
        }
        else
        {
            tmp.push_back(trajectory.at(i));
//            cout << "cell to be keeped \n" << trajectory.at(i) << endl;
        }
    }
    tmp.push_back(trajectory.at(trajectory.size()-1));
    return tmp;
}

bool PlannarTrajectorySmoothing::robotCanDoTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj,
                                                Robot* BoundingBox, Robot* trajOfThisRobot, double dist)
{
    shared_ptr<Configuration> q_cur_robot (trajOfThisRobot->getCurrentPos());
    shared_ptr<Configuration> q_robot (trajOfThisRobot->getCurrentPos());
    (*q_robot)[6] = 0;
    (*q_robot)[7] = 0;
    trajOfThisRobot->setAndUpdate(*q_robot);

    shared_ptr<Configuration> q_cur (BoundingBox->getCurrentPos());
    shared_ptr<Configuration> q (BoundingBox->getCurrentPos());

    for (unsigned int i = 0; i < traj.size() -1; i++)\
    {
        double x1 = traj.at(i)[0];
        double y1 = traj.at(i)[1];
        double x2 = traj.at(i+1)[0];
        double y2 = traj.at(i+1)[1];

        double angle = atan2(y2-y1,x2-x1);
        double segment = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

        for(double d = 0; d < segment; d += dist)
        {
            double xr = x1 + cos(angle)*d;
            double yr = y1 + sin(angle)*d;
            (*q)[6] = xr;
            (*q)[7] = yr;
            BoundingBox->setAndUpdate(*q);
            if (BoundingBox->isInCollisionWithOthersAndEnv())
            {
                BoundingBox->setAndUpdate(*q_cur);
                trajOfThisRobot->setAndUpdate(*q_cur_robot);
                return false;
            }
        }
    }

    trajOfThisRobot->setAndUpdate(*q_cur_robot);
    BoundingBox->setAndUpdate(*q_cur);
    return true;


}


std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > PlannarTrajectorySmoothing::smoothTrajectory(Robot* robot,
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > trajectory)
{

    Robot* robCyl;
    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);
        if(name.find("PR_2CYLINDER") != string::npos )
        {
            robCyl = new Robot(XYZ_ENV->robot[i]);
            break;
        }
    }



    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > result;

    result.clear();
    result =removeUnnecessaryPoint(trajectory,0.03);

    for (int m = 0; m< 100; m++)
    {
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > tmp;

        tmp = removeUnnecessaryPoint(result,0.03);
        if (robotCanDoTraj(tmp,robCyl,robot,0.2))
        {
            result = tmp;
        }


        int limitUp = result.size() ;
        if (limitUp > 2)
        {
            unsigned int i = p3d_random_integer(1,limitUp-2);
            unsigned int j = p3d_random_integer(i+1,limitUp - 1);
            tmp = findShortCut(result,i,j);
            bool test = robotCanDoTraj(tmp,robCyl,robot,0.2);
            if (test)
            {
                result = tmp;
            }
        }
        else
        {
            break;
        }
    }
    return result;

}

std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > PlannarTrajectorySmoothing::findShortCut(
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, int i, int j)
{
    if (i > (int)traj.size() || j > (int)traj.size() || i < 1 || j < 1)
    {
        return traj;
    }

    double errorT = 0.05;
    Vector2d p1 = getRandomPointInSegment(traj.at(i-1),traj.at(i),errorT);
    Vector2d p2 = getRandomPointInSegment(traj.at(j-1),traj.at(j),errorT);


    double oldDist = computeDistFromTraj(traj,i-1,j);
    double newDist = sqrt(pow(traj.at(i-1)[1]-p1[1],2) + pow(traj.at(i-1)[0]-p1[0],2)) +
                     sqrt(pow(p2[1]-p1[1],2) + pow(p2[0]-p1[0],2)) +
                     sqrt(pow(p2[1]-traj.at(j)[1],2) + pow(p2[0]-traj.at(j)[0],2));
    if (oldDist <= newDist)
    {
        return traj;
    }

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > res;
    bool first=true;
    for  (int k = 0; k < (int)traj.size() ;k++)
    {

        if (k < i || k>= j)
        {
            res.push_back(traj.at(k));
        }
        else
        {
            if (first)
            {
                first= false;
                res.push_back(p1);
                res.push_back(p2);
            }
        }
    }
    res.push_back(traj.at(traj.size()-1));

    return res;
}


Eigen::Vector2d PlannarTrajectorySmoothing::getRandomPointInSegment(Eigen::Vector2d p1, Eigen::Vector2d p2, double errorT)
{
    if ((p2[0] - p1[0]) != 0)
    {
        double pente =  (p2[1] - p1[1])/(p2[0] - p1[0]);
        double cst = p2[1] - pente*p2[0];
    //    double dist = sqrt(pow(p2[1]-p1[1],2) + pow(p2[0]-p1[0],2));
        double xDist = fabs(p2[0] - p1[0]);
        double xRand = p3d_random(0, xDist);
        Eigen::Vector2d res;
        double xMin = p1[0];
        if (p2[0]<p1[0])
        {
            xMin = p2[0];
        }
        res[0] = xRand + xMin;
        res[1] = pente*res[0] + cst;
    //    cout << "randCell:\n" << res << endl;

        double dist = sqrt(pow(res[1]-p1[1],2) + pow(res[0]-p1[0],2));
        if (dist < errorT)
        {
            res = p1;
        }
        dist = sqrt(pow(res[1]-p2[1],2) + pow(res[0]-p2[0],2));
        if (dist < errorT)
        {
            res = p2;
        }

        return res;
    }
    else
    {
        Eigen::Vector2d res;
        res[0] = p1[0];
        if(p2[1] > p1[1])
        {
            res[1] = p3d_random(p1[1],p2[1]);
        }
        else
        {
            res[1] = p3d_random(p2[1],p1[1]);
        }
        return res;
    }

}


std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > PlannarTrajectorySmoothing::add3Dim(
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, Robot* robot, double epsilon)
{

    shared_ptr<Configuration> q_robot (robot->getCurrentPos());

    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > result;
    Vector3d tmp;
    tmp[0] = (*q_robot)[6];
    tmp[1] = (*q_robot)[7];
    tmp[2] = (*q_robot)[11];
    result.push_back(tmp);
    for (unsigned int i = 1; i < traj.size();i++)
    {
        double x1 = traj.at(i-1)[0];
        double y1 = traj.at(i-1)[1];
        double x2 = traj.at(i)[0];
        double y2 = traj.at(i)[1];

        if ((x1 - x2) < epsilon && (y1 -y2) < epsilon)
        {
            continue;
        }

        double angle = atan2(y2-y1,x2-x1);

        double minDist = 0.3;
        double segSize = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
        if (segSize > minDist*2)
        {
            tmp[0] = x1+minDist*cos(angle);
            tmp[1] = y1+minDist*sin(angle);
            tmp[2] = angle;
            result.push_back(tmp);
    //

            tmp[0] = x2-minDist*cos(angle);
            tmp[1] = y2-minDist*sin(angle);
            tmp[2] = angle;
            result.push_back(tmp);
        }
        else
        {
            tmp[0] = x1+(segSize/2)*cos(angle);
            tmp[1] = y1+(segSize/2)*sin(angle);
            tmp[2] = angle;
            result.push_back(tmp);
        }

    }
//    return result;

    return removeSamePoints(result,epsilon);
}



std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > PlannarTrajectorySmoothing::removeSamePoints(
        std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > traj, double epsilon)
{
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > result;
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > tmp = traj;
//    for (int k = 0; k< 4; k ++)
//    {
        result.clear();
        for (unsigned int i = 1; i < tmp.size(); i++)
        {
            if (fabs(tmp.at(i-1)[0] - tmp.at(i)[0]) > epsilon ||
                fabs(tmp.at(i-1)[1] - tmp.at(i)[1]) > epsilon ||
                fabs(tmp.at(i-1)[2] - tmp.at(i)[2]) > epsilon )
            {
                result.push_back(tmp.at(i));
            }
        }
        tmp = result;
//    }
    return result;
}



