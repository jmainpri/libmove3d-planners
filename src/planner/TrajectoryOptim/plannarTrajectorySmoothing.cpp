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

double PlannarTrajectorySmoothing::computeDistOfTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, int begin, int end)
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



double computeDist(Vector2d v, Vector2d w, Vector2d p)
{
    double l2 = pow(v[0] - w[0],2) + pow(v[1] - w[1],2);
    if (l2 == 0)
    {
        return sqrt(pow(v[0] - p[0],2) + pow(v[1] - p[1],2));
    }
    double t = (((p[0] - v[0]) * (w[0] -v[0])) + ((p[1] - v[1]) * (w[1] - v[1])))/l2;
    if (t < 0)
    {
        return sqrt(pow(v[0] - p[0],2) + pow(v[1] - p[1],2));
    }
    if (t > 1)
    {
        return sqrt(pow(w[0] - p[0],2) + pow(w[1] - p[1],2));
    }
    Vector2d tmp;
    tmp[0] = v[0] + (t * (w[0] - v[0]));
    tmp[1] = v[1] + (t * (w[1] - v[1]));

    return sqrt(pow(tmp[0] - p[0],2) + pow(tmp[1] - p[1],2));
}




double PlannarTrajectorySmoothing::computeDistBetweenTrajAndPoint(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, Eigen::Vector2d p)
{
    /*
      for each pair of point
      compute dist between them and p
      return the smallest dist
      */
    double dist = numeric_limits<double>::max( );

    for (unsigned int i =0; i < traj.size() - 1; i++)
    {
        //compute distance (dtmp) between segment [traj[i],traj[i+1]] and p
        double dtmp = computeDist(traj.at(i),traj.at(i+1),p);
        if ( dtmp < dist)
        {
            dist = dtmp;
        }
    }
    return dist;
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

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > result;
    if (trajectory.empty())
    {
        Vector2d v;
        v[0] = (*robot->getCurrentPos())[6];
        v[1] = (*robot->getCurrentPos())[7];
        result.push_back(v);
        return result;
    }
    Robot* cyl;
    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);
        if (robot->getName().find("ROBOT") != string::npos)
        {
            if(name.find("PR_2CYLINDER") != string::npos )
            {
                cyl = new Robot(XYZ_ENV->robot[i]);
                break;
            }
        }
        else if (robot->getName().find("HUMAN") != string::npos)
        {
            if(name.find("HUMCYLINDER") != string::npos )
            {
                cyl = new Robot(XYZ_ENV->robot[i]);
                break;
            }
        }
    }


    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > tmp;

    result.clear();
    result = trajectory;
    tmp = removeUnnecessaryPoint(result,0.06);
    if (robotCanDoTraj(tmp,cyl,robot,0.2))
    {
        result = tmp;
    }

    bool finished = false;
    unsigned int id = 0;
    while (!finished)
    {
        result = findShortCut(result,id,cyl,robot);
        id++;
        if (id > result.size())
        {
            finished = true;
        }
//        if (robotCanDoTraj(tmp,cyl,robot,0.1))
//        {
//            result = tmp;
//        }
    }









//    result = removeUnnecessaryPoint(trajectory,0.03);

//    for (int m = 0; m< 100; m++)
//    int am = 0;
////    double bm = 0;
////    double dist = computeDistOfTraj(result,0,result.size()-1);
//
//
//    while (am < 100)
//    {
//
//        int limitUp = result.size() ;
//        if (limitUp > 2)
//        {
//            unsigned int i = p3d_random_integer(1,limitUp-2);
//            unsigned int j = p3d_random_integer(i+1,limitUp - 1);
//            tmp = findShortCut(result,i,j);
//            bool test = robotCanDoTraj(tmp,cyl,robot,0.1);
//            if (test)
//            {
//                result = tmp;
//            }
//        }
//        else
//        {
//            break;
//        }
//
////        tmp = removeUnnecessaryPoint(result,0.07);
////        if (robotCanDoTraj(tmp,cyl,robot,0.2))
////        {
////            result = tmp;
////        }
//
//        am++;
////        double tmpDist = numeric_limits<double>::max( );
////        dist = computeDistOfTraj(result,0,result.size()-1);
////        cout << "distance = " << dist << "for " << am << endl;
////        if (tmpDist < dist)
////        {
////            bm =0;
////            dist = tmpDist;
////        }
////        else
////        {
////            bm++;
////            if (bm> 40)
////            {
////                break;
////            }
////        }
//
//    }

//    tmp = removeUnnecessaryPoint(result,0.06);
//    if (robotCanDoTraj(tmp,cyl,robot,0.2))
//    {
//        result = tmp;
//    }

//    for (unsigned int j =0; j< result.size();j++)
//    {
//        cout << "cell nb " << j << " with coord:\n" << result.at(j) << endl;
//    }

    return result;

}


std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > PlannarTrajectorySmoothing::findShortCut(
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, int i, Robot* cyl, Robot* robot)
{
    double errorT = EPS1/2;

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > tmp = traj;
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > res = traj;
    bool ended = false;
    int id = 0;
    while (!ended && tmp.size()>2 && i+1 < tmp.size()-1 )
    {

        tmp.erase(tmp.begin()+i+1);
        if (!robotCanDoTraj(tmp,cyl,robot,0.1))
        {
            break;
        }
        if(computeDistOfTraj(tmp,0,tmp.size()) <= computeDistOfTraj(res,0,res.size()))
        {
            res = tmp;
        }


    }
    if (robotCanDoTraj(res,cyl,robot,0.1))
    {
        return res;
    }
    else
    {
        return traj;
    }
}

std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > PlannarTrajectorySmoothing::findShortCut(
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, int i, int j)
{
    if (i > (int)traj.size() || j > (int)traj.size() || i < 1 || j < 1)
    {
        return traj;
    }

    double errorT = EPS1/2;

    Vector2d p1 = getRandomPointInSegment(traj.at(i-1),traj.at(i),errorT);
    Vector2d p2 = getRandomPointInSegment(traj.at(j-1),traj.at(j),errorT);


    double oldDist = computeDistOfTraj(traj,i-1,j);
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

//    for (unsigned int j =0; j< traj.size();j++)
//    {
//        cout << "cell nb " << j << " with coord:\n" << traj.at(j) << endl;
//    }
    shared_ptr<Configuration> q_robot (robot->getCurrentPos());

    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > result;
    result.clear();
    Vector3d tmp;
//    tmp[0] = traj.at(0)[0];
//    tmp[1] = traj.at(0)[1];
//    tmp[2] = (*q_robot)[11];
//    result.push_back(tmp);
    tmp[0] = traj.at(0)[0];
    tmp[1] = traj.at(0)[1];
    tmp[2] = atan2(-(*q_robot)[7]+traj.at(1)[1],-(*q_robot)[6]+traj.at(1)[0]);
    result.push_back(tmp);
    if (traj.size() > 1)
    {
//        cout << "----------- add3Ddim -------------" << endl;
        for (unsigned int i = 1; i < traj.size();i++)
        {
            double x1 = traj.at(i-1)[0];
            double y1 = traj.at(i-1)[1];
            double x2 = traj.at(i)[0];
            double y2 = traj.at(i)[1];
//            cout << "loop nb " << i << endl;
//            cout << "x1 = " << x1 << " x2 = " << x2<<" y1 = " << y1<< " y2 = "<< y2<< endl;
//            cout << "(x1 - x2) = " << (x1 - x2) << " (y1 -y2) = " << (y1 -y2) << " epsilon = " << epsilon << endl;

            if (fabs(x1 - x2) < epsilon && fabs(y1 -y2) < epsilon)
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
//        cout << "----------- end add3Ddim -------------" << endl;


    }
//    else
//    {
//        tmp[0] = traj.at(0)[0];
//        tmp[1] = traj.at(0)[1];
//        tmp[2] = atan2((*q_robot)[7]-tmp[1],(*q_robot)[6]-tmp[0]);;
//        result.push_back(tmp);
//    }


//    result = removeSamePoints(result,epsilon);
//
//    for (unsigned int j =0; j< result.size();j++)
//    {
//        cout << "cell nb " << j << " with coord:\n" << result.at(j) << endl;
//    }
    return result;
}

std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > PlannarTrajectorySmoothing::add3DimwithoutTrajChange(
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > traj, Robot* robot, double epsilon)
{

//    for (unsigned int j =0; j< traj.size();j++)
//    {
//        cout << "cell nb " << j << " with coord:\n" << traj.at(j) << endl;
//    }
    shared_ptr<Configuration> q_robot (robot->getCurrentPos());

    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > result;
    result.clear();
    Vector3d tmp;
//    tmp[0] = traj.at(0)[0];
//    tmp[1] = traj.at(0)[1];
//    tmp[2] = atan2(-(*q_robot)[7]+traj.at(1)[1],-(*q_robot)[6]+traj.at(1)[0]);
//    result.push_back(tmp);
    if (traj.size() > 1)
    {
        for (unsigned int i = 1; i < traj.size();i++)
        {
            double x1 = traj.at(i-1)[0];
            double y1 = traj.at(i-1)[1];
            double x2 = traj.at(i)[0];
            double y2 = traj.at(i)[1];

            if (fabs(x1 - x2) < epsilon && fabs(y1 -y2) < epsilon)
            {
                continue;
            }

            tmp[0] = x1;
            tmp[1] = y1;
            tmp[2] = atan2(y2-y1,x2-x1);;
            result.push_back(tmp);

            if (i == traj.size() - 1)
            {
                tmp[0] = x2;
                tmp[1] = y2;
                tmp[2] = atan2(y2-y1,x2-x1);;
                result.push_back(tmp);
            }
        }
    }
    return result;
}



std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > PlannarTrajectorySmoothing::removeSamePoints(
        std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > traj, double epsilon)
{
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > result;
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > tmp = traj;
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
    
//    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > tmp;
    
    
    return result;
}



std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > PlannarTrajectorySmoothing::get2DtrajFrom3Dtraj(
        std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > traj)
{
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > res;
    for (unsigned int i = 0; i < traj.size(); i++)
    {
        Vector2d tmp;
        tmp[0] = traj.at(i)[0];
        tmp[1] = traj.at(i)[1];
        res.push_back(tmp);
    }
    return res;
}
