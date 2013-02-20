#include "HRICS_EnvGrid.hpp"

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "planEnvironment.hpp"
#include "time.h"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif


class CellDistComp
{
public:

        bool operator()(pair<double,EnvCell*> first, pair<double,EnvCell*> second)
        {
                return ( first.first < second.first );
        }

} CellDistCompObject;

//---------------------------------------------------------------------------
// Grid
//---------------------------------------------------------------------------
EnvGrid::EnvGrid() :
        API::TwoDGrid(),
        mRobot(0x00),
        mHuman(0x00),
        m_humanMaxDist(1),
        m_robotMaxDist(1)
{
}

EnvGrid::EnvGrid(double pace, vector<double> envSize, bool isHumanCentered) :
        API::TwoDGrid(pace,envSize),
        mRobot(0x00),
        mHuman(0x00),
        m_humanMaxDist(1),
        m_robotMaxDist(1)
{
    createAllCells();
    cout << "Number total of cells = " << _nbCellsX*_nbCellsY << endl;
    ENV.setDouble(Env::optimalDistFactor,0.5);
    ENV.setDouble(Env::robotMaximalDistFactor,0);
    ENV.setDouble(Env::gazeAngleFactor,1.0);
}

EnvGrid::EnvGrid(double pace, vector<double> envSize, bool isHumanCentered, Robot* robot, Robot* human) :
        API::TwoDGrid(pace,envSize),
        mRobot(robot),
        mHuman(human),
        m_humanMaxDist(1),
        m_robotMaxDist(1)
{
    createAllCells();
    cout << "Number total of cells = " << _nbCellsX*_nbCellsY << endl;
    ENV.setDouble(Env::optimalDistFactor,0.5);
    ENV.setDouble(Env::robotMaximalDistFactor,0);
    ENV.setDouble(Env::gazeAngleFactor,1.0);
}

void EnvGrid::init(pair<double,double> minMax)
{
    bool showText = PlanEnv->getBool(PlanParam::env_showText);
    if (showText)
    {
        cout << "in: EnvGrid::init(pair<double,double> minMax)\n" << endl;
//        cout << "Store position of all robot in the scene (object too); and move them out of the scene" << endl;
    }
    vector<pair<Robot*,shared_ptr<Configuration> > > initConfiguration;
    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);
        if(name.find("HUMCYLINDER") != string::npos )
        {
            humCyl = new Robot(XYZ_ENV->robot[i]);
        }
        else if (name.find("PR_2CYLINDER") != string::npos)
        {
            robotCyl = new Robot(XYZ_ENV->robot[i]);
        }
//        Robot* r = new Robot(XYZ_ENV->robot[i]);
//        pair<Robot*,shared_ptr<Configuration> > p;
//        p.first = r;
//        shared_ptr<Configuration> q = r->getCurrentPos();
//        p.second = r->getCurrentPos();
//        initConfiguration.push_back(p);
//
//        (*q)[6] = -3;
//        r->setAndUpdate(*q);
    }

    if (!humCyl || !robotCyl)
    {
        cout << "No cylinder to make reacheability tests. The Otp may segfault." << endl;
        return;
    }

    if (showText)
    {
//        cout << "All Robot moved with success\n" << endl;
        cout << "Compute reacheability of each cell by the robot and by the human" << endl;
    }
    initAllReachability();

    Robot* robot = getRobot();
    shared_ptr<Configuration> q_robot_cur = robot->getCurrentPos();
    shared_ptr<Configuration> q_robot = robot->getCurrentPos();
    (*q_robot)[6] = 0;
    (*q_robot)[7] = 0;
    robot->setAndUpdate(*q_robot);
    Robot* human = getHuman();
    shared_ptr<Configuration> q_human_cur = human->getCurrentPos();
    shared_ptr<Configuration> q_human = human->getCurrentPos();
    (*q_human)[6] = 0;
    (*q_human)[7] = 0;
    human->setAndUpdate(*q_human);


    for (unsigned int x =0;x<_nbCellsX;++x)
    {
        for (unsigned int y =0;y<_nbCellsY;++y)
        {
            EnvCell* Cell = dynamic_cast<EnvCell*>(getCell(x,y));

            Cell->computeHumanReach();
            if (Cell->isHumAccessible())
            {
                m_HumanAccessible.push_back(Cell);
            }
            Cell->computeRobotReach();
            if (Cell->isRobAccessible())
            {
                m_RobotAccessible.push_back(Cell);
            }
        }
    }

    human->setAndUpdate(*q_human_cur);
    robot->setAndUpdate(*q_robot_cur);

    if (showText)
    {
        cout << "Reacheability computed\n" << endl;
        cout << "find in which cell the robot can be placed accordingly to each cell where the human can be" << endl;
    }

    computeHumanRobotReacheability(minMax);
    std::vector<EnvCell*> tmpVect;
    for (unsigned int i = 0; i < m_HumanAccessible.size();i++ )
    {
        if(m_HumanAccessible.at(i)->isHumAccessible())
        {
            tmpVect.push_back(m_HumanAccessible.at(i));
        }
    }
    m_HumanAccessible.clear();
    m_HumanAccessible = tmpVect;
    if (showText)
    {
        cout << "Robot cells list found\n" << endl;
        cout << "Replace all robot to there original place (object too)" << endl;
    }

    for(unsigned int i=0;i < initConfiguration.size(); i++)
    {
        initConfiguration.at(i).first->setAndUpdate(*initConfiguration.at(i).second);
    }
    if (showText)
    {
        cout << "All robot are replaced\n" << endl;
        cout << "Move the cylinder out of the scenne" << endl;
    }
    shared_ptr<Configuration> q = humCyl->getCurrentPos();
    (*q)[6] = 0;
    (*q)[7] = 0;
    humCyl->setAndUpdate(*q);

     q = robotCyl->getCurrentPos();
    (*q)[6] = 0;
    (*q)[7] = 0;
    robotCyl->setAndUpdate(*q);

    if (showText)
    {
        cout << "Cylinder has been moved\n" << endl;
        cout << "out of: EnvGrid::init(pair<double,double> minMax)\n\n" << endl;
    }
}

void EnvGrid::initGrid(Eigen::Vector3d humanPos)
{
    bool showText = PlanEnv->getBool(PlanParam::env_showText);
    if (showText)
    {
        cout << "in: void EnvGrid::initGrid()\n" << endl;
        cout << "Compute cells reacheability for both human and robot with the obstacles" << endl;
    }
    initAllReachability();

    Robot* robot = getRobot();
    shared_ptr<Configuration> q_robot_cur = robot->getCurrentPos();
    shared_ptr<Configuration> q_robot = robot->getCurrentPos();
    (*q_robot)[6] = 10;
    (*q_robot)[7] = 1;
    robot->setAndUpdate(*q_robot);
//    for (unsigned int i = 0; i < m_HumanAccessible.size(); i++)
//    {
//        m_HumanAccessible.at(i)->computeHumanReach();
//    }
    robot->setAndUpdate(*q_robot_cur);


    Robot* human = getHuman();
    shared_ptr<Configuration> q_human_cur = human->getCurrentPos();
    shared_ptr<Configuration> q_human = human->getCurrentPos();
    (*q_human)[6] = 10;
    (*q_human)[7] = 1;
    human->setAndUpdate(*q_human);
//    for (unsigned int i = 0; i < m_RobotAccessible.size(); i++)
//    {
//        m_RobotAccessible.at(i)->computeRobotReach();
//    }
    human->setAndUpdate(*q_human_cur);

    if (showText)
    {
        cout << "End computing reacheability\n" << endl;
    }

    if (!humCyl || !robotCyl)
    {
        for (int i=0; i<XYZ_ENV->nr; i++)
        {
            string name(XYZ_ENV->robot[i]->name);
            if(name.find("HUMCYLINDER") != string::npos )
            {
                humCyl = new Robot(XYZ_ENV->robot[i]);
            }
            else if (name.find("PR_2CYLINDER") != string::npos)
            {
                robotCyl = new Robot(XYZ_ENV->robot[i]);
            }
        }
        if (!humCyl || !robotCyl)
        {
            cout << "No cylinder to make reacheability tests. The Otp may segfault. (in initGrid)" << endl;
            return;
        }
    }

    if (showText)
    {
        cout << "Compute human distances" << endl;
    }
    initAllCellState();

//    qCurrentPos();

    int firstIndexOfHumanDof = mHuman->getJoint("Pelvis")->getIndexOfFirstDof();
    Vector2d pos;
    pos[0] = (*q_human_cur)[firstIndexOfHumanDof + 0];
    pos[1] = (*q_human_cur)[firstIndexOfHumanDof + 1];

    pos[0] = humanPos[0];
    pos[1] = humanPos[1];

    EnvCell* cell = dynamic_cast<EnvCell*>(getCell(pos));
    computeHumanDistances(cell);
    if (showText)
    {
        cout << "Human distances computed with sucess\n" << endl;
        cout << "Compute Robot distances" << endl;
    }

    initAllCellState();

    q_robot_cur = mRobot->getCurrentPos();

    pos[0] = (*q_robot_cur)[firstIndexOfHumanDof + 0];
    pos[1] = (*q_robot_cur)[firstIndexOfHumanDof + 1];

    cell = dynamic_cast<EnvCell*>(getCell(pos));
    computeRobotDistances(cell);
    if (showText)
    {
        cout << "Robot distances computed with success\n" << endl;
        cout << "Update Reacheability in function of the computed distances, and compute if necessary the angle of human arrival" << endl;
    }

    for (unsigned int x =0;x<_nbCellsX;++x)
    {
        for (unsigned int y =0;y<_nbCellsY;++y)
        {
            EnvCell* Cell = dynamic_cast<EnvCell*>(getCell(x,y));
            Cell->setHumAccessible(Cell->isHumanDistComputed());
            Cell->setRobAccessible(Cell->isRobotDistComputed());
            if (PlanEnv->getBool(PlanParam::env_useOrientedSlice))
            {
                if (Cell->getHumanTraj().size() > 1)
                {
                    Vector2d prevCenter = Cell->getHumanTraj().at(Cell->getHumanTraj().size() - 2)->getCenter();
                    Cell->setAngleForHumanComming(atan2(Cell->getCenter()[1] - prevCenter[1], Cell->getCenter()[0] - prevCenter[0]));
    //                cout << "x = " << x << " y = " << y << " angle = " << Cell->getAngleForHumanComming() << endl;
                }
                else
                {
                    Cell->setAngleForHumanComming(numeric_limits<double>::max());
                }
            }

        }
    }
    if (showText)
    {
        cout << "End updating\n" << endl;
        cout << "Update Human robot reacheability while the obstacles is in the scenne" << endl;
    }


    if (!PlanEnv->getBool(PlanParam::env_normalRand) && !PlanEnv->getBool(PlanParam::env_useAllGrid))
    {
        for (unsigned int i = 0; i < m_HumanAccessible.size(); i++)
        {
            EnvCell* cell = dynamic_cast<EnvCell*>(m_HumanAccessible.at(i));
            vector<EnvCell*> newVect;
            pair<double,EnvCell*> p;
            p.first = numeric_limits<double>::max( );
            for (unsigned int j = 0; j < cell->getHumanRobotReacheable().size(); j++)
            {
//                if (cell->getHumanRobotReacheable().at(j)->isRobAccessible())
//                {
//                    newVect.push_back(cell->getHumanRobotReacheable().at(j));
                    if (cell->getHumanRobotReacheable().at(j)->getRobotDist() < p.first)
                    {
                        p.first = cell->getHumanRobotReacheable().at(j)->getRobotDist();
                        p.second = cell->getHumanRobotReacheable().at(j);
                    }

//                }
            }
    //        p.first = max((p.second->getRobotDist()/ m_robotMaxDist)*PlanEnv->getDouble(PlanParam::env_objectNessecity) ,(cell->getHumanDist()/m_humanMaxDist));
    //        p.first = 0;
            cell->clearCurrentHumanRobotReacheable();
            cell->setCurrentHumanRobotReacheable(cell->getHumanRobotReacheable());
            cell->setRobotBestPos(p);
        }
    }
    if (showText)
    {
        cout << "End updating\n" << endl;
        cout << "out of: void EnvGrid::initGrid()\n" << endl;
    }
    gridIsSorted = false;
}

void EnvGrid::recomputeGridWhenHumanMove(Eigen::Vector3d humanPos)
{
    initAllTrajs();
    initAllCellState();

    shared_ptr<Configuration> q_human_cur = mHuman->getCurrentPos();
    shared_ptr<Configuration> q_robot_cur = mRobot->getCurrentPos();
    shared_ptr<Configuration> q_human = mHuman->getCurrentPos();
    shared_ptr<Configuration> q_robot = mRobot->getCurrentPos();

    int firstIndexOfHumanDof = mHuman->getJoint("Pelvis")->getIndexOfFirstDof();
    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(mRobot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;


    (*q_human)[firstIndexOfHumanDof + 0] = 0;
    (*q_human)[firstIndexOfHumanDof + 1] = 0;
    mHuman->setAndUpdate(*q_human);


    Vector2d pos;
    pos[0] = (*q_robot_cur)[firstIndexOfRobotDof + 0];
    pos[1] = (*q_robot_cur)[firstIndexOfRobotDof + 1];

    EnvCell* cell = dynamic_cast<EnvCell*>(getCell(pos));
    computeRobotDistances(cell);

    mHuman->setAndUpdate(*q_human_cur);

    initAllCellState();


    pos[0] = humanPos[0];
    pos[1] = humanPos[1];

    (*q_robot)[firstIndexOfRobotDof + 0] = 0;
    (*q_robot)[firstIndexOfRobotDof + 1] = 0;
    mRobot->setAndUpdate(*q_robot);

     cell = dynamic_cast<EnvCell*>(getCell(pos));
    computeHumanDistances(cell);

    mRobot->setAndUpdate(*q_robot_cur);

    gridIsSorted = false;
}


void EnvGrid::computeHumanRobotReacheability(std::pair<double,double> minMax)
{
    for (unsigned int i = 0; i < m_HumanAccessible.size(); i++)
    {
        m_HumanAccessible.at(i)->getCrown(minMax.first,minMax.second);
    }
}

std::vector<std::pair<double,EnvCell*> > EnvGrid::getSortedGrid()
{

    if (!gridIsSorted)
    {
        std::vector<std::pair<double,EnvCell*> > vect;
        for (unsigned int i = 0; i < m_HumanAccessible.size() ; i++)
        {
            if (m_HumanAccessible.at(i)->isHumanDistComputed() && m_HumanAccessible.at(i)->getCurrentHumanRobotReacheable().size() > 0)
            {
                pair<double,EnvCell*> p;
                p.second = m_HumanAccessible.at(i);
                p.first = p.second->getCost();
                vect.push_back(p);
            }

//	    cout << "p.second->getCurrentHumanRobotReacheable().size() = " << p.second->getCurrentHumanRobotReacheable().size() << endl;
//	    cout << "p.second->isHumAccessible() = "  << p.second->isHumAccessible() << endl;

//            if (p.second->getCurrentHumanRobotReacheable().size() > 0 && p.second->isHumAccessible())
//            {
////                p.first = max((m_HumanAccessible.at(i)->getRobotBestPos().first / m_robotMaxDist)*PlanEnv->getDouble(PlanParam::env_objectNessecity)
////                               ,(m_HumanAccessible.at(i)->getHumanDist()/m_humanMaxDist));
//                double hTime = m_HumanAccessible.at(i)->getHumanDist()/humanSpeed;
//                double rTime = m_HumanAccessible.at(i)->getRobotBestPos().first/robotSpeed;
//                double tempCost = max(hTime,rTime) * timeStamp;
//
//                double mvCost = m_HumanAccessible.at(i)->getHumanDist();
//                p.first = (ksi * mvCost) * (1 - objectNecessity)/(rho + ksi) + tempCost * objectNecessity;
//                vect.push_back(p);
//            }
        }
        sort(vect.begin(),vect.end(),CellDistCompObject);
        sortedGrid = vect;
        gridIsSorted = true;
    }
    else
      {
//	cout << "Grid : Is not sorted" << endl;
      }
    return sortedGrid;
}

API::TwoDCell* EnvGrid::createNewCell(unsigned int index,unsigned  int x,unsigned  int y )
{
    Vector2i coord;

    coord[0] = x;
    coord[1] = y;

    if (index == 0)
    {
        return new EnvCell( 0, coord, _originCorner , this );
    }
    API::TwoDCell* newCell = new EnvCell( index, coord, computeCellCorner(x,y) , this );
    return newCell;
}

void EnvGrid::draw()
{
    if( mRobot == 0x00 )
    {
        std::cout << "Error : PlanGrid::draw() => No Robot "  << std::endl;
    }

#ifdef LIGHT_PLANNER
	deactivateCcCntrts(mRobot->getRobotStruct(),-1);
#else
	cout << "Warning: Lihght Planner not compiled" << endl;
#endif
	
	double colorvector[4];

    colorvector[0] = 1.0;       //red
    colorvector[1] = 0.5;       //green
    colorvector[2] = 0.0;       //blue
    colorvector[3] = 0.05;       //transparency


    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);

    glEnable(GL_CULL_FACE);
    glBegin(GL_QUADS);


    double color[3];


    std::vector<std::pair<double,EnvCell*> > sortedCells;
    std::vector<double> cellToDraw;
    std::vector<double> cellsCost;
    double maxCost = 0;
    double minCost = 100;
    if (PlanEnv->getBool(PlanParam::env_drawDistGrid))
    {
        sortedCells = getSortedGrid();
        for (unsigned int k = 0; k < sortedCells.size(); k++)
        {
            cellToDraw.push_back(sortedCells.at(k).second->getIndex());
            cellsCost.push_back(sortedCells.at(k).first);
            if(sortedCells.at(k).first > maxCost)
            {
                maxCost = sortedCells.at(k).first;
            }
            if (sortedCells.at(k).first < minCost)
            {
                minCost = sortedCells.at(k).first;
            }
       }
    }



    for (unsigned int x =0;x<_nbCellsX;++x)
    {
        for (unsigned int y =0;y<_nbCellsY;++y)
        {

            EnvCell* Cell = dynamic_cast<EnvCell*>(getCell(x,y));
            double colorRation = 0;
            bool isToDraw = false;
            if (PlanEnv->getBool(PlanParam::env_humanGridDraw))
            {
                isToDraw = Cell->isHumAccessible();
                colorRation = Cell->getHumanDist() / m_humanMaxDist;
            }

            if (PlanEnv->getBool(PlanParam::env_robotGridDraw))
            {
                isToDraw = Cell->isRobAccessible();
                colorRation = Cell->getRobotDist() / m_robotMaxDist;
            }

            if (PlanEnv->getBool(PlanParam::env_drawDistGrid))
            {
                double cost = 1;
                for (unsigned int k = 0; k < cellToDraw.size(); k++)
                {
                    if (Cell->getIndex() == cellToDraw.at(k))
                    {
                        isToDraw = true;
                        cost = cellsCost.at(k);
                    }
                }
                colorRation = (cost-minCost)/(maxCost-minCost);
            }


            if (isToDraw)
            {
                if (!PlanEnv->getBool(PlanParam::env_drawRandomPoint))
                {
                    Vector2d center = Cell->getCenter();

                    //            double colorRation = (((double)x*(double)_nbCellsY)+(double)y)/(nbCells);
                    //            cout << " X = "  << _nbCellsX << " , Y = "  << _nbCellsY << endl;
                    //            cout << "ColorRation[" << x*_nbCellsY+y << "]  =  "  << colorRation << endl;

        //            colorRation = colorRation*ENV.getDouble(Env::colorThreshold2)*1000;
                    GroundColorMixGreenToRed(color,colorRation);
    //                colorRation = depth;

                    double depth;
//                    depth =colorRation;
                    depth =0.1;
                    glColor3d(color[0],color[1],color[2]);
                    glVertex3d( (double)(center[0] - _cellSize[0]/2) + 0.01 , (double)(center[1] - _cellSize[1]/2) + 0.01, depth);//depth );
                    glVertex3d( (double)(center[0] + _cellSize[0]/2) - 0.01 , (double)(center[1] - _cellSize[1]/2) + 0.01, depth);//depth );
                    glVertex3d( (double)(center[0] + _cellSize[0]/2) - 0.01 , (double)(center[1] + _cellSize[1]/2) - 0.01, depth);//depth );
                    glVertex3d( (double)(center[0] - _cellSize[0]/2) + 0.01 , (double)(center[1] + _cellSize[1]/2) - 0.01, depth);//depth );
                }
                else
                {
                    Vector2d center = Cell->getCenter();
                    int color = Grey;
                    double height = 0.1;
                    if (x ==  (unsigned int)PlanEnv->getInt(PlanParam::env_xToDraw) && y == (unsigned int)PlanEnv->getInt(PlanParam::env_yToDraw))
                    {
                        color = Yellow;

                        for (unsigned int i = 0; i < Cell->getCurrentHumanRobotReacheable().size(); i++)
                        {
                            Vector2d center = Cell->getCurrentHumanRobotReacheable().at(i)->getCenter();
                            double xMin = (double)(center[0] - _cellSize[0]/2);
                            double xMax = (double)(center[0] + _cellSize[0]/2);
                            double yMin = (double)(center[1] - _cellSize[1]/2);
                            double yMax = (double)(center[1] + _cellSize[1]/2);
                            height = 0.101;
                            int distColor = Blue;
                            if (Cell->getRobotBestPos().second->getIndex() == Cell->getCurrentHumanRobotReacheable().at(i)->getIndex())
                            {
                                height = 0.102 ;
                                distColor = Yellow;
                            }

                            glLineWidth(3.);
                            g3d_drawOneLine(xMin,   yMin,    height,
                                            xMin,   yMax,    height,
                                            distColor, NULL);
                            glLineWidth(1.);

                            glLineWidth(3.);
                            g3d_drawOneLine(xMin,   yMax,    height,
                                            xMax,   yMax,    height,
                                            distColor, NULL);
                            glLineWidth(1.);

                            glLineWidth(3.);
                            g3d_drawOneLine(xMax,   yMax,    height,
                                            xMax,   yMin,    height,
                                            distColor, NULL);
                            glLineWidth(1.);

                            glLineWidth(3.);
                            g3d_drawOneLine(xMax,   yMin,    height,
                                            xMin,   yMin,    height,
                                            distColor, NULL);
                            glLineWidth(1.);
                        }

//                        for(unsigned int i=0;i<Cell->getHumanTraj().size()-1;i++)
//                        {
//                            glLineWidth(3.);
//                            g3d_drawOneLine(Cell->getHumanTraj().at(i)->getCenter()[0],      Cell->getHumanTraj().at(i)->getCenter()[1],      0.4,
//                                            Cell->getHumanTraj().at(i + 1)->getCenter()[0],  Cell->getHumanTraj().at(i + 1)->getCenter()[1],  0.4,
//                                            Green, NULL);
//                            glLineWidth(1.);
//                        }
                    }
                    double xMin = (double)(center[0] - _cellSize[0]/2);
                    double xMax = (double)(center[0] + _cellSize[0]/2);
                    double yMin = (double)(center[1] - _cellSize[1]/2);
                    double yMax = (double)(center[1] + _cellSize[1]/2);

                    glLineWidth(3.);
                    g3d_drawOneLine(xMin,   yMin,    height,
                                    xMin,   yMax,    height,
                                    color, NULL);
                    glLineWidth(1.);

                    glLineWidth(3.);
                    g3d_drawOneLine(xMin,   yMax,    height,
                                    xMax,   yMax,    height,
                                    color, NULL);
                    glLineWidth(1.);

                    glLineWidth(3.);
                    g3d_drawOneLine(xMax,   yMax,    height,
                                    xMax,   yMin,    height,
                                    color, NULL);
                    glLineWidth(1.);

                    glLineWidth(3.);
                    g3d_drawOneLine(xMax,   yMin,    height,
                                    xMin,   yMin,    height,
                                    color, NULL);
                    glLineWidth(1.);



                    if (Cell->getRandomVector().size() > 0)
                    {
                        for (unsigned int i=0; i< Cell->getRandomVector().size(); i++)
                        {
                            double angle = Cell->getRandomVector().at(i);
                            double depth = i*0.1;

                            p3d_vector3 origin, end;
                            end[0] = Cell->getCenter()[0] + cos(angle)*(_cellSize[0]/2);
                            end[1] = Cell->getCenter()[1] + sin(angle)*(_cellSize[1]/2);
                            end[2] = depth;

                            origin[0] = Cell->getCenter()[0];
                            origin[1] = Cell->getCenter()[1];
                            origin[2] = depth;

                            g3d_draw_arrow(origin, end, 1, 0, 0);
                        }
                    }
                }
            }
        }
    }


    //    for(int i=0; i<nbCells; i++)
    //    {
    //        TwoDCell* cell = static_cast<TwoDCell*>(getCell(i));
    //        glColor4dv(colorvector);
    //        cell->draw();
    //    }

    glEnd();

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

    //    glEnable(GL_LIGHTING);
    //    glEnable(GL_LIGHT0);
}

void EnvGrid::setCellsToblankCost()
{
    for (int i = 0; i < getNumberOfCells(); i++)
    {
        dynamic_cast<EnvCell*>(this->getCell(i))->setBlankCost();
    }

}



void EnvGrid::initAllCellState()
{
    for (int i = 0; i < getNumberOfCells(); i++)
    {
        dynamic_cast<EnvCell*>(this->getCell(i))->resetExplorationStatus();
    }
}

void EnvGrid::initAllTrajs()
{
    for (int i = 0; i < getNumberOfCells(); i++)
    {
        dynamic_cast<EnvCell*>(this->getCell(i))->resetTraj();
    }
}

void EnvGrid::initAllReachability()
{
    for (int i = 0; i < getNumberOfCells(); i++)
    {
        dynamic_cast<EnvCell*>(this->getCell(i))->resetReacheability();

    }
    m_humanMaxDist = 0;
    m_robotMaxDist = 0;
}


void  EnvGrid::computeHumanDistances(EnvCell* cell)
{
    cell->setOpen();
    vector<EnvCell*> openCells;
    openCells.push_back(cell);

    cell->setHumAccessible(true);
    cell->setHumanDist(0);
    cell->setHumanDistIsComputed();
    cell->computeCost();

    std::vector<std::pair<double,EnvCell*> > vect;

    pair<double,EnvCell*> p;
    p.second = cell;
    p.first = cell->getCost();
    vect.push_back(p);

    while (openCells.size() > 0)
    {
        EnvCell* currentCell = openCells.front();
        vector<EnvCell*> currentNeighbors = currentCell->getNeighbors(true);

        double currentDist = 0;

        if (currentCell->isHumanDistComputed())
        {
            currentDist = currentCell->getHumanDist();
        }
        else
        {
            currentCell->setHumanDistIsComputed();
            currentCell->setHumAccessible(true);
        }

        for (unsigned int i = 0; i < currentNeighbors.size(); i++)
        {
            EnvCell* neighCell = currentNeighbors.at(i);
            if (!neighCell->getClosed())
            {
                double dist = currentDist + currentCell->computeDist(neighCell);
                if (neighCell->getOpen())
                {
                    if (dist > neighCell->getHumanDist())
                    {
                        dist = neighCell->getHumanDist();
                    }
                    else
                    {
                        vector<EnvCell*> traj = currentCell->getHumanTraj();
                        traj.push_back(neighCell);

                        vector<Vector2d,Eigen::aligned_allocator<Vector2d> > vectorTraj = currentCell->getHumanVectorTraj();
                        vectorTraj.push_back(neighCell->getCenter());

                        neighCell->setHumanTraj(traj);
                        neighCell->setHumanVectorTraj(vectorTraj);
                    }
                }
                else
                {
                    vector<EnvCell*> traj = currentCell->getHumanTraj();
                    traj.push_back(neighCell);

                    vector<Vector2d,Eigen::aligned_allocator<Vector2d> > vectorTraj = currentCell->getHumanVectorTraj();
                    vectorTraj.push_back(neighCell->getCenter());

                    neighCell->setHumanTraj(traj);
                    neighCell->setHumanVectorTraj(vectorTraj);

                    neighCell->setOpen();
                    openCells.push_back(neighCell);
                }
                neighCell->setHumanDist(dist);
                neighCell->setHumanDistIsComputed();
                neighCell->setHumAccessible(true);
                if (dist > m_humanMaxDist) {m_humanMaxDist = dist; }
//                neighCell->computeBestRobotPos();
                neighCell->computeCost();


                pair<double,EnvCell*> p;
                p.second = neighCell;
                p.first = p.second->getCost();
                vect.push_back(p);
            }

        }
        currentCell->setClosed();
        openCells.erase(openCells.begin());
    }
    sort(vect.begin(),vect.end(),CellDistCompObject);
    sortedGrid = vect;
    gridIsSorted = true;

}

void  EnvGrid::computeRobotDistances(EnvCell* cell)
{
    cell->setOpen();
    vector<EnvCell*> openCells;
    openCells.push_back(cell);


    cell->setRobAccessible(true);
    cell->setRobotDist(0);
    cell->setRobotDistIsComputed();



    while (openCells.size() > 0)
    {

        EnvCell* currentCell = openCells.front();
        vector<EnvCell*> currentNeighbors = currentCell->getNeighbors(false);

        double currentDist = 0;

        if (currentCell->isRobotDistComputed())
        {
            currentDist = currentCell->getRobotDist();
        }
        else
        {
            currentCell->setRobotDistIsComputed();
            currentCell->setRobAccessible(true);
        }


        for (unsigned int i = 0; i < currentNeighbors.size(); i++)
        {
            EnvCell* neighCell = currentNeighbors.at(i);

            if (!neighCell->getClosed())
            {
                double dist = currentDist + currentCell->computeDist(neighCell);
                if (neighCell->getOpen())
                {
                    if (dist > neighCell->getRobotDist())
                    {
                        dist = neighCell->getRobotDist();
                    }
                    else
                    {
                        vector<EnvCell*> traj = currentCell->getRobotTraj();
                        traj.push_back(neighCell);

                        vector<Vector2d,Eigen::aligned_allocator<Vector2d> > vectorTraj = currentCell->getRobotVectorTraj();
                        vectorTraj.push_back(neighCell->getCenter());

                        neighCell->setRobotTraj(traj);
                        neighCell->setRobotVectorTraj(vectorTraj);
                    }
                }
                else
                {
                    vector<EnvCell*> traj = currentCell->getRobotTraj();
                    traj.push_back(neighCell);

                    vector<Vector2d,Eigen::aligned_allocator<Vector2d> > vectorTraj = currentCell->getRobotVectorTraj();
                    vectorTraj.push_back(neighCell->getCenter());

                    neighCell->setRobotTraj(traj);
                    neighCell->setRobotVectorTraj(vectorTraj);

                    neighCell->setOpen();
                    openCells.push_back(neighCell);
                }
                neighCell->setRobotDist(dist);
                neighCell->setRobotDistIsComputed();
                neighCell->setRobAccessible(true);
                if (dist > m_robotMaxDist) {m_robotMaxDist = dist; }

            }

        }
        currentCell->setClosed();
        openCells.erase(openCells.begin());
    }
}


void  EnvGrid::computeDistances(EnvCell* cell, bool ishuman)
{
    cell->setOpen();
    vector<EnvCell*> openCells;
    openCells.push_back(cell);

    if (ishuman)
    {
        cell->setHumAccessible(true);
        cell->setHumanDist(0);
        cell->setHumanDistIsComputed();
    }
    else
    {
        cell->setRobAccessible(true);
        cell->setRobotDist(0);
        cell->setRobotDistIsComputed();
    }


    while (openCells.size() > 0)
    {
        EnvCell* currentCell = openCells.front();
        vector<EnvCell*> currentNeighbors = currentCell->getNeighbors(ishuman);

        double currentDist = 0;
        if (ishuman)
        {
            if (currentCell->isHumanDistComputed())
            {
                currentDist = currentCell->getHumanDist();
            }
            else
            {
                currentCell->setHumanDistIsComputed();
                currentCell->setHumAccessible(true);
            }
        }
        else
        {
            if (currentCell->isRobotDistComputed())
            {
                currentDist = currentCell->getRobotDist();
            }
            else
            {
                currentCell->setRobotDistIsComputed();
                currentCell->setRobAccessible(true);
            }
        }

        for (unsigned int i = 0; i < currentNeighbors.size(); i++)
        {
            EnvCell* neighCell = currentNeighbors.at(i);
            if (ishuman)
            {
                if (!neighCell->getClosed())
                {
                    double dist = currentDist + currentCell->computeDist(neighCell);
                    if (neighCell->getOpen())
                    {
                        if (dist > neighCell->getHumanDist())
                        {
                            dist = neighCell->getHumanDist();
                        }
                        else
                        {
                            vector<EnvCell*> traj = currentCell->getHumanTraj();
                            traj.push_back(neighCell);

                            vector<Vector2d,Eigen::aligned_allocator<Vector2d> > vectorTraj = currentCell->getHumanVectorTraj();
                            vectorTraj.push_back(neighCell->getCenter());

                            neighCell->setHumanTraj(traj);
                            neighCell->setHumanVectorTraj(vectorTraj);
                        }
                    }
                    else
                    {
                        vector<EnvCell*> traj = currentCell->getHumanTraj();
                        traj.push_back(neighCell);

                        vector<Vector2d,Eigen::aligned_allocator<Vector2d> > vectorTraj = currentCell->getHumanVectorTraj();
                        vectorTraj.push_back(neighCell->getCenter());

                        neighCell->setHumanTraj(traj);
                        neighCell->setHumanVectorTraj(vectorTraj);

                        neighCell->setOpen();
                        openCells.push_back(neighCell);
                    }
                    neighCell->setHumanDist(dist);
                    neighCell->setHumanDistIsComputed();
                    neighCell->setHumAccessible(true);
                    if (dist > m_humanMaxDist) {m_humanMaxDist = dist; }
                    neighCell->computeBestRobotPos();
                    neighCell->computeCost();

                }
            }
            else
            {
                if (!neighCell->getClosed())
                {
                    double dist = currentDist + currentCell->computeDist(neighCell);
                    if (neighCell->getOpen())
                    {
                        if (dist > neighCell->getRobotDist())
                        {
                            dist = neighCell->getRobotDist();
                        }
                        else
                        {
                            vector<EnvCell*> traj = currentCell->getRobotTraj();
                            traj.push_back(neighCell);

                            vector<Vector2d,Eigen::aligned_allocator<Vector2d> > vectorTraj = currentCell->getRobotVectorTraj();
                            vectorTraj.push_back(neighCell->getCenter());

                            neighCell->setRobotTraj(traj);
                            neighCell->setRobotVectorTraj(vectorTraj);
                        }
                    }
                    else
                    {
                        vector<EnvCell*> traj = currentCell->getRobotTraj();
                        traj.push_back(neighCell);

                        vector<Vector2d,Eigen::aligned_allocator<Vector2d> > vectorTraj = currentCell->getRobotVectorTraj();
                        vectorTraj.push_back(neighCell->getCenter());

                        neighCell->setRobotTraj(traj);
                        neighCell->setRobotVectorTraj(vectorTraj);

                        neighCell->setOpen();
                        openCells.push_back(neighCell);
                    }
                    neighCell->setRobotDist(dist);
                    neighCell->setRobotDistIsComputed();
                    neighCell->setRobAccessible(true);
                    if (dist > m_robotMaxDist) {m_robotMaxDist = dist; }

                }
            }
        }
        currentCell->setClosed();
        openCells.erase(openCells.begin());
    }

}

void EnvGrid::dumpVar()
{

    cout << "######### grid variables #########" << endl;
    cout << "CellSize = \n" << _cellSize << endl;
    cout << "nb of cells = " << _cells.size() << endl;
    cout << "humanMaxDist in the grid = " << m_humanMaxDist<< endl;
    cout << "robotMaxDist in the grid = " << m_robotMaxDist << endl;
    cout << "nb of human accessible cells = " << m_HumanAccessible.size() << endl;
    cout << "nb of robot accessible cells = " << m_RobotAccessible.size() << endl;
    double mean=0;
    double max = 0;
    double min = numeric_limits<double>::max();
    for (unsigned int i = 0; i < m_HumanAccessible.size(); i++)
    {
        EnvCell* cell = m_HumanAccessible.at(i);
        int crownSize = cell->getCurrentHumanRobotReacheable().size();
        mean+= crownSize;
        if (crownSize > max)
        {
            max = crownSize;
        }
        else if (crownSize < min)
        {
            min = crownSize;
        }
    }
    cout << "mean of crown cells = " << mean/m_HumanAccessible.size()<< endl;
    cout << "min of crown cells = " << min << endl;
    cout << "max of crown cells = " << max << endl;
    cout << "##################################" << endl;
}

//---------------------------------------------------------------------------
// Cell
//---------------------------------------------------------------------------
EnvCell::EnvCell() :
        _Open(false),
        _Closed(false),
        mCostIsComputed(false),
        mCost(0.0),
        m_reachComputed(false),
        m_humanDistIsComputed(false),
        m_robotDistIsComputed(false)
{
}

EnvCell::EnvCell(int i, Vector2i coord, Vector2d corner, EnvGrid* grid) :
        API::TwoDCell(i,corner,grid),
        _Coord(coord),
        _Open(false),
        _Closed(false),
        mCostIsComputed(false),
        mCost(0.0),
        m_reachComputed(false),
        m_humanDistIsComputed(false),
        m_robotDistIsComputed(false)
{
}


void EnvCell::computeHumanReach()
{

    Robot* humCyl = dynamic_cast<EnvGrid*>(_grid)->getHumanCylinder();

    //human detection of collision
    Robot* human = dynamic_cast<EnvGrid*>(_grid)->getHuman();
    shared_ptr<Configuration> q_human_cur = human->getCurrentPos();
    shared_ptr<Configuration> q_human = human->getCurrentPos();
    (*q_human)[6] = 0;
    (*q_human)[7] = 0;
    human->setAndUpdate(*q_human);

    shared_ptr<Configuration> q = humCyl->getCurrentPos();
    (*q)[6] = this->getCenter()[0];
    (*q)[7] = this->getCenter()[1];
    humCyl->setAndUpdate(*q);

    m_isHumanAccessible = true;
    if (humCyl->isInCollisionWithOthersAndEnv())
    {
        m_isHumanAccessible = false;
    }
    (*q)[6] = 0;
    (*q)[7] = 0;
    humCyl->setAndUpdate(*q);
    human->setAndUpdate(*q_human_cur);



}

void EnvCell::computeRobotReach()
{
//    Robot* humCyl = dynamic_cast<EnvGrid*>(_grid)->getHumanCylinder();
    Robot* robotCyl = dynamic_cast<EnvGrid*>(_grid)->getRobotCylinder();

    //robot detection of collision
    Robot* rob = dynamic_cast<EnvGrid*>(_grid)->getRobot();
    shared_ptr<Configuration> q_rob_cur = rob->getCurrentPos();
    shared_ptr<Configuration> q_rob = rob->getCurrentPos();
    (*q_rob)[6] = 0;
    (*q_rob)[7] = 0;
    rob->setAndUpdate(*q_rob);

    shared_ptr<Configuration> q = robotCyl->getCurrentPos();
    (*q)[6] = this->getCenter()[0];
    (*q)[7] = this->getCenter()[1];
    robotCyl->setAndUpdate(*q);

    m_isRobotAccessible = true;
    if (robotCyl->isInCollisionWithOthersAndEnv())
    {
        m_isRobotAccessible = false;
    }
    (*q)[6] = 0;
    (*q)[7] = 0;
    robotCyl->setAndUpdate(*q);
    rob->setAndUpdate(*q_rob_cur);


}

double EnvCell::computeCost()
{
    double robotSpeed =  PlanEnv->getDouble(PlanParam::env_robotSpeed);//1;
    double humanSpeed = PlanEnv->getDouble(PlanParam::env_humanSpeed);//1;


    double timeStamp = PlanEnv->getDouble(PlanParam::env_timeStamp);//0.1;
    double objectNecessity = PlanEnv->getDouble(PlanParam::env_objectNessecity);

    double ksi = PlanEnv->getDouble(PlanParam::env_ksi);//0.5;
    double rho = PlanEnv->getDouble(PlanParam::env_rho);//0.5;

    double cost = numeric_limits<double>::max();
    if (getCurrentHumanRobotReacheable().size() > 0 && isHumAccessible())
    {
//                p.first = max((m_HumanAccessible.at(i)->getRobotBestPos().first / m_robotMaxDist)*PlanEnv->getDouble(PlanParam::env_objectNessecity)
//                               ,(m_HumanAccessible.at(i)->getHumanDist()/m_humanMaxDist));
        double hTime = getHumanDist()/humanSpeed;
        double rTime = getRobotBestPos().first/robotSpeed;
        double tempCost = max(hTime,rTime) * timeStamp;

        double mvCost = getHumanDist();
        cost = (ksi * mvCost) * (1 - objectNecessity)/(rho + ksi) + tempCost * objectNecessity;
    }
    mCost = cost;

    return mCost;

}

bool EnvCell::computeBestRobotPos()
{
    if (getCurrentHumanRobotReacheable().empty())
    {
        return false;
    }
    double index = 0;
    double dist = numeric_limits<double>::max();
    for (unsigned int i =0; i < getCurrentHumanRobotReacheable().size(); i++)
    {
        double tmp = getCurrentHumanRobotReacheable().at(i)->getRobotDist();
        if (tmp < dist)
        {
            dist = tmp;
            index = i;
        }
    }
    std::pair<double,EnvCell*> p;
    p.first = dist;
    p.second = getCurrentHumanRobotReacheable().at(index);
    setRobotBestPos(p);
    return true;
}

std::vector<EnvCell*> EnvCell::getNeighbors(bool isHuman)
{
    std::vector<EnvCell*> neighbors;
    Vector2i coord = getCoord();
    double nbCellX = dynamic_cast<EnvGrid*>(_grid)->getNbCellX();
    double nbCellY = dynamic_cast<EnvGrid*>(_grid)->getNbCellY();
//    EnvCell* cell =dynamic_cast<EnvCell*>(dynamic_cast<EnvGrid*>(_grid)->getCell(coord[0]+1,coord[1]));
    for (int i = -1; i <= 1; i++)
    {
        for (int j = -1; j <= 1; j++)
        {
            if (i != 0 || j != 0)
            {

                if (coord[0]+i < nbCellX && coord[0]+i >= 0 && coord[1]+j < nbCellY && coord[1]+j >= 0)
                {
                    EnvCell* cell = dynamic_cast<EnvCell*>(dynamic_cast<EnvGrid*>(_grid)->getCell(coord[0]+i,coord[1]+j));
//                    bool reachable = isHuman?cell->isHumAccessible():cell->isRobAccessible();
                    if (isHuman &&  cell->isHumAccessible())
                    {
                        neighbors.push_back(cell);
                    }
                    if (!isHuman && cell->isRobAccessible())
                    {
                        neighbors.push_back(cell);
                    }
                }
            }
        }
    }

    return neighbors;
}

double EnvCell::computeDist(EnvCell* neighCell)
{
    return sqrt(pow(getCenter()[0] - neighCell->getCenter()[0] , 2) + pow(getCenter()[1] - neighCell->getCenter()[1] , 2));
}

void EnvCell::resetReacheability()
{
    m_reachComputed = false;
    m_humanDistIsComputed = false;
    m_robotDistIsComputed = false;
    humanTraj.clear();
    humanVectorTraj.clear();
    robotTraj.clear();
    robotVectorTraj.clear();
    randomVectorPoint.clear();
}

void EnvCell::resetTraj()
{
    m_humanDistIsComputed = false;
    m_robotDistIsComputed = false;
    humanTraj.clear();
    humanVectorTraj.clear();
    robotTraj.clear();
    robotVectorTraj.clear();
}

void EnvCell::addPoint(double Rz)
{
    randomVectorPoint.push_back(Rz);
}


std::vector<EnvCell*> EnvCell::getCrown(double min, double max)
{
    vector<EnvCell*> crownCells;
    double idMaxDist = max/getCellSize()[0];
//    double idMinDist = min/(sqrt(2)* getCellSize()[0]);

    double maxInitX = getCoord()[0] - idMaxDist;
    if(maxInitX < 0)
    {
        maxInitX = 0;
    }

    double maxInitY = getCoord()[1] - idMaxDist;
    if(maxInitY < 0)
    {
        maxInitY = 0;
    }

    double maxEndX = getCoord()[0] + idMaxDist;
    if (maxEndX >= dynamic_cast<EnvGrid*>(_grid)->getNbCellX())
    {
        maxEndX = dynamic_cast<EnvGrid*>(_grid)->getNbCellX()-1;
    }

    double maxEndY = getCoord()[1] + idMaxDist;
    if (maxEndY >= dynamic_cast<EnvGrid*>(_grid)->getNbCellY()-1)
    {
        maxEndY = dynamic_cast<EnvGrid*>(_grid)->getNbCellY()-1;
    }

//    double minInit = getCenter()[0] - idMinDist;
//    if(minInit < 0)
//    {
//        minInit = 0;
//    }
//
//    double minEnd = getCenter()[0] + idMinDist;
//    if(minEnd > dynamic_cast<EnvGrid*>(_grid)->_nbCellsX)
//    {
//        minEnd = dynamic_cast<EnvGrid*>(_grid)->_nbCellsX;
//    }
//


    initHumanRobotReacheable.clear();
    crownCells.clear();
    for (int x = maxInitX; x <= maxEndX ; x++)
    {
        for (int y = maxInitY; y <= maxEndY  ; y ++)
        {
            EnvCell* cell = dynamic_cast<EnvCell*>(dynamic_cast<EnvGrid*>(_grid)->getCell(x,y));
            double dist = sqrt( pow( getCenter()[0] - cell->getCenter()[0] , 2) +
                                pow( getCenter()[1] - cell->getCenter()[1] , 2) );
            if (fabs(dist) <= max && fabs(dist) >= min && cell->isRobAccessible())
            {
                crownCells.push_back(cell);
                initHumanRobotReacheable.push_back(cell);
            }
        }
    }
//    cout << "crownCells.size() = " << crownCells.size() <<endl;
    if (crownCells.size() == 0)
    {
        m_isHumanAccessible = false;
    }

    return crownCells;
}











