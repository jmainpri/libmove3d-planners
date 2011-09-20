#include "HRICS_EnvGrid.hpp"

using namespace std;
using namespace tr1;
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

//---------------------------------------------------------------------------
// Grid
//---------------------------------------------------------------------------
EnvGrid::EnvGrid() :
        API::TwoDGrid(),
        mRobot(0x00),
        mHuman(0x00),
        _isHumanCentered(false),
        m_humanMaxDist(1),
        m_robotMaxDist(1)
{
}

EnvGrid::EnvGrid(double pace, vector<double> envSize, bool isHumanCentered) :
        API::TwoDGrid(pace,envSize),
        mRobot(0x00),
        mHuman(0x00),
        _isHumanCentered(isHumanCentered),
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
        _isHumanCentered(isHumanCentered),
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
    cout << "in: EnvGrid::init(pair<double,double> minMax)\n" << endl;
    cout << "Store position of all robot in the scene (object too); and move them out of the scene" << endl;
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
        Robot* r = new Robot(XYZ_ENV->robot[i]);
        pair<Robot*,shared_ptr<Configuration> > p;
        p.first = r;
        shared_ptr<Configuration> q = r->getCurrentPos();
        p.second = r->getCurrentPos();
        initConfiguration.push_back(p);

        (*q)[6] = -3;
        r->setAndUpdate(*q);
    }

    if (!humCyl || !robotCyl)
    {
        cout << "No cylinder to make reacheability tests. The Otp may segfault." << endl;
        return;
    }

    cout << "All Robot moved with success\n" << endl;
    cout << "Compute reacheability of each cell by the robot and by the human" << endl;
    initAllReachability();
    for (unsigned int x =0;x<_nbCellsX;++x)
    {
        for (unsigned int y =0;y<_nbCellsY;++y)
        {
            EnvCell* Cell = dynamic_cast<EnvCell*>(getCell(x,y));

            Cell->computeReach();
            if (Cell->isHumAccessible())
            {
                m_HumanAccessible.push_back(Cell);
            }
            if (Cell->isRobAccessible())
            {
                m_RobotAccessible.push_back(Cell);
            }
        }
    }

    cout << "Reacheability computed\n" << endl;

    cout << "find in which cell the robot can be placed accordingly to each cell where the human can be" << endl;
    computeHumanRobotReacheability(minMax);
    cout << "Robot cells list found\n" << endl;


    cout << "Replace all robot to there original place (object too)" << endl;
    for(unsigned int i=0;i < initConfiguration.size(); i++)
    {
        initConfiguration.at(i).first->setAndUpdate(*initConfiguration.at(i).second);
    }
    cout << "All robot are replaced\n" << endl;

    cout << "Move the cylinder out of the scenne" << endl;
    shared_ptr<Configuration> q = humCyl->getCurrentPos();
    (*q)[6] = -3;
    (*q)[7] = 1;
    humCyl->setAndUpdate(*q);

     q = robotCyl->getCurrentPos();
    (*q)[6] = -3;
    (*q)[7] = 1;
    robotCyl->setAndUpdate(*q);
    cout << "Cylinder has been moved\n" << endl;

    cout << "out of: EnvGrid::init(pair<double,double> minMax)\n\n" << endl;
}

void EnvGrid::initGrid()
{
    cout << "in: void EnvGrid::initGrid()\n" << endl;
    cout << "Compute cells reacheability for both human and robot with the obstacles" << endl;
    initAllReachability();
    for (unsigned int i = 0; i < m_HumanAccessible.size(); i++)
    {
        m_HumanAccessible.at(i)->computeReach();
    }

    for (unsigned int i = 0; i < m_RobotAccessible.size(); i++)
    {
        m_RobotAccessible.at(i)->computeReach();
    }

    cout << "End computing reacheability\n" << endl;

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

    cout << "Compute human distances" << endl;
    initAllCellState();

    shared_ptr<Configuration> q_human_cur = mHuman->getCurrentPos();

    int firstIndexOfHumanDof = mHuman->getJoint("Pelvis")->getIndexOfFirstDof();
    Vector2d pos;
    pos[0] = (*q_human_cur)[firstIndexOfHumanDof + 0];
    pos[1] = (*q_human_cur)[firstIndexOfHumanDof + 1];

    EnvCell* cell = dynamic_cast<EnvCell*>(getCell(pos));
    computeDistances(cell, true);
    cout << "Human distances computed with sucess\n" << endl;

    cout << "Compute Robot distances" << endl;
    initAllCellState();

    shared_ptr<Configuration> q_robot_cur = mRobot->getCurrentPos();

    pos[0] = (*q_robot_cur)[firstIndexOfHumanDof + 0];
    pos[1] = (*q_robot_cur)[firstIndexOfHumanDof + 1];

    cell = dynamic_cast<EnvCell*>(getCell(pos));
    computeDistances(cell, false);
    cout << "Robot distances computed with success\n" << endl;

    cout << "Update Reacheability in function of the computed distances, and compute if necessary the angle of human arrival" << endl;
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
    cout << "End updating\n" << endl;

    cout << "Update Human robot reacheability while the obstacles is in the scenne" << endl;
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
                if (cell->getHumanRobotReacheable().at(j)->isRobAccessible())
                {
                    newVect.push_back(cell->getHumanRobotReacheable().at(j));
                    if (cell->getHumanRobotReacheable().at(j)->getRobotDist() < p.first)
                    {
                        p.first = cell->getHumanRobotReacheable().at(j)->getRobotDist();
                        p.second = cell->getHumanRobotReacheable().at(j);
                    }

                }
            }
    //        p.first = max((p.second->getRobotDist()/ m_robotMaxDist)*PlanEnv->getDouble(PlanParam::env_objectNessecity) ,(cell->getHumanDist()/m_humanMaxDist));
    //        p.first = 0;
            cell->clearCurrentHumanRobotReacheable();
            cell->setCurrentHumanRobotReacheable(newVect);
            cell->setRobotBestPos(p);
        }
    }
    cout << "End updating\n" << endl;

    cout << "out of: void EnvGrid::initGrid()\n" << endl;
}



void EnvGrid::computeHumanRobotReacheability(std::pair<double,double> minMax)
{
    int k = 0;
    for (unsigned int i = 0; i < m_HumanAccessible.size(); i++)
    {
        for (unsigned int j = 0; j < m_RobotAccessible.size(); j++)
        {
            double dist = sqrt( pow( m_RobotAccessible.at(j)->getCenter()[0] - m_HumanAccessible.at(i)->getCenter()[0] , 2) +
                                pow( m_RobotAccessible.at(j)->getCenter()[1] - m_HumanAccessible.at(i)->getCenter()[1] , 2) );
            if (dist > minMax.first && dist < minMax.second)
            {
                m_HumanAccessible.at(i)->addToHumanRobotReacheable(m_RobotAccessible.at(j));
                k++;
            }
        }
    }
    cout << k << " cells has been computed." << endl;
}


class CellDistComp
{
public:

	bool operator()(pair<double,EnvCell*> first, pair<double,EnvCell*> second)
	{
		return ( first.first < second.first );
	}

} CellDistCompObject;

std::vector<std::pair<double,EnvCell*> > EnvGrid::getSortedGrid()
{
    double robotSpeed =  PlanEnv->getDouble(PlanParam::env_robotSpeed);//1;
    double humanSpeed = PlanEnv->getDouble(PlanParam::env_humanSpeed);//1;


    double timeStamp = PlanEnv->getDouble(PlanParam::env_timeStamp);//0.1;
    double objectNecessity = PlanEnv->getDouble(PlanParam::env_objectNessecity);

    double ksi = PlanEnv->getDouble(PlanParam::env_ksi);//0.5;
    double rho = PlanEnv->getDouble(PlanParam::env_rho);//0.5;

    if (!gridIsSorted)
    {
        std::vector<std::pair<double,EnvCell*> > vect;
        for (unsigned int i = 0; i < m_HumanAccessible.size() ; i++)
        {
            pair<double,EnvCell*> p;
            p.second = m_HumanAccessible.at(i);

//	    cout << "p.second->getCurrentHumanRobotReacheable().size() = " << p.second->getCurrentHumanRobotReacheable().size() << endl;
//	    cout << "p.second->isHumAccessible() = "  << p.second->isHumAccessible() << endl;

            if (p.second->getCurrentHumanRobotReacheable().size() > 0 && p.second->isHumAccessible())
            {
//                p.first = max((m_HumanAccessible.at(i)->getRobotBestPos().first / m_robotMaxDist)*PlanEnv->getDouble(PlanParam::env_objectNessecity)
//                               ,(m_HumanAccessible.at(i)->getHumanDist()/m_humanMaxDist));
                double hTime = m_HumanAccessible.at(i)->getHumanDist()/humanSpeed;
                double rTime = m_HumanAccessible.at(i)->getRobotBestPos().first/robotSpeed;
                double tempCost = max(hTime,rTime) * timeStamp;

                double mvCost = m_HumanAccessible.at(i)->getHumanDist();
                p.first = (ksi * mvCost) * (1 - objectNecessity)/(rho + ksi) + tempCost * objectNecessity;
                vect.push_back(p);
            }
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

/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
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
    //    Vector2d corner = newCell->getCorner();
    //    cout << " = (" << corner[0] <<"," << corner[1] << ")" << endl;
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

//    double depth = 0.0;

    //    cout << "Drawing 2D Grid"  << endl;

    double color[3];

    //    double nbCells = static_cast<double>(getNumberOfCells());

    for (unsigned int x =0;x<_nbCellsX;++x)
    {
        for (unsigned int y =0;y<_nbCellsY;++y)
        {

            EnvCell* Cell = dynamic_cast<EnvCell*>(getCell(x,y));

//            double colorRation = ENV.getDouble(Env::colorThreshold1)-(Cell->getCost()/(ENV.getDouble(Env::Kdistance)+ENV.getDouble(Env::Kvisibility)));
//            Cell->computeReach();

//            double colorRation = Cell->getCost();//*360;
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
                isToDraw = Cell->isHumAccessible();
//                colorRation = Cell->getRobotBestPos().first;
                colorRation = max((Cell->getRobotBestPos().first / m_robotMaxDist)*PlanEnv->getDouble(PlanParam::env_objectNessecity) ,(Cell->getHumanDist()/m_humanMaxDist)) ;
            }

//            if (PlanEnv->getBool(PlanParam::env_robotGridDraw) && Cell->isRobAccessible() && Cell->isHumAccessible())
//            {
//                isToDraw = Cell->isRobAccessible() && Cell->isHumAccessible();
//                colorRation = max((Cell->getRobotDist() / m_robotMaxDist)*PlanEnv->getDouble(PlanParam::env_objectNessecity) ,(Cell->getHumanDist()/m_humanMaxDist)) ;
//            }

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

                    glColor3d(color[0],color[1],color[2]);
                    glVertex3d( (double)(center[0] - _cellSize[0]/2) + 0.01 , (double)(center[1] - _cellSize[1]/2) + 0.01, colorRation);//depth );
                    glVertex3d( (double)(center[0] + _cellSize[0]/2) - 0.01 , (double)(center[1] - _cellSize[1]/2) + 0.01, colorRation);//depth );
                    glVertex3d( (double)(center[0] + _cellSize[0]/2) - 0.01 , (double)(center[1] + _cellSize[1]/2) - 0.01, colorRation);//depth );
                    glVertex3d( (double)(center[0] - _cellSize[0]/2) + 0.01 , (double)(center[1] + _cellSize[1]/2) - 0.01, colorRation);//depth );
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
                            height = 0.15;
                            int distColor = Orange;
                            if (Cell->getRobotBestPos().second->getIndex() == Cell->getCurrentHumanRobotReacheable().at(i)->getIndex())
                            {
                                height = 0.2 ;
                                distColor = Red;
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

                        for(unsigned int i=0;i<Cell->getHumanTraj().size()-1;i++)
                        {
                            glLineWidth(3.);
                            g3d_drawOneLine(Cell->getHumanTraj().at(i)->getCenter()[0],      Cell->getHumanTraj().at(i)->getCenter()[1],      0.4,
                                            Cell->getHumanTraj().at(i + 1)->getCenter()[0],  Cell->getHumanTraj().at(i + 1)->getCenter()[1],  0.4,
                                            Green, NULL);
                            glLineWidth(1.);
                        }
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
/**
 * call setBlankCost() in each cell
 */
void EnvGrid::setCellsToblankCost()
{
    for (int i = 0; i < getNumberOfCells(); i++)
    {
        dynamic_cast<EnvCell*>(this->getCell(i))->setBlankCost();
    }

}

/**
 * call resetexplorationstatus() in each cell
 */
void EnvGrid::initAllCellState()
{
    for (int i = 0; i < getNumberOfCells(); i++)
    {
        dynamic_cast<EnvCell*>(this->getCell(i))->resetExplorationStatus();
    }
}

/**
 * call resetexplorationstatus() in each cell
 */
void EnvGrid::initAllReachability()
{
    for (int i = 0; i < getNumberOfCells(); i++)
    {
        dynamic_cast<EnvCell*>(this->getCell(i))->resetReacheability();

    }
    m_humanMaxDist = 0;
    m_robotMaxDist = 0;
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
                    if (dist > m_humanMaxDist) {m_humanMaxDist = dist; }

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
                    if (dist > m_robotMaxDist) {m_robotMaxDist = dist; }

                }
            }
        }
        currentCell->setClosed();
        openCells.erase(openCells.begin());
    }

}



class CellComp
{
public:

	bool operator()(EnvCell* first, EnvCell* second)
	{
		return ( first->getCost() < second->getCost() );
	}

} CellCompObject;

vector<EnvCell*> EnvGrid::getSortedCells()
{
    vector<EnvCell*> cells;
    for (unsigned int i=0; i < _cells.size(); i++)
    {
        cells.push_back(dynamic_cast<EnvCell*>(_cells.at(i)));
    }

    sort(cells.begin(), cells.end(),CellCompObject);
    return cells;
}

void EnvGrid::recomputeCostRobotOnly()
{
    ENV.setDouble(Env::optimalDistFactor,0);
    ENV.setDouble(Env::robotMaximalDistFactor,1);
    ENV.setDouble(Env::gazeAngleFactor,0);
    ENV.setDouble(Env::robotMaximalDist,6);

    setCellsToblankCost();
}

//---------------------------------------------------------------------------
// Cell
//---------------------------------------------------------------------------
EnvCell::EnvCell() :
        _Open(false),
        _Closed(false),
        mCostIsComputed(false),
        mCost(0.0),
        isNotAccessible(false),
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
        isNotAccessible(false),
        m_reachComputed(false),
        m_humanDistIsComputed(false),
        m_robotDistIsComputed(false)
{
}

double EnvCell::getCost()
{
    if(mCostIsComputed)
    {
        return mCost;
    }

    if (!dynamic_cast<EnvGrid*>(_grid)->isHumanCentered())
    {

//        Robot* humCyl;
//        for (int i=0; i<XYZ_ENV->nr; i++)
//        {
//            string name(XYZ_ENV->robot[i]->name);
//            if(name.find("PR_2CYLINDER") != string::npos )
//            {
//                humCyl = new Robot(XYZ_ENV->robot[i]);
//            }
//        }
        Robot* human = dynamic_cast<EnvGrid*>(_grid)->getHuman();
        shared_ptr<Configuration> q_human_cur = human->getCurrentPos();
        shared_ptr<Configuration> q_human = human->getCurrentPos();
        (*q_human)[6] = 10;
        (*q_human)[7] = 1;
        human->setAndUpdate(*q_human);


        Robot* rob = dynamic_cast<EnvGrid*>(_grid)->getRobot();
        mCost=0;
        shared_ptr<Configuration> q_cur = rob->getCurrentPos();

        shared_ptr<Configuration> q_tmp = rob->getCurrentPos();
        int firstIndexOfDof = dynamic_cast<p3d_jnt*>(rob->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

        (*q_tmp)[firstIndexOfDof + 0] = this->getCenter()[0];
        (*q_tmp)[firstIndexOfDof + 1] = this->getCenter()[1];

        rob->setAndUpdate(*q_tmp);

//        shared_ptr<Configuration> q = humCyl->getCurrentPos();
//        (*q)[6] = this->getCenter()[0];
//        (*q)[7] = this->getCenter()[1];
//        humCyl->setAndUpdate(*q);

        mCost=0.1;
        if (rob->isInCollision())
        {
            mCost=1.0;
            isNotAccessible = true;
        }

        rob->setAndUpdate(*q_cur);
        mCostIsComputed = true;

        human->setAndUpdate(*q_human_cur);
        return mCost;
    }
    else if (false) // using another grid actually
    {
        double optimalDist = ENV.getDouble(Env::optimalDist);

        double robotDistanceMax = ENV.getDouble(Env::robotMaximalDist);

        Robot* r =dynamic_cast<EnvGrid*>(_grid)->getHuman();
      if ( (r->getName().find( "ACHILE") != string::npos) ||
          (r->getName().find( "HERAKLES") != string::npos) )
        {
            int mIndexOfDoF = r->getJoint("Pelvis")->getIndexOfFirstDof();
            double x = (*r->getCurrentPos())[mIndexOfDoF + 0];
            double y = (*r->getCurrentPos())[mIndexOfDoF + 1];

            //the distance
            double distance = std::sqrt(std::pow(x - getCenter()[0],2) + std::pow((y - getCenter()[1]),2));

            double distanceParam = 0.0;
            if (distance < optimalDist)
            {
                distanceParam = 1 - std::exp(-(std::pow((distance - optimalDist)*2,2)));
            }
            else
            {
                distanceParam = (distance - optimalDist)/8.0;
            }
            if (distanceParam > 1){ distanceParam = 1.; }
            else if (distanceParam < 0){ distanceParam = 0.; }

            // get the rotation of the human
            double rot = (*r->getCurrentPos())[mIndexOfDoF + 5];
            double angle = atan2((getCenter()[1] - y ), getCenter()[0] - x);

            double fieldOfVision = 1.0;

            // the opening angle of the field of vision
            double cAngle = ENV.getDouble(Env::gazeAngle)*M_PI/180.0 ;
            if ((angle <= cAngle + rot && angle >= 0 + rot) || (cAngle + rot >= M_PI && angle <= cAngle + rot - 2*M_PI))
            {
                fieldOfVision = tan(angle - rot) / tan(cAngle);
            }
            else if ((angle > -cAngle + rot && angle < 0 + rot) || (-cAngle + rot < -M_PI && angle > -cAngle + rot + 2*M_PI))
            {
                fieldOfVision = -tan(angle - rot ) / tan(cAngle);
            }

            Robot* r2 = dynamic_cast<EnvGrid*>(_grid)->getRobot();

            double RobotPreference = 1.0;
            if (r2->getName().find( "JIDO") != string::npos ||r2->getName().find( "PR2") != string::npos)
            {
                // distance between robot (jido) and the point
                double distance2 = std::sqrt(std::pow((*r2->getCurrentPos())[mIndexOfDoF + 0] - getCenter()[0],2) + std::pow((*r2->getCurrentPos())[mIndexOfDoF + 1] - getCenter()[1],2));
                RobotPreference = distance2/robotDistanceMax;
                if (RobotPreference > 1.0)
                {
                    RobotPreference = 1.0;
                }
            }


              int type =  0; //ENV.getInt(Env::typeRobotBaseGrid);
              if (type == 0)
              {
                  double a = ENV.getDouble(Env::optimalDistFactor);
                  double b = ENV.getDouble(Env::robotMaximalDistFactor);
                  double c = ENV.getDouble(Env::gazeAngleFactor);
                  double d = a + b + c;
                  a = a / d;
                  b = b / d;
                  c = c / d;

                  mCost = 1 - (a * ( 1 - distanceParam) + b * (1 - RobotPreference) + c * (1 - fieldOfVision));
                  // 1 - (weighted sum of : human ditance distanceParam , robot distance and field of vision)
              }

              else if (type == 1){ mCost = distanceParam; }
              else if (type == 2){ mCost = RobotPreference; }
              else if (type == 3){ mCost = fieldOfVision; }

        }
    }
    else
    {
        Robot* rob = dynamic_cast<EnvGrid*>(_grid)->getRobot();
        Robot* hum = dynamic_cast<EnvGrid*>(_grid)->getHuman();

        int mHumanIndexOrFirstDof = hum->getJoint("Pelvis")->getIndexOfFirstDof();
        int mRobotIndexOrFirstDof =  dynamic_cast<p3d_jnt*>(rob->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

        shared_ptr<Configuration> q_human_cur = hum->getCurrentPos();
        shared_ptr<Configuration> q_robot_cur = rob->getCurrentPos();

        double humanRobotDistance = std::sqrt( std::pow((*q_human_cur)[mHumanIndexOrFirstDof + 0] - (*q_robot_cur)[mRobotIndexOrFirstDof + 0] ,2 ) +
                                               std::pow((*q_human_cur)[mHumanIndexOrFirstDof + 1] - (*q_robot_cur)[mRobotIndexOrFirstDof + 1] ,2 ) );
        humanRobotDistance /= 2;

        double robotDistToCell = std::sqrt( std::pow(getCenter()[0] - (*q_robot_cur)[mRobotIndexOrFirstDof + 0] ,2 ) +
                                            std::pow(getCenter()[1] - (*q_robot_cur)[mRobotIndexOrFirstDof + 1] ,2 ) );
//        if (robotDistToCell/humanRobotDistance > 1) {robotDistToCell = humanRobotDistance;}

        double humanDistToCell = std::sqrt( std::pow(getCenter()[0] - (*q_human_cur)[mHumanIndexOrFirstDof + 0] ,2 ) +
                                            std::pow(getCenter()[1] - (*q_human_cur)[mHumanIndexOrFirstDof + 1] ,2 ) );

//        if (humanDistToCell/humanRobotDistance > 1) {humanDistToCell = humanRobotDistance;}

        if (humanDistToCell > robotDistToCell)
        {
            mCost = robotDistToCell/humanRobotDistance;
        }
        else
        {
             mCost = humanDistToCell/humanRobotDistance;
        }

//        mCost = 0.5 * robotDistToCell/humanRobotDistance + 0.5 * humanDistToCell/humanRobotDistance;

    }

    mCostIsComputed = true;


    return mCost;

}


void EnvCell::computeReach()
{
//    cout << "begin computeReach nb : " << _index << endl;
    if (m_reachComputed)
    {
        return;
    }
    Robot* humCyl = dynamic_cast<EnvGrid*>(_grid)->getHumanCylinder();
    Robot* robotCyl = dynamic_cast<EnvGrid*>(_grid)->getRobotCylinder();


    //human detection of collision
    Robot* human = dynamic_cast<EnvGrid*>(_grid)->getHuman();
    shared_ptr<Configuration> q_human_cur = human->getCurrentPos();
    shared_ptr<Configuration> q_human = human->getCurrentPos();
    (*q_human)[6] = 10;
    (*q_human)[7] = 1;
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
    (*q)[6] = 10;
    (*q)[7] = 1;
    humCyl->setAndUpdate(*q);
    human->setAndUpdate(*q_human_cur);

    //robot detection of collision
    Robot* rob = dynamic_cast<EnvGrid*>(_grid)->getRobot();
    shared_ptr<Configuration> q_rob_cur = rob->getCurrentPos();
    shared_ptr<Configuration> q_rob = rob->getCurrentPos();
    (*q_rob)[6] = 10;
    (*q_rob)[7] = 1;
    rob->setAndUpdate(*q_rob);

    q = robotCyl->getCurrentPos();
    (*q)[6] = this->getCenter()[0];
    (*q)[7] = this->getCenter()[1];
    robotCyl->setAndUpdate(*q);

    m_isRobotAccessible = true;
    if (robotCyl->isInCollisionWithOthersAndEnv())
    {
        m_isRobotAccessible = false;
    }
    (*q)[6] = 10;
    (*q)[7] = 1;
    robotCyl->setAndUpdate(*q);
    rob->setAndUpdate(*q_rob_cur);

    m_reachComputed = true;

//    cout << "end computeReach" << endl;
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

void EnvCell::addPoint(double Rz)
{
    randomVectorPoint.push_back(Rz);
}


//---------------------------------------------------------------------------
// State
//---------------------------------------------------------------------------
EnvState::EnvState( Vector2i cell , EnvGrid* grid) :
        _Grid(grid)
{
    _Cell = dynamic_cast<EnvCell*>(grid->getCell(cell));
}

EnvState::EnvState( EnvCell* cell , EnvGrid* grid) :
        _Grid(grid),
        _Cell(cell)
{

}


vector<API::State*> EnvState::getSuccessors()
{
    vector<API::State*> newStates;
    //    newStates.reserve(26);

    //    cout << "--------------------" << endl;
    for(int i=0;i<8;i++)
    {
        EnvCell* neigh = dynamic_cast<EnvCell*>(_Grid->getNeighbour( _Cell->getCoord(), i));
        if( neigh != NULL && !neigh->isNotAccessible)
        {
            //            _Grid->isVirtualObjectPathValid(dynamic_cast<EnvCell*>(_Cell),neigh);
            newStates.push_back( new EnvState(neigh,_Grid));
        }
    }

    return newStates;
}

bool EnvState::isLeaf()
{
    return false;
}

bool EnvState::equal(API::State* other)
{
//    bool equal(false);
    EnvState* state = dynamic_cast<EnvState*>(other);
    Vector2i pos = _Cell->getCoord();
    for(int i=0;i<2;i++)
    {
        if( pos[i] != state->_Cell->getCoord()[i] )
        {
            //            cout << "EnvState::equal false" << endl;
            return false;
        }
    }


    //    cout << "State::equal true" << endl;
    return true;
}

void EnvState::setClosed(std::vector<EnvState*>& closedStates,std::vector<EnvState*>& openStates)
{
    //    cout << "State :: set Closed" <<endl;
    _Cell->setClosed();
}

bool EnvState::isColsed(std::vector<EnvState*>& closedStates)
{
    //    cout << "State :: get Closed" <<endl;
    return _Cell->getClosed();
}

void EnvState::setOpen(std::vector<EnvState*>& openStates)
{
    //     cout << "State :: set open" <<endl;
    _Cell->setOpen();
}


bool EnvState::isOpen(std::vector<EnvState*>& openStates)
{
    //    cout << "State :: get open" <<endl;
    return _Cell->getOpen();
}

void EnvState::reset()
{
    _Cell->resetExplorationStatus();
}

void EnvState::print()
{

}

double EnvState::computeLength(API::State *parent)
{
    EnvState* preced = dynamic_cast<EnvState*>(parent);

    Vector2d pos1 = _Cell->getCenter();
    Vector2d pos2 = preced->_Cell->getCenter();

    double dist = ( pos1 - pos2 ).norm();

    //    double cost1 = preced->_Cell->getCost();
    double cost2 = _Cell->getCost();
    double g = preced->g() + /*cost1 +*/ cost2 * dist;

    //    cout << "dist = " << dist << endl;
    //    cout << "g = " << g << endl;
    return g;
}

double EnvState::computeHeuristic( API::State *parent, API::State* goal )
{
    EnvState* state = dynamic_cast<EnvState*>(goal);

    Vector2d posGoal = state->_Cell->getCenter();
    Vector2d posThis = _Cell->getCenter();

    double dist=0;

//    dist += ( posGoal - posThis ).norm();

    return dist;
}
