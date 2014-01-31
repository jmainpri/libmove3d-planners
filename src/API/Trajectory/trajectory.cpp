/*
 * trajectory.cpp
 *
 *  Created on: Jun 17, 2009
 *      Author: jmainpri
 */

#include "API/Trajectory/trajectory.hpp"

#include "planEnvironment.hpp"

#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Localpath-pkg.h"
#include "GroundHeight-pkg.h"
#include "Planner-pkg.h"

#include "cost_space.hpp"

#if defined( HRI_COSTSPACE ) && defined ( HRI_PLANNER )
#include "hri_costspace/HRICS_HAMP.hpp"
#endif

#include "move3d-headless.h"

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

using namespace API;

std::vector<Trajectory> global_trajToDraw;

Trajectory::Trajectory() :
    m_Robot(NULL),
    HighestCostId(0),
    isHighestCostIdSet(false),
    name(""),
    file(""),
    mColor(0),
    m_Source(new Configuration(m_Robot,NULL))
{

}

Trajectory::Trajectory(Robot* R) :
    m_Robot(R),
    HighestCostId(0),
    isHighestCostIdSet(false),
    name(""),
    file(""),
    mColor(0),
    m_Source(new Configuration(m_Robot,NULL))
{

}

Trajectory::Trajectory(const Trajectory& T) :
    m_Robot(T.m_Robot),
    HighestCostId(T.HighestCostId),
    isHighestCostIdSet(T.isHighestCostIdSet),
    name(T.name),
    file(T.file),
    mColor(T.mColor),
    m_Source(T.m_Source),
    m_Target(T.m_Target)
{

    for (uint i = 0; i < T.m_Courbe.size(); i++)
    {
        m_Courbe.push_back(new LocalPath(*(T.m_Courbe.at(i))));
    }
    //	cout << "Copy Trajectory" << endl;
}

Trajectory& Trajectory::operator=(const Trajectory& T)
{
    if (m_Courbe.size() > 0)
    {
        for (uint i = 0; i <m_Courbe.size(); i++)
        {
            delete m_Courbe[i];
        }
    }

    m_Courbe.clear();

    // TODO Name of file and robot
    name = T.name;
    file = T.file;

    m_Robot = T.m_Robot;

    for (uint i = 0; i <T.m_Courbe.size(); i++)
    {
        m_Courbe.push_back(new LocalPath(*(T.m_Courbe[i])));
    }

    //	cout << "Copy in operator= Trajtecory" << endl;

    m_Source = T.m_Source;
    m_Target = T.m_Target;
    mColor = T.mColor;
    HighestCostId = T.HighestCostId;
    isHighestCostIdSet = T.isHighestCostIdSet;

    return *this;
}

Trajectory::Trajectory( vector< shared_ptr<Configuration> >& configs) :
    HighestCostId(0),
    isHighestCostIdSet(false)
{
    if ( !configs.empty() )
    {
        name = "";
        file = "";

        m_Robot	= configs[0]->getRobot();

        m_Source = configs[0];
        m_Target = configs.back();

        m_Courbe.clear();

        for (unsigned int i=0; i<configs.size()-1; i++)
        {
            if ( !configs[i]->equal( *configs[i+1]) )
            {
                LocalPath* path = new LocalPath( configs[i], configs[i+1] );
                m_Courbe.push_back(path);
            }
            else
            {
                cout << "two configuration are the same in traj constructor" << endl;
            }
        }
    }
    else
    {
        cout
                << "Warning: Constructing a class out of empty vector of configuration"
                << endl;
    }
    mColor = 1;
}

Trajectory::Trajectory(Robot* R, p3d_traj* t)
{
    m_Courbe.clear();

    if ((t == NULL) || (t->courbePt==NULL))
    {
        return;
    }

    // TODO Name and file (string based)
    m_Robot = R;

    p3d_localpath* localpathPt = t->courbePt;

    while (localpathPt != NULL)
    {
        LocalPath* path = new LocalPath(m_Robot, localpathPt);
        //			path->getBegin()->print();
        //			path->getEnd()->print();
        m_Courbe.push_back(path);
        localpathPt = localpathPt->next_lp;
    }

    m_Source = confPtr_t (new Configuration(m_Robot,p3d_config_at_param_along_traj(t, 0)));
    m_Source->setConstraints();

    //	cout << "m_Source:" << endl;
    //	m_Source->print();

    m_Target = confPtr_t (new Configuration(m_Robot,p3d_config_at_param_along_traj(t, getRangeMax())));
    m_Target->setConstraints();
    //	cout << "m_Target:" << endl;
    //	m_Target->print();

    //	cout << "range_param = " << range_param << endl;
    mColor = 0;

    if (!getBegin()->equal(*configAtParam(0)))
    {
        cout << "Error in constructor : !getBegin()->equal(*configAtParam(0))" << endl;
    }

    if (!getEnd()->equal(*configAtParam(getRangeMax())))
    {
        cout << "------------------------------------------" << endl;
        cout << "Error in constructor : !getEnd()->equal(*configAtParam(getRangeMax()))" << endl;
        getEnd()->print();
        configAtParam(getRangeMax())->print();
    }
}

bool Trajectory::operator==( const Trajectory& t ) const
{
    if( m_Courbe.size() != t.m_Courbe.size())
    {
        return false;
    }

    if( getRangeMax() != t.getRangeMax())
    {
        return false;
    }

    for (int i=0; i< int(m_Courbe.size()); i++)
    {
        if( (*m_Courbe[i]->getBegin()) != (*t.m_Courbe[i]->getBegin()) )
        {
            return false;
        }
        if( (*m_Courbe[i]->getEnd()) != (*t.m_Courbe[i]->getEnd()) )
        {
            return false;
        }
        if( m_Courbe[i]->getParamMax() != t.m_Courbe[i]->getParamMax() )
        {
            return false;
        }
    }

    return true;
}

bool Trajectory::operator!=( const Trajectory& t ) const
{
    return !( *this == t );
}

Trajectory::~Trajectory()
{
    for (int i = 0; i < int(m_Courbe.size()); i++)
    {
        delete m_Courbe.at(i);
    }
}

bool Trajectory::replaceP3dTraj() const
{
    //cout << "Robot name : " << m_Robot->getRobotStruct()->name << endl;
    return replaceP3dTraj( p3d_get_robot_by_name( m_Robot->getName().c_str() )->tcur );
}

p3d_traj* Trajectory::replaceP3dTraj(p3d_traj* trajPt) const
{
    //	print();

    if(trajPt!=NULL)
    {
        if(strcmp(trajPt->rob->name,m_Robot->getRobotStruct()->name) != 0 )
        {
            cout << " Warning : Robot not the same as the robot in traj "  << endl;
            return NULL;
        }
        if( trajPt->courbePt != NULL )
        {
            destroy_list_localpath( m_Robot->getRobotStruct(), trajPt->courbePt);
        }
    }
    else
    {
        trajPt = p3d_create_empty_trajectory(m_Robot->getRobotStruct());
    }

    // trajPt->name = strdup(name);
    // trajPt->file = NULL;  // Modification Fabien
    trajPt->num = 0; //m_Robot->getRobotStruct()->nt;
    // trajPt->rob = m_Robot->getRobotStruct();

    p3d_localpath *localpathPt = NULL;
    p3d_localpath *localprevPt = NULL;

    bool first = true;

    for (int i=0; i<int(m_Courbe.size()); i++)
    {
        localprevPt = localpathPt;
        localpathPt = m_Courbe[i]->getLocalpathStruct()->copy( m_Robot->getRobotStruct(),
                                                               m_Courbe[i]->getLocalpathStruct() );

        if( localprevPt )
        {
            localprevPt->next_lp = localpathPt;
        }

        localpathPt->prev_lp = localprevPt;

        if ( first )
        {
            trajPt->courbePt = localpathPt;
            first = false;
        }
    }

    trajPt->range_param = p3d_compute_traj_rangeparam(trajPt);
    trajPt->nlp = m_Courbe.size();

    if (m_Courbe.size() != 0)
        localpathPt->next_lp = NULL;
    else
    {
        cout << "replaceP3dTraj with empty trajectory" << endl;
        trajPt->courbePt = NULL;
    }

    return trajPt;
    //	print()
}

void Trajectory::copyPaths( vector<LocalPath*>& vect )
{
    vector<LocalPath*>::iterator it;

    for ( it = vect.begin() ; it != vect.end() ; it++ )
    {
        *it = new LocalPath(**it);
    }
}


p3d_traj* Trajectory::replaceHumanP3dTraj(Robot*rob, p3d_traj* trajPt)
{
    //	print();

    //	Robot* rob =new Robot(p3d_get_robot_by_name(trajPt->rob->name));
    if(trajPt!=NULL)
    {
        destroy_list_localpath(rob->getRobotStruct(), trajPt->courbePt);
    }
    else
    {
        trajPt = p3d_create_empty_trajectory(rob->getRobotStruct());
    }

    //	trajPt->name       = strdup(name);
    //	trajPt->file       = NULL;  // Modification Fabien
    trajPt->num = 0; //rob->getRobotStruct()->nt;
    //    trajPt->rob = m_Robot->getRobotStruct();

    //	cout << rob->getRobotStruct() << endl;

    p3d_localpath *localpathPt = NULL;
    p3d_localpath *localprevPt = NULL;

    bool first = true;

    for (unsigned int i = 0; i < m_Courbe.size(); i++)
    {
        //    if ( *m_Courbe[i]->getBegin() ==  *m_Courbe[i]->getEnd() )
        //    {
        //      cout << "null LocalPath in replaceP3dTraj" << endl;
        //      continue;
        //    }

        localprevPt = localpathPt;
        localpathPt = m_Courbe[i]->getLocalpathStruct()->copy( rob->getRobotStruct(),
                                                               m_Courbe[i]->getLocalpathStruct() );

        if( localprevPt )
        {
            localprevPt->next_lp = localpathPt;
        }

        localpathPt->prev_lp = localprevPt;

        if ( first )
        {
            trajPt->courbePt = localpathPt;
            first = false;
        }
    }

    if (m_Courbe.size() != 0)
    {
        localpathPt->next_lp = NULL;
    }
    else
    {
        cout << "replaceP3dTraj with empty trajectory" << endl;
    }

    trajPt->nlp = m_Courbe.size();

#ifdef P3D_PLANNER
    trajPt->range_param = p3d_compute_traj_rangeparam(trajPt);
#else
    printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif

    return trajPt;
    //	print()
}

confPtr_t Trajectory::configAtParam(double param, unsigned int* id_localpath) const
{
    if(m_Courbe.empty()) {
        return confPtr_t(new Configuration(m_Robot,NULL));
    }

    double soFar(0.0);
    double prevSoFar(0.0);
    unsigned int i=0;

    for ( ; i<m_Courbe.size(); i++)
    {
        soFar = soFar + m_Courbe.at(i)->getParamMax();

        // Parameter lies in the previous local path
        // return configuration inside previous LP
        if ( param < soFar )
        {
            if ( param < prevSoFar )
            {
                cout
                        << "Error: getting Configuration at parameter in trajectory"
                        << endl;
            }

            if( id_localpath != NULL ) {
                *id_localpath = i;
            }

            return m_Courbe.at(i)->configAtParam( param - prevSoFar );
        }
        prevSoFar = soFar;
    }

    if( id_localpath != NULL ) {
        *id_localpath = m_Courbe.size()-1;
    }
    return m_Courbe.back()->configAtParam(param);
}

vector<confPtr_t> Trajectory::getVectorOfConfiguration() const
{
    vector<confPtr_t> vect;

    if (m_Courbe.empty()) {
        return vect;
    }

    vect.push_back( m_Courbe[0]->getBegin() );

    for (size_t i=0; i<m_Courbe.size(); i++)
        vect.push_back( m_Courbe[i]->getEnd() );

    return vect;
}
vector<confPtr_t> Trajectory::getNConfAtParam(double delta) const
{
    vector<confPtr_t> tmpVector(0);

    double param = 0;
    double soFar(0.0);
    double prevSoFar(0.0);

    for (size_t i=0; i<m_Courbe.size(); i++)
    {
        soFar = soFar + m_Courbe[i]->getParamMax();

        // Parameter lies in the previous local path
        // return configuration inside previous LP
        while (param < soFar)
        {
            if (param < prevSoFar)
            {
                cout
                        << "Error: getting Configuration at parameter in trajectory"
                        << endl;
            }

            tmpVector.push_back(m_Courbe.at(i)->configAtParam(param - prevSoFar));
            param += delta;
        }
        // TODO watch out
        if (i < m_Courbe.size() - 1)
        {
            prevSoFar = soFar;
        }
    }

    tmpVector.push_back(m_Courbe.at(m_Courbe.size() - 1)->configAtParam(param - prevSoFar));

    return tmpVector;
}

bool Trajectory::isEmpty()
{
    return m_Courbe.empty();
}

void Trajectory::clear()
{
    m_Source = confPtr_t(new Configuration(m_Robot,NULL));

    for (int i=0; i<int(m_Courbe.size()); i++)
    {
        delete m_Courbe.at(i);
    }

    m_Courbe.clear();
}

LocalPath* Trajectory::getLocalPath(unsigned int id) const
{
    return m_Courbe[id];
}

int Trajectory::getNbOfViaPoints() const
{
    if ( m_Courbe.empty() )
        return 0;

    return (m_Courbe.size()+1);
}

confPtr_t Trajectory::operator[]( const int &i ) const
{
    if( i<0 || m_Courbe.empty() || (i>int(m_Courbe.size())))
    {
        return confPtr_t(new Configuration(m_Robot));
    }

    if(i == int(m_Courbe.size()))
    {
        return m_Courbe[i-1]->getEnd();
    }

    return m_Courbe[i]->getBegin();
}

double Trajectory::computeSubPortionRange(const vector<LocalPath*>& portion) const
{
    double range(0.0);

    for (int i=0; i<int(portion.size()); i++)
    {
        // portion[i]->getBegin()->print();
        // portion[i]->getEnd()->print();

        range += portion[i]->getParamMax();
    }

    return range;
}

//void Trajectory::updateRange()
//{
//	nloc = m_Courbe.size();
//	range_param = computeSubPortionRange(m_Courbe);
//}

bool Trajectory::isValid() const
{
    for (int i=0; i<int(m_Courbe.size()); i++)
    {
        if (!m_Courbe[i]->isValid())
        {
            return false;
        }
        // cout <<"LocalPath["<<i<<"] = "<< m_Courbe[i]->getNbColTest() << endl;
    }

    return true;
}

void Trajectory::resetIsValid()
{
    for (int i=0; i<int(m_Courbe.size()); i++)
    {
        m_Courbe[i]->setLocalpathAsNotTested();
    }
}

vector< pair<double,double > > Trajectory::getCostProfile()
{
    vector< pair<double,double> > vectOfCost;

    double previousPathParam = 0.0;

    for (unsigned int i = 0; i < m_Courbe.size(); i++)
    {
        vector< pair<double,double > > tmp = m_Courbe[i]->getCostProfile();

        for (unsigned int j = 0; j < tmp.size(); j++)
        {
            tmp[j].first += previousPathParam;
            vectOfCost.push_back( tmp[j] );
        }

        previousPathParam += m_Courbe[i]->getParamMax();

        //		vectOfCost.insert( vectOfCost.end() ,
        //											tmp.begin(),
        //											tmp.end());
    }

    return vectOfCost;
}

double Trajectory::computeSubPortionMaxCost(vector<LocalPath*>& portion)
{
    double maxCost(0.0);

    for (int i=0; i<int(portion.size()); i++)
    {
        double cost =  portion[i]->cost();

        if( cost > maxCost )
        {
            maxCost = cost;
        }
    }
    return maxCost;
}

double Trajectory::computeSubPortionCost(const vector<LocalPath*>& portion) const
{
    double sumCost(0.0);

    //    cout << " cost : " ;

    for (int i=0; i<int(portion.size()); i++)
    {
        double cost = portion[i]->cost();

        //        cout << portion[i]->getBegin()->cost() << "  " ;

        //    cout << "cost[" << i << "] = " << cost << endl;
        //    cout << "resolution[" << i << "] = " << portion[i]->getResolution() ;
        //    cout << " , length["  << i << "] = " << portion[i]->getParamMax() ;
        //		cout << ", cost[" << i << "] = " << cost << endl;
        sumCost += cost;
    }

    //    cout << endl;

    return sumCost;
}

double Trajectory::ReComputeSubPortionCost(vector<LocalPath*>& portion, int& nb_cost_tests)
{
    double sumCost(0.0);

    nb_cost_tests = 0;

    for (int i=0; i<int(portion.size()); i++)
    {
        portion[i]->resetCostComputed();
        double cost =  portion[i]->cost();

        nb_cost_tests += portion[i]->getNbCostTest();
        //    cout << "cost[" << i << "] = " << cost << endl;
        //    cout << "resolution[" << i << "] = " << portion[i]->getResolution() ;
        //    cout << " , length["  << i << "] = " << portion[i]->getParamMax() ;
        //		cout << ", cost[" << i << "] = " << cost << endl;
        sumCost += cost;
    }

    return sumCost;
}

double Trajectory::computeSubPortionIntergralCost(vector<LocalPath*>& portion)
{
    double cost(0.0);
    double step = p3d_get_env_dmax()*PlanEnv->getDouble(PlanParam::costResolution);
    double currentParam(0.0), currentCost, prevCost;
    double range = computeSubPortionRange(portion);
    int n_step = int(range/step);

    confPtr_t q = configAtParam(0.0);
    prevCost = q->cost();

    cout << "--------- Integral ----------------" << endl;
    cout << "Range = " << range << endl;
    cout << "step = " << step << endl;
    cout << "n_step = " << n_step << endl;

    for (int i=0; i<n_step;i++ )
    {
        currentParam += step;

        q = configAtParam(currentParam);
        currentCost = q->cost();

        double delta_cost = global_costSpace->deltaStepCost( prevCost, currentCost, step );

        cost += delta_cost;
        prevCost = currentCost;
    }
    return cost;
}

double Trajectory::computeSubPortionCostVisib( vector<LocalPath*>& portion )
{
    double epsilon = 0.002;
    double cost(0.0);
    double dmax = 0;

    bool inVisibilty(false);

    for( unsigned int i=0; i<m_Courbe.size();i++)
    {
        double resol = m_Courbe[i]->getResolution();
        dmax += resol;
    }

    dmax /= (double)m_Courbe.size();

    double currentParam(0.0);
    double prevCost;
    double currentCost = portion[0]->getBegin()->cost();
    double range = computeSubPortionRange(portion);

    int jnt_id=0;

#if defined( HRI_COSTSPACE ) && defined ( HRI_PLANNER )
    //jnt_id = hriSpace->getTask();
    cout << "Be carefull not defined" << endl;
#else
    cout << "Error : HRI Planner not compiled nor linked" << endl;
    return 0;
#endif

    m_Robot->setAndUpdate(*m_Source);
    Vector3d prevPos;
    Vector3d currentPos = m_Robot->getJointPos(jnt_id);

    while (currentParam <= range)
    {
        currentParam += dmax;

        shared_ptr<Configuration> currentConf = configAtParam(currentParam);

        prevCost = currentCost;
        prevPos = currentPos;

        m_Robot->setAndUpdate(*currentConf);
        currentPos = m_Robot->getJointPos(jnt_id);
        double distStep=0;
        for(int k=0;k<currentPos.size();k++)
        {
            distStep += pow((currentPos[k]-prevPos[k]),2);
        }
        distStep = sqrt(distStep);

        double step_cost;

        if(!inVisibilty)
        {
            currentCost = currentConf->cost();
            //the cost associated to a small portion of curve

            step_cost =
                    global_costSpace->deltaStepCost(prevCost, currentCost,distStep);
            //			p3d_ComputeDeltaStepCost(prevCost, currentCost, distStep);

            cout << " Step Cost = " << step_cost << endl;

            if( currentCost < ENV.getDouble(Env::visThresh) )
            {
                inVisibilty = true;
            }
        }
        else
        {
            step_cost =  epsilon * distStep;
        }

        cost += step_cost;
    }

    return cost;
}

double Trajectory::collisionCost() const
{  
    if (isValid())
    {
        return 0.0;
    }
    else {
        return 1000.0;
    }
}

double Trajectory::cost() const
{
    if( !ENV.getBool(Env::isCostSpace) )
    {
        return collisionCost();
    }

    double cost(0.0);
    cost = computeSubPortionCost(m_Courbe);
    // cost =  computeSubPortionIntergralCost(m_Courbe);
    // cost = ReComputeSubPortionCost(m_Courbe,nb_cost_tests);
    return cost;
}

double Trajectory::costRecomputed()
{
    if( !ENV.getBool(Env::isCostSpace) )
    {
        return collisionCost();
    }

    int nb_test=0;
    return ReComputeSubPortionCost(m_Courbe,nb_test);
}

double Trajectory::costNoRecompute()
{
    if( !ENV.getBool(Env::isCostSpace) )
    {
        return collisionCost();
    }

    return computeSubPortionCost(m_Courbe);
}

double Trajectory::costStatistics(TrajectoryStatistics& stat)
{
    int nb_cost_tests=0; int total_cost_tests=0;

    stat.length = getRangeMax();

    if( global_costSpace != NULL )
    {
        stat.is_valid = isValid();

        CostSpaceDeltaStepMethod method = global_costSpace->getDeltaStepMethod();

        stat.sum = costNPoints(100);

        global_costSpace->setDeltaStepMethod( cs_max );
        ReComputeSubPortionCost( m_Courbe, nb_cost_tests );
        stat.max = computeSubPortionMaxCost( m_Courbe );
        total_cost_tests += nb_cost_tests;

        global_costSpace->setDeltaStepMethod( cs_average );
        stat.average = ReComputeSubPortionCost( m_Courbe, nb_cost_tests );

        global_costSpace->setDeltaStepMethod( cs_integral );
        stat.integral = ReComputeSubPortionCost( m_Courbe, nb_cost_tests );

        global_costSpace->setDeltaStepMethod( cs_mechanical_work );
        stat.mecha_work = ReComputeSubPortionCost( m_Courbe, nb_cost_tests );

        global_costSpace->setDeltaStepMethod( method );

        resetCostComputed();
    }
    else
    {
        stat.max = 0.0;
        stat.average = 0.0;
        stat.integral = 0.0;
        stat.mecha_work = 0.0;
    }

    return stat.integral;
}

double Trajectory::costDeltaAlongTraj()
{
    if( !ENV.getBool(Env::isCostSpace) )
    {
        return collisionCost();
    }

    cout << "Sum of LP cost = " << computeSubPortionCost(m_Courbe) << endl;
    Trajectory tmp(*this);
    if( tmp != (*this) ){
        cout << "Trajectory not the same" << endl;
    }
    int nb_cost_tests=0;
    cout << "Sum of LP cost (Recomputed) = "  << ReComputeSubPortionCost(tmp.m_Courbe, nb_cost_tests) << endl;
    double cost = computeSubPortionIntergralCost(m_Courbe);
    cout << "Intergral along traj = " << cost << endl;
    return cost;
}

double Trajectory::costNPoints(const int n_points)
{
    double s = 0.0;
    double cost = 0.0;
    double delta = getRangeMax()/double(n_points-1);

    for (int i=0; i<n_points; i++)
    {
        cost += configAtParam(s)->cost();
        //cout << "delta_cost["<<i<<"] = " << configAtParam(s)->cost() << endl;
        s += delta;
    }
    //cout << "cost : " << cost << endl;
    return cost;
}

double Trajectory::costSum()
{
    double cost = 0.0;
    int i=0;

    for (i=0; i<int(m_Courbe.size()); i++)
    {
        cost += m_Courbe[i]->getBegin()->cost();
        //cout << "delta_cost["<<i<<"] = " << m_Courbe[i]->getBegin()->cost() << endl;
    }
    cost += m_Courbe[i-1]->getEnd()->cost();
    //cout << "delta_cost["<<i-1<<"] = " << m_Courbe[i-1]->getEnd()->cost() << endl;
    return cost;
}

double Trajectory::costOfPortion(double param1, double param2)
{

    uint first(0);
    uint last(0);
    vector<LocalPath*> path;

    pair<bool, vector<LocalPath*> > valid_portion = extractSubPortion(param1, param2, first, last);

    if( valid_portion.first )
    {
        path = valid_portion.second;
    }
    else {
        cout << "Error: inconsistant query in costOfPortion" << endl;
    }

    double Cost = computeSubPortionCost(path);

    for (unsigned int i = 0; i < path.size(); i++)
    {
        delete path.at(i);
    }
    return Cost;

}
//double Trajectory::costOfPortion(double param1,double param2){
//
//	double soFar(0.0);
//	double prevSoFar(0.0);
//	uint id_start;
//	uint id_end;
//	shared_ptr<Configuration> confPtrStart;
//	shared_ptr<Configuration> confPtrEnd;
//
//	// TODO change that function
//
//	if( param1 > param2 ){
//		cout << "Error: in Trajectory::costofPortion() " << endl;
//		return 0;
//	}
//
//	for(uint i=0;i<nloc;i++){
//
//		soFar = soFar + m_Courbe.at(i)->getParamMax();
//
//		// Parameter lies in the previous local path
//		// return configuration inside previous LP
//		if(param1 < soFar){
//
//			if(param1 < prevSoFar)
//				cout << "Error getting Conf at param" << endl;
//
//			confPtrStart = m_Courbe.at(i)->configAtParam(param1-prevSoFar);
//			id_start = i;
//			break;
//		}
//		if(i==nloc-1){
//			confPtrStart = m_Courbe.at(i)->configAtParam(param1-prevSoFar);
//			id_start = i;
//		}
//		prevSoFar = soFar;
//	}
//
//	soFar = prevSoFar;
//
//	for(uint i=id_start;i<nloc;i++){
//
//		soFar = soFar + m_Courbe.at(i)->getParamMax();
//
//		// Paramameter lies in the previous local path
//		// return configuration inside previous LP
//		if(param2 < soFar){
//
//			if(param2 < prevSoFar)
//				cout << "Error getting Conf at param" << endl;
//
//			confPtrEnd = m_Courbe.at(i)->configAtParam(param2-prevSoFar);
//			id_end = i;
//			break;
//		}
//		if(i==nloc-1){
//			confPtrEnd = m_Courbe.at(i)->getEnd();
//			id_end = i;
//		}
//		prevSoFar = soFar;
//	}
//
//	LocalPath pathStart(confPtrStart, m_Courbe.at(id_start)->getEnd());
//	LocalPath pathEnd(m_Courbe.at(id_end)->getBegin(),confPtrEnd);
//
//	double cost =0.0;
//
//	cost += pathStart.cost() + pathEnd.cost();
//
//	for(uint i=id_start+1;i<id_end;i++){
//		cost+=m_Courbe.at(i)->cost();
//	}
//
//	return cost;
//}

void Trajectory::resetCostComputed()
{
    m_Source->setCostAsNotTested();
    m_Target->setCostAsNotTested();

    for (unsigned int i = 0; i < m_Courbe.size(); i++)
    {
        m_Courbe[i]->resetCostComputed();
        m_Courbe[i]->getBegin()->setCostAsNotTested();
        m_Courbe[i]->getEnd()->setCostAsNotTested();
    }
}

vector<confPtr_t> Trajectory::getTowConfigurationAtParam(double param1, double param2, uint& lp1, uint& lp2) const
{
    vector<confPtr_t> conf;

    if (param1 < 0)
    {
        cout << "Error: the parameter is out of band" << endl;
    }
    if (param2 > getRangeMax() )
    {
        cout << "Error: the parameter is out of band" << endl;
    }

    if (param1 > param2)
    {
        cout
                << "Error: not possible to replace trajectory because of the parameters"
                << endl;
        return conf;
    }

    // Looks for the local paths to be changed linear in NLOC
    //-------------------------------------------------------------------
    double soFar(0.0);
    double prevSoFar(0.0);

    shared_ptr<Configuration> q1;
    shared_ptr<Configuration> q2;

    for (uint i = 0; i < m_Courbe.size(); i++)
    {
        soFar = soFar + m_Courbe.at(i)->getParamMax();

        if (param1 < soFar)
        {
            q1 = m_Courbe.at(i)->configAtParam(param1 - prevSoFar);
            //			q1->print();
            //			param_start = param1-prevSoFar;
            lp1 = i;
            break;
        }
        prevSoFar = soFar;
        if (i == (m_Courbe.size() - 1))
        {
            q1 = m_Courbe.at(i)->getEnd();
            lp1 = i;
            cout << "Error : conf ends local path" << endl;
        }
    }

    // id_LP_1 is the id of the path of which param1 lies in
    // we start searching for parm2 from here
    soFar = prevSoFar;

    for (uint i = lp1; i < m_Courbe.size(); i++)
    {
        soFar = soFar + m_Courbe.at(i)->getParamMax();

        if (param2 < soFar)
        {
            q2 = m_Courbe.at(i)->configAtParam(param2 - prevSoFar);
            //			q2->print();
            //			param_end = param2-soFar;
            lp2 = i;
            break;
        }
        prevSoFar = soFar;

        if (i == (m_Courbe.size() - 1))
        {
            q2 = m_Courbe.at(m_Courbe.size() - 1)->getEnd();
            lp2 = i;
        }
    }

    //        q1->setConstraints();
    //        q2->setConstraints();

    conf.push_back(q1);
    conf.push_back(q2);
    return conf;
}

double Trajectory::extractCostPortion(double param1, double param2)
{
    double totalCost(0.0);

    vector<confPtr_t> conf;

    uint first;
    uint last;

    conf = getTowConfigurationAtParam(param1, param2, first, last);

    confPtr_t q1 = conf.at(0);
    confPtr_t q2 = conf.at(1);

    if (first > last)
    {
        cout
                << "Error: extractCostPortion: inconsistent query for subpath extraction"
                << endl;
        return 0;
    }

    // Case where they lie in the same path
    //----------------------------------------------------------------------------
    if (first == last)
    {
        if (q1->equal(*q2))
        {
            cout
                    << "Error: extractCostPortion: q1 and q2 are the same in subportion extraction"
                    << endl;
            return 0;
        }
        LocalPath LP(q1, q2);
        return LP.cost();
    }

    confPtr_t start = m_Courbe.at(first)->getBegin();
    confPtr_t end = m_Courbe.at(last)->getEnd();

    // Adds the modified first local path to subpaths
    // Verifies that the configuration is not starting the local path
    if (!start->equal(*q1))
    {
        if (!m_Courbe.at(first)->getEnd()->equal(*q1))
        {
            LocalPath LP(q1, m_Courbe.at(first)->getEnd());

            if (LP.isValid())
            {
                totalCost += LP.cost();
            }
            else
            {
                cout << "Error: extractCostPortion: portion of path not valid"
                     << endl;
                return -1;
            }
        }
    }
    else
    {

        if (m_Courbe.at(first)->isValid())
        {
            totalCost += m_Courbe.at(first)->cost();
        }
        else
        {
            cout << "Error: extractCostPortion: portion of path not valid"
                 << endl;
            return -1;
        }
    }

    // Adds all the paths between First and Last
    for (uint i = first + 1; i < last; i++)
    {
        totalCost += m_Courbe.at(i)->cost();
    }

    // Verifies that the configuration is not ending the local path
    // Adds the modified Last local path to subpaths
    if (!end->equal(*q2))
    {
        if (!m_Courbe.at(last)->getBegin()->equal(*q2))
        {
            LocalPath LP(m_Courbe.at(last)->getBegin(), q2);

            if (LP.isValid())
            {
                totalCost += LP.cost();
            }
            else
            {
                cout << "Error: extractCostPortion: portion of path not valid"
                     << endl;
                return -1;
            }
        }
    }
    else
    {
        if (m_Courbe.at(last)->isValid())
        {
            totalCost += m_Courbe.at(last)->cost();
        }
        else
        {
            cout << "Error: extractCostPortion: portion of path not valid"
                 << endl;
            return -1;
        }
    }

    // Verifies the integrity of the sub paths
    /*if ((!q1->equal(*paths.at(0)->getBegin())) || (!q2->equal(
     *paths.back()->getEnd())))
     {
     paths.at(0)->getBegin()->print();
     q1->print();
     paths.back()->getEnd()->print();
     q2->print();
     cout << "Error: extractCostPortion: in extract sub portion integrity" << endl;
     }*/

    return totalCost;
}

pair<bool, vector<LocalPath*> > Trajectory::extractSubPortion(double param1, double param2, unsigned int& first, unsigned int& last, bool check_for_coll) const
{
    vector<LocalPath*> paths;
    vector<confPtr_t> conf;

    conf = getTowConfigurationAtParam(param1, param2, first, last);

    confPtr_t q1 = conf.at(0);
    confPtr_t q2 = conf.at(1);

    if (first > last)
    {
        cout << "Error: inconsistent query for subpath extraction" << endl;
        return make_pair(false,paths);
    }

    // Case where they lie in the same path
    //----------------------------------------------------------------------------
    if (first == last)
    {
        if (q1->equal(*q2))
        {
            paths.resize(0);
            cout << "Error: q1 and q2 are the same in subportion extraction"
                 << endl;
            return make_pair(false,paths);
        }
        paths.push_back(new LocalPath(q1, q2));
        return make_pair(true,paths);
    }

    confPtr_t start = m_Courbe.at(first)->getBegin();
    confPtr_t end = m_Courbe.at(last)->getEnd();

    // Adds the modified first local path to subpaths
    // Verifies that the configuration is not starting the local path
    if (!start->equal(*q1))
    {
        if (!m_Courbe.at(first)->getEnd()->equal(*q1))
        {
            LocalPath* startNew = new LocalPath(q1, m_Courbe.at(first)->getEnd());

            if ( (check_for_coll && startNew->isValid()) || !check_for_coll )
            {
                paths.push_back(startNew);
            }
            else
            {
                cout << "Error: portion of path not valid" << endl;
                return make_pair(false,paths);
            }
        }
    }
    else
    {
        LocalPath* startNew = new LocalPath(*m_Courbe.at(first));

        if ( (check_for_coll && startNew->isValid()) || !check_for_coll )
        {
            paths.push_back(startNew);
        }
        else
        {
            cout << "Error: portion of path not valid" << endl;
            return make_pair(false,paths);
        }
    }

    // Adds all the paths between First and Last
    for (uint i = first + 1; i < last; i++)
    {
        paths.push_back(new LocalPath(*m_Courbe.at(i)));
    }

    // Verifies that the configuration is not ending the local path
    // Adds the modified Last local path to subpaths
    if (!end->equal(*q2))
    {
        if (!m_Courbe.at(last)->getBegin()->equal(*q2))
        {
            LocalPath* endNew = new LocalPath(m_Courbe.at(last)->getBegin(), q2);

            if( (check_for_coll && endNew->isValid()) || !check_for_coll )
            {
                paths.push_back(endNew);
            }
            else
            {
                cout << "Error: portion of path not valid" << endl;
                return make_pair(false,paths);
            }
        }
    }
    else
    {
        LocalPath* endNew = new LocalPath(*m_Courbe.at(last));

        if( ( check_for_coll && endNew->isValid() ) || !check_for_coll )
        {
            paths.push_back(endNew);
        }
        else
        {
            cout << "Error: portion of path not valid" << endl;
            return make_pair(false,paths);
        }
    }

    // Verifies the integrity of the sub paths
    if ((!q1->equal(*paths.at(0)->getBegin())) || (!q2->equal(
                                                       *paths.back()->getEnd())))
    {
        paths.at(0)->getBegin()->print();
        q1->print();
        paths.back()->getEnd()->print();
        q2->print();
        cout << "Error: in extract sub portion integrity" << endl;
    }

    return make_pair(true,paths);
}

//! Extract sub trajectory
//! @param start is the id of the first localpath
//! @param end is the id of the last localpath
Trajectory Trajectory::extractSubTrajectoryOfLocalPaths(unsigned int id_start, unsigned int id_end) const
{
    vector<LocalPath*> path;

    if (id_start > id_end)
    {
        cout << "Warning: inconsistant query in extractSubTrajectory" << endl;
    }
    else
    {
        for (unsigned int i=id_start; i<=id_end; i++)
        {
            path.push_back(new LocalPath(*m_Courbe[i]));
        }
    }

    Trajectory newTraj(m_Robot);

    newTraj.m_Courbe = path;

    if (path.size() == 0)
    {
        newTraj.m_Source = m_Courbe[id_start]->getBegin();
        newTraj.m_Target = m_Courbe[id_end]->getEnd();
    }
    else
    {
        newTraj.m_Source = path.at(0)->getBegin();
        newTraj.m_Target = path.back()->getEnd();
    }

    return newTraj;
}

Trajectory Trajectory::extractSubTrajectory(double param1, double param2, bool check_for_coll) const
{
    unsigned int first(0);
    unsigned int last(0);

    Trajectory newTraj( m_Robot );

    if (param1 > param2)
    {
        cout << "Warning: inconsistant query in extractSubTrajectory" << endl;
    }
    else
    {
        pair<bool, vector<LocalPath*> > valid_portion = extractSubPortion( param1, param2, first, last, check_for_coll );

        if( check_for_coll )
        {
            if( valid_portion.first )
            {
                newTraj.m_Courbe = valid_portion.second;
            }
            else {
                cout << "Error: inconsistant query in extractSubTrajectory" << endl;
            }
        }
        else {
            newTraj.m_Courbe = valid_portion.second;
        }
    }

    if ( newTraj.m_Courbe.size() == 0 )
    {
        newTraj.m_Source = configAtParam( param1 );
        newTraj.m_Target = newTraj.m_Source;
    }
    else
    {
        newTraj.m_Source = newTraj.m_Courbe[0]->getBegin();
        newTraj.m_Target = newTraj.m_Courbe.back()->getEnd();
    }

    return newTraj;
}

Trajectory Trajectory::extractReverseTrajectory() const
{
    Trajectory newTraj( m_Robot );
    newTraj.m_Source = m_Target;
    newTraj.m_Target = m_Source;

    for(int i=int(m_Courbe.size()-1);i>=0;i--)
    {
        newTraj.push_back( m_Courbe[i]->getBegin()->copy() );
    }

    return newTraj;
}

extern double ZminEnv;
extern double ZmaxEnv;

#ifndef P3D_PLANNER
double ZminEnv;
double ZmaxEnv;
#endif

extern void* GroundCostObj;

void Trajectory::draw(int nbKeyFrame)
{
    double du = getRangeMax() / (nbKeyFrame - 1);
    if( du == 0.0 )
        return;

    double u = du;

    double Cost1, Cost2;

    p3d_jnt* drawnjnt = NULL;

    int indexjnt = p3d_get_user_drawnjnt();
    if (indexjnt != -1 && indexjnt <= m_Robot->getRobotStruct()->njoints ) {
        drawnjnt = m_Robot->getRobotStruct()->joints[indexjnt];
    }

    confPtr_t qSave = m_Robot->getCurrentPos();
    confPtr_t q = m_Source;
    m_Robot->setAndUpdate(*q);

    p3d_vector3 pi, pf;
    p3d_jnt_get_cur_vect_point(drawnjnt, pi);

    int saveColor;
    bool red = false;

    double range_max = getRangeMax();

    while ( u <= range_max )
    {
        if( u > ( range_max - du/2 ) )
            u = range_max;

        /* position of the robot corresponding to parameter u */
        q = configAtParam(u);
        m_Robot->setAndUpdate(*q);
        p3d_jnt_get_cur_vect_point(drawnjnt, pf);

        if (isHighestCostIdSet)
        {
            if (getLocalPathId(u) == HighestCostId && !red)
            {
                red = true;
                saveColor = mColor;
                mColor = 3;
            }
            if ((mColor == 3) && (getLocalPathId(u) != HighestCostId) && red)
            {
                mColor = saveColor;
                red = false;
            }
        }

        if ((!ENV.getBool(Env::isCostSpace)) || (GroundCostObj == NULL))
        {
            glLineWidth(3.);
            g3d_drawOneLine(pi[0], pi[1], pi[2], pf[0], pf[1], pf[2], mColor, NULL);
            glLineWidth(1.);
        }
        else
        {
            /*val1 =*/ GHintersectionVerticalLineWithGround(GroundCostObj, pi[0], pi[1], &Cost1);
            /*val2 =*/ GHintersectionVerticalLineWithGround(GroundCostObj, pf[0], pf[1], &Cost2);
            glLineWidth(3.);
            g3d_drawOneLine(pi[0], pi[1], Cost1 + (ZmaxEnv - ZminEnv) * 0.02,
                            pf[0], pf[1], Cost2 + (ZmaxEnv - ZminEnv) * 0.02,
                            mColor, NULL);
            glLineWidth(1.);
        }
        p3d_vectCopy(pf, pi);
        u += du;
    }

    if ((ENV.getBool(Env::isCostSpace)) && (GroundCostObj != NULL))
    {
        for (unsigned int i=0; i<m_Courbe.size(); i++)
        {
            m_Robot->setAndUpdate(*m_Courbe[i]->getEnd());
            p3d_jnt_get_cur_vect_point(drawnjnt, pf);
            /*val2 =*/ GHintersectionVerticalLineWithGround(GroundCostObj, pf[0],pf[1], &Cost2);
            g3d_drawSphere(pf[0],pf[1], Cost2 + (ZmaxEnv - ZminEnv) * 0.02,1.);
        }
    }

    m_Robot->setAndUpdate(*qSave);
}

bool Trajectory::push_back(shared_ptr<LocalPath> path)
{
    if( m_Courbe.empty() )
    {
        m_Source = path->getBegin();
        m_Target = path->getEnd();

        m_Courbe.push_back(new LocalPath(*path));
    }
    else
    {
        if ( m_Target->equal(*path->getBegin()) )
        {
            m_Courbe.push_back(new LocalPath(*path));
            m_Target = path->getEnd();
        }
        else {
            return false;
        }
    }

    return true;
}

void Trajectory::push_back(shared_ptr<Configuration> q)
{
    if( m_Courbe.empty() )
    {
        if( m_Source->getConfigStruct() == NULL )
        {
            m_Source = q;
        }
        else
        {
            if ( !m_Source->equal(*q) )
            {
                m_Courbe.push_back(new LocalPath(m_Source,q));
                m_Target = q;
            }
        }
    }
    else
    {
        if ( !m_Target->equal(*q) )
        {
            m_Courbe.push_back(new LocalPath(m_Target,q));
            m_Target = q;
        }
    }
}

bool Trajectory::cutTrajInSmallLPSimple(unsigned int nLP)
{
    double range = computeSubPortionRange(m_Courbe);
    double delta = range/double(nLP);

    vector<LocalPath*> portion;
    bool null_length_local_path = false;
    double length = 0.0;
    double s=0.0;
    //    cout << "range : " << range << " , delta : " << delta << endl;

    for( unsigned int i=0; i<nLP; i++ )
    {
        confPtr_t q_init = configAtParam(s);
        confPtr_t q_goal = configAtParam(s+delta);
        portion.push_back(new LocalPath(q_init,q_goal));
        s += delta;

        double path_length = portion.back()->getParamMax();
        if( path_length == 0.0 )
            null_length_local_path = true;

        length += portion.back()->getParamMax();
    }

    m_Courbe = portion;

    if( portion.size() != nLP ){
        throw string("Error: int cutTrajInSmallLPSimple");
    }

    if( length == 0.0 ){
        cout << "Null length trajectory in cutTrajInSmallLPSimple" << endl;
        return false;
    }

    if( null_length_local_path  ){
        cout << "Null length localpath in cutTrajInSmallLPSimple" << endl;
        return false;
    }

    return true;
}

unsigned int Trajectory::cutPortionInSmallLP(vector<LocalPath*>& portion, unsigned int nLP)
{
    double range = computeSubPortionRange(portion);

    double soFar = 0.0;
    double prevSoFar = 0.0;
    double param = 0.0;

    if (nLP == 0)
    {
        cout << "Error: cutPortionInSmallLP" << endl;
    }

    if (nLP == 1)
    {
        return nLP;
    }

    if( portion.empty() )
    {
        cout << "Can not cut an empty portion" << endl;
        return 0;
    }

    if( range == 0.0 )
    {
        cout << "Can not cut a zero range trajectory" << endl;
        return 0;
    }

    cout << "NB Of LP = " << portion.size() << endl;
    cout << "NB Of LP = " << nLP << endl;

    shared_ptr<Configuration> confPtr;
    shared_ptr<Configuration> confPtrTmp;

    prevSoFar			= 0.0;
    soFar					= portion[0]->getParamMax();
    confPtrTmp		= portion[0]->getBegin();

    const double dMax = range / nLP;

    // Compute real number of small LP
    unsigned int nbOfSmallLP = 0;

    for ( int i=0; i<int(portion.size()); i++)
    {
        double resol;
        double length = portion[i]->getParamMax();

        if ( length < dMax ) {
            resol = length;
        }
        else if ( floor(length/dMax) == length/dMax ) {
            resol = dMax;
        }
        else {
            double n = floor(length/dMax); // minimal number of segment
            resol = length / n;
        }

        unsigned int nbOfDMax = floor((portion[i]->getParamMax()/resol)+0.5);
        nbOfSmallLP += nbOfDMax;
    }
    cout << "range = " << range << endl;
    cout << "nbOfSmallLP = " << nbOfSmallLP << endl;
    vector<LocalPath*> portionTmp;

    try
    {
        int j=0;
        bool tooFar=false;

        // Loop for every small localpath
        for ( int i=0; i<int(nbOfSmallLP); i++)
        {
            double resol;
            double length = portion[j]->getParamMax();

            if ( length < dMax )
            {
                resol = length;
            }
            else if ( floor(length/dMax) == length/dMax )
            {
                resol = dMax;
            }
            else
            {
                double n = floor(length/dMax); // minimal number of segment
                resol = length / n;
            }

            param += resol;

            // Check to go seek the conf on next big LP
            while ( param > soFar )
            {
                j++;
                if ( j >= int(portion.size()) )
                {
                    cout << "Warning : Went too far on traj" << endl;
                    j = portion.size() - 1;
                    tooFar = true;
                    break;
                }
                prevSoFar = soFar;
                soFar += portion[j]->getParamMax();
            }

            if (!tooFar) {
                // Create small localpath
                confPtr = portion[j]->configAtParam(param - prevSoFar);
                portionTmp.push_back(new LocalPath(confPtrTmp, confPtr));
                confPtrTmp = confPtr;
            }
            else {
                break;
            }
        }
    }
    catch(string str)
    {
        cout << "Exeption in cutPortionInSmallLP" << endl;
        cout << str << endl;
        return 0;
    }
    catch (...)
    {
        cout << "Exeption in cutPortionInSmallLP" << endl;
        return 0;
    }

    // The last LP is made of the end configuration
    if( portionTmp.size() >= 2 ) {
        confPtrTmp = portionTmp[portionTmp.size()-2]->getEnd();
    }
    else {
        //portionTmp.back
        confPtrTmp = portionTmp.back()->getBegin();
    }

    confPtr = portion.back()->getEnd();

    delete portionTmp.back();
    portionTmp.back() = new LocalPath(confPtrTmp, confPtr);

    cout <<  "old range = " << range << endl;
    cout <<  "new range = " << computeSubPortionRange(portionTmp) << endl;
    cout << "portionTmp.size() : " << portionTmp.size() << endl;

    // Delete and replace every thing
    for (int i = 0; i<int(portion.size()); i++)
    {
        delete portion.at(i);
    }

    portion.clear();
    portion = portionTmp;

    if (nLP != portion.size())
    {
        cout << "Error: cutPortionInSmallLP ( nLP = " << nLP << " , portion.size() = " << portion.size() << " )" << endl;
    }

    return portion.size();
}

bool Trajectory::cutTrajInSmallLP(unsigned int nLP)
{
    bool succeed = true;

    try
    {
        // cutPortionInSmallLP(m_Courbe, nLP);
        succeed = cutTrajInSmallLPSimple(nLP);
    }
    catch(string str)
    {
        cout << "Exeption in cutTrajInSmallLP" << endl;
        cout << str << endl;
        return false;
    }
    catch (...)
    {
        cout << "Exeption in cutTrajInSmallLP" << endl;
        return false;
    }

    // cout << "Cutting into " << nLP << " local paths" << endl;
    // cout << "Traj Size = " << m_Courbe.size() << endl;
    //cout << "Cost Of trajectory :" << this->cost() << endl;

    if( !succeed )
        return false;

    //    if (!m_Source->equal(*configAtParam(0)))
    //    {
    //        cout << "Error" << endl;
    //        return false;
    //    }

    //    if (!m_Target->equal(*configAtParam(getRangeMax())))
    //    {
    //        m_Source->print();
    //        m_Target->print();
    //        configAtParam( getRangeMax() )->print();
    //        cout << "Error" << endl;
    //        return false;
    //    }
    return true;
}

bool Trajectory::concat(const Trajectory& traj)
{
    if( traj.m_Courbe.size() == 0 )
        return true;

    if( !m_Courbe.back()->getEnd()->equal(*traj.m_Courbe[0]->getBegin()))
    {
        m_Courbe.back()->getEnd()->print();
        traj.m_Courbe[0]->getBegin()->print();
        m_Courbe.back()->getEnd()->equal(*traj.m_Courbe[0]->getBegin(),true);
        return false;
    }

    if( m_Courbe.size() == 0 ) {
        m_Source = traj.m_Courbe[0]->getBegin();
    }

    for( int i=0; i<int(traj.m_Courbe.size()); i++ )
    {
        m_Courbe.push_back( new LocalPath( *traj.m_Courbe[i] ) );
    }
    m_Target = traj.m_Courbe.back()->getEnd();
    return true;
}

bool Trajectory::replacePortionOfLocalPaths(unsigned int id1, unsigned int id2, vector<LocalPath*> paths, bool freeMemory )
{
    // WARNING: the ids are ids of nodes and not LocalPaths
    if (id1 == id2)
    {
        cout << "Error: in replace local (id1 and id2 are the same)" << endl;
        return false;
    }
    if (id2 < id1)
    {
        cout << "Error: in replace local (id2 > id2)" << endl;
        return false;
    }
    if (freeMemory)
    {
        for (uint i = id1; i < id2; i++)
        {
            delete m_Courbe[i];
        }
    }

    m_Courbe.erase(m_Courbe.begin() + id1, m_Courbe.begin() + id2);
    m_Courbe.insert(m_Courbe.begin() + id1, paths.begin(), paths.end());
    return true;
}

bool Trajectory::replacePortion( double param1, double param2, vector<LocalPath*> paths , bool freeMemory )
{
    confPtr_t q11 = paths.at(0)->getBegin();
    confPtr_t q12 = paths.back()->getEnd();

    confPtr_t q21;
    confPtr_t q22;

    if (param1 > param2)
    {
        cout
                << "Warning: Error not possible to replace trajectory because of the parameters"
                << endl;
        return false;
    }

    // TODO replace with extratSubPortion

    // Looks for the local paths to be changed linear in NLOC
    //-------------------------------------------------------------------
    double soFar(0.0);
    double prevSoFar(0.0);

//    double param_start(0.0);
//    double param_end(0.0);

    unsigned int id_LP_1(0);
    unsigned int id_LP_2(0);

    for (unsigned int i = 0; i < m_Courbe.size(); i++)
    {
        soFar = soFar + m_Courbe[i]->getParamMax();

        // param1 is in local path i
        if (param1 < soFar)
        {
            // get configuration in local path i
            q21 = m_Courbe[i]->configAtParam(param1 - prevSoFar);
            //param_start = param1 - prevSoFar;
            id_LP_1 = i;
            break;
        }
        prevSoFar = soFar;
        if (i == (getRangeMax() - 1))
        {
            cout << "Warning: first parameter not found on trajectory" << endl;
            //			return;
        }
    }

    soFar = prevSoFar;

    for (unsigned int i = id_LP_1; i < m_Courbe.size(); i++)
    {
        soFar = soFar + m_Courbe[i]->getParamMax();

        // param2 is in local path i
        if (param2 < soFar)
        {
            // get configuration in local path i
            q22 = m_Courbe.at(i)->configAtParam(param2 - prevSoFar);
//            param_end = param2 - soFar;
            id_LP_2 = i;
            break;
        }
        prevSoFar = soFar;

        if (i == (m_Courbe.size() - 1))
        {
            q22 = m_Courbe.at(m_Courbe.size() - 1)->getEnd();
            id_LP_2 = i;
//            param_end = soFar;
        }
    }

    // Condition has to be false if the trajectory is consistent
    if ((!q11->equal(*q21)) || (!q12->equal(*q22)))
    {
        if (!q11->equal(*q21))
        {
            cout << "q11 and q21" << endl;
            q11->print();
            q21->print();
        }
        if (!q12->equal(*q22))
        {
            cout << "q12 and q22" << endl;
            q12->print();
            q22->print();
        }
        cout
                << "Warning: Error not possible to replace trajectory because configuration are not the same"
                << endl;
        return false;
    }

    // Adds to paths the portion of local path (At begin and End of the portion)
    //----------------------------------------------------------------------------
    confPtr_t start = m_Courbe[id_LP_1]->getBegin();
    confPtr_t end =   m_Courbe[id_LP_2]->getEnd();

    /*cout << "Start and End" << endl;
     start->print(); end->print();
     cout << "q21 and q22" << endl;
     q21->print(); q22->print();*/

    // Verifies that the configuration is not starting the local path
    if (!start->equal(*q21))
    {
        LocalPath* startNew = new LocalPath(start, q21);

        if (startNew->isValid())
        {
            paths.insert(paths.begin(), new LocalPath(start, q21));
        }
        else
        {
            //cout << "Error: portion of path not valid" << endl;
            return false;
        }
    }

    // Verifies that the configuration is not ending the local path
    if (!end->equal(*q22))
    {
        LocalPath* endNew = new LocalPath(q22, end);

        if (endNew->isValid())
        {
            paths.push_back(endNew);
        }
        else
        {
            //cout << "Error: portion of path not valid" << endl;
            return false;
        }
    }

    /*for(int i=0;i<paths.size();i++)
     {
     paths[i]->print();
     }*/
    //	cout << "PathSizeOf() = " << paths.size() << endl;
    // TODO optional
    //cutPortionInSmallLP(paths);


    // Free, Erases the old ones and the Inserts the new ones
    //---------------------------------------------------------------------------
    unsigned int id1_erase = id_LP_1;
    unsigned int id2_erase = id_LP_2 + 1;

    replacePortionOfLocalPaths( id1_erase, id2_erase, paths, freeMemory );
    return true;
}

bool Trajectory::replaceBegin( double param, const vector<LocalPath*>& paths )
{
    uint first(0); uint last(0);
    vector<LocalPath*> new_courbe;
    cout << "Replace begining at " << param << " over (" << getRangeMax() << ")" << endl;

    pair<bool, vector<LocalPath*> > valid_portion = extractSubPortion( param, getRangeMax(), first, last, false );

    if( valid_portion.first )
    {
        new_courbe = paths;
    }
    else {
        cout << "Error: inconsistant query in replaceBegin" << endl;
        return false;
    }

    for (int i=0; i<int(valid_portion.second.size()); i++) {
        new_courbe.push_back( valid_portion.second[i] );
    }

    for (int i=0; i<int(m_Courbe.size()); i++) {
        delete m_Courbe[i];
    }

    m_Courbe = new_courbe;
    m_Source = new_courbe.at(0)->getBegin();
    m_Target = new_courbe.back()->getEnd();
    return true;
}

bool Trajectory::replaceEnd( double param, const vector<LocalPath*>& paths )
{
    uint first(0); uint last(0);
    vector<LocalPath*> new_courbe;

    pair<bool, vector<LocalPath*> > valid_portion = extractSubPortion(0.0, param, first, last, false);

    if( valid_portion.first )
    {
        new_courbe = valid_portion.second;
    }
    else {
        cout << "Error: inconsistant query in replaceEnd" << endl;
        return false;
    }

    for (int i=0; i<int(paths.size()); i++) {
        new_courbe.push_back( paths[i] );
    }

    for (int i=0; i<int(m_Courbe.size()); i++) {
        delete m_Courbe[i];
    }

    m_Courbe = new_courbe;
    m_Source = new_courbe.at(0)->getBegin();
    m_Target = new_courbe.back()->getEnd();
    return true;
}

unsigned int Trajectory::getLocalPathId(double param) const
{
    double soFar(0.0);

    for (unsigned int i = 0; i < m_Courbe.size(); i++)
    {
        soFar = soFar + m_Courbe[i]->getParamMax();

        if (param < soFar)
        {
            return i;
        }
    }
    return m_Courbe.size() - 1;
}

int Trajectory::meanCollTest()
{
    int CollTest = 0.0;
    for(unsigned int i=0;i<m_Courbe.size();i++)
    {
        if(!(m_Courbe[i]->isValid()))
        {
            cout << "Trajectory::Warning => LocalPath is not valid in trajectory" << endl;
        }
        CollTest += m_Courbe[i]->getNbColTest();
    }
    return (int)(CollTest/m_Courbe.size());
}

vector<double> Trajectory::getCostAlongTrajectory(int nbSample)
{

    double step = this->getRangeMax() / (double) nbSample;

    vector<double> cost;

    for( double param=0; param<this->getRangeMax(); param = param + step)
    {
        cost.push_back(this->configAtParam(param)->cost());
        //        cout << this->configAtParam(param)->cost() << endl;
    }

    cout << "Compute Cost Along Traj of " << cost.size() << " samples" << endl;

    cost.resize(nbSample);

    return cost;
}

// Returns a matrix with the waypoints of the trajectory
// The number of rows is the number of dofs
// The number of cols is the number of waypoints
Eigen::MatrixXd Trajectory::getEigenMatrix(int startIndex, int endIndex) const
{
    if( startIndex==0 && endIndex==0)
    {
        if( m_Courbe.size() > 0 )
        {
            int rows = m_Courbe[0]->getBegin()->getEigenVector().size();
            int cols = m_Courbe.size()+1;

            Eigen::MatrixXd mat( rows, cols );

            for (int j=0; j<int(m_Courbe.size()); j++)
            {
                mat.col(j) = m_Courbe[j]->getBegin()->getEigenVector();
            }

            if( m_Courbe.size()-1 >= 0 )
            {
                mat.col(m_Courbe.size()) = m_Courbe.back()->getEnd()->getEigenVector();
            }

            return mat;
        }
        else{
            return Eigen::MatrixXd(0,0);
        }
    }
    else
    {
        // Get indicies in order
        std::vector<int> incides;
        for(int i=startIndex;i<=endIndex;i++)
            incides.push_back(i);

        return getEigenMatrix( incides );
    }
}

Eigen::MatrixXd Trajectory::getEigenMatrix(const std::vector<int>& incides) const
{
    if( m_Courbe.size() > 0 )
    {
        int rows = incides.size();
        int cols = m_Courbe.size()+1;

        Eigen::MatrixXd mat( rows, cols );

        for (int j=0; j<int(m_Courbe.size()); j++)
        {
            mat.col(j) = m_Courbe[j]->getBegin()->getEigenVector( incides );
        }

        if( m_Courbe.size()-1 >= 0 )
        {
            mat.col(m_Courbe.size()) = m_Courbe.back()->getEnd()->getEigenVector( incides );
        }

        return mat;
    }
    else{
        return Eigen::MatrixXd(0,0);
    }
}

void Trajectory::printAllLocalpathCost()
{
    cout <<  "( " ;
    for ( int i=0; i<int(m_Courbe.size()); i++)
    {
        cout <<  m_Courbe[i]->cost();

        if (i != int(m_Courbe.size()-1)) {
            cout << " , " ;
        }
        else {
            cout << " )" << endl;
        }
    }
}

void Trajectory::print()
{
    cout << "-------------- Trajectory --------------" << endl;
    cout << " Number of LP " << m_Courbe.size() << endl;
    cout << " Range Parameter " << this->getRangeMax() << endl;

    Eigen::MatrixXd mat = getEigenMatrix();
    cout << mat << endl;
    cout << "-----------------------------------" << endl;
}

void draw_traj_debug()
{
    if( ENV.getBool(Env::debugCostOptim) || ENV.getBool(Env::drawTrajVector) )
    {
        //std::cout << "Should be drawing traj" << std::endl;
        for(unsigned i=0;i<global_trajToDraw.size();i++)
        {
            global_trajToDraw.at(i).draw(500);
            //std::cout << "Drawing traj" << std::endl;
        }
        //    p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
        //    if (!robotPt->tcur)
        //    {
        //      global_trajToDraw.clear();
        //    }
    }
}
