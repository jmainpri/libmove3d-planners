/*
 * CostOptimization.cpp
 *
 *  Created on: Jun 25, 2009
 *      Author: jmainpri
 */

#include "costOptimization.hpp"

#include <algorithm>

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"

#include "API/Grids/gridsAPI.hpp"
#include "API/scene.hpp"

#include "planEnvironment.hpp"
#include "replanningSimulators.hpp"
#include "cost_space.hpp"

#include <iomanip>
#include <iosfwd>
#include <sstream>
#include <sys/time.h>

const bool show_debug_terminal = false;

extern void* GroundCostObj;

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

using namespace API;

CostOptimization::CostOptimization() :
    m_cheat(false),
    m_mincost(numeric_limits<double>::max()),
    m_nbErrors(0),
    m_DeformBiased(true)
{
}

CostOptimization::CostOptimization(const Trajectory& T) :
    Smoothing(T),
    m_cheat(false),
    m_mincost(numeric_limits<double>::max()),
    m_nbErrors(0),
    m_DeformBiased(true)
{
}

CostOptimization::CostOptimization(Robot* R, p3d_traj* t) :
    Smoothing(R, t),
    m_cheat(false),
    m_mincost(numeric_limits<double>::max()),
    m_nbErrors(0),
    m_DeformBiased(true)
{
}

CostOptimization::~CostOptimization()
{

    //	cout << "Delete CostOptimization" << endl;

}

bool CostOptimization::oneLoopDeform()
{
    m_inCollision=false;

    double lPrev = 0., lCurrent = 0., lNext = 0.;
    bool isOptimSuccess(false);
    double TrajCost = this->cost();

    shared_ptr<Configuration> qInitPt = getRobot()->getCurrentPos();

    //Get 3 configurations at random along the trajectory
    vector<confPtr_t>  vectConf = get3RandSuccesConfAlongTraj( lPrev, lCurrent, lNext, m_step );
    confPtr_t qPrevPt = vectConf.at(0);
    confPtr_t qCurrPt = vectConf.at(1);
    confPtr_t qNextPt = vectConf.at(2);

    // Take a configuration in a certain direction
    confPtr_t qRandPt = getRobot()->shoot();

    // Direction path for perturbation
    LocalPath path( qCurrPt, qRandPt );

    // Get the New configuration in qtRand dirrenction
    confPtr_t qNewPt = perturbCurrent( qCurrPt, qRandPt, m_step, m_descent );

    if ( qNewPt->getConfigStruct() == NULL )
        return false;

    if( qPrevPt->equal( *qNewPt ) || qNextPt->equal( *qNewPt ) )
        return false;

    // Set the middle config in the configuration vector
    vectConf[1] = qNewPt;

    // If qNew is free then
    // Check the triangle localPath
    if ( !qNewPt->isInCollision() )
    {
        LocalPath* FirstHalf	= new LocalPath(qPrevPt, qNewPt);
        LocalPath* SecondHalf = new LocalPath(qNewPt,  qNextPt);

        bool supposedValid = true;
        // If the path is valid
        // Check for cost
        if ( !ENV.getBool(Env::costBeforeColl) )
        {
            supposedValid = FirstHalf->isValid() && SecondHalf->isValid();
        }

        if ( supposedValid )
        {
            // Computes the subportion of the CURRENT trajectory
            unsigned int first,last;
            vector<confPtr_t> confs = getTowConfigurationAtParam(lPrev,lNext,first,last);

            vector<LocalPath*> paths;
            for(unsigned int i=first;i<=last;i++)
            {
                paths.push_back(getLocalPath(i));
            }

            double costOfPortion = computeSubPortionCost(paths);

            if( !(*confs[0] == *qPrevPt) || !(*confs[1] == *qNextPt) )
            {
                cout << "Error in oneLoopDeform" << endl;
            }

            paths.clear();
            paths.push_back(FirstHalf);
            paths.push_back(SecondHalf);

            LocalPath* LP1 = new LocalPath(getLocalPath(first)->getBegin(),qPrevPt);
            if(LP1->getParamMax()>0)
            {
                paths.insert(paths.begin(),LP1);
            }

            LocalPath* LP2 = new LocalPath(qNextPt,getLocalPath(last)->getEnd());
            if(LP2->getParamMax()>0)
            {
                paths.push_back(LP2);
            }

            double sumOfCost = computeSubPortionCost(paths);

            delete LP1;
            delete LP2;

            // Replace
            bool lowerCost = ( sumOfCost < costOfPortion );

            if(ENV.getBool(Env::costBeforeColl))
            {
                supposedValid = ( FirstHalf->isValid() && SecondHalf->isValid() );
            }

            if ( lowerCost && supposedValid )
            {
                vector<LocalPath*> newPortion;
                newPortion.push_back(FirstHalf);
                newPortion.push_back(SecondHalf);

                replacePortion(lPrev, lNext, newPortion);

                if (ENV.getBool(Env::debugCostOptim))
                {
                    double newTrajCost = cost();
                    double delta1 = costOfPortion - sumOfCost;
                    double delta2 = TrajCost - newTrajCost;

                    if( TrajCost < newTrajCost )
                    {
                        cout << "Delta = delta2 - delta1 = " << delta2 - delta1 << endl;
                        cout << "Error (TrajCost < newTrajCost)" << endl;
                    }
                }

                // Sort index for biasing
                setSortedIndex();

                // Begin and End validity
                if (! (*getBegin() == *configAtParam(0))) {
                    cout << "------------------------------------------" << endl;
                    cout << "Error in oneLoopDeform : !getBegin()->equal(*configAtParam(0))" << endl;
                }

                if (! (*getEnd() == *configAtParam(getRangeMax()))) {
                    cout << "------------------------------------------" << endl;
                    cout << "Error in oneLoopDeform : !getEnd()->equal(*configAtParam(getRangeMax()))" << endl;
                    getEnd()->print();
                    configAtParam(getRangeMax())->print();
                }

                isOptimSuccess = true;
            }
            if ( ENV.getBool(Env::debugCostOptim) ){
                if (isOptimSuccess) {
                    debugShowTraj(lPrev, lNext, qNewPt, 1);
                }
                else {
                    debugShowTraj(lPrev, lNext, qNewPt, 2);
                }
            }
        }
        else
        {
            m_inCollision=true;
            if (ENV.getBool(Env::debugCostOptim)) {
                debugShowTraj(lPrev, lNext, qNewPt, 3);
            }
        }

        if (!isOptimSuccess) {
            delete FirstHalf;
            delete SecondHalf;
        }
    }
    else {
        m_inCollision=true;
    }

    // Store the trajectory for drawing
    if (PlanEnv->getBool(PlanParam::showExploration))
    {
        global_rePlanningEnv->store_exploration(*this,lPrev, lNext, qNewPt);
    }

    if( (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) ) {

        if( isOptimSuccess ){
            replaceP3dTraj();
        }

        if( ENV.getBool(Env::drawGraph) || isOptimSuccess ) {
            getRobot()->setAndUpdate(*qInitPt);
            g3d_draw_allwin_active();
        }
    }
    return isOptimSuccess;
}

bool CostOptimization::oneLoopDeformRecompute()
{
    m_inCollision=false;

    double lPrev = 0., lCurrent = 0., lNext = 0.;
    bool isOptimSuccess(false);

    // Store current configuration
    confPtr_t qInitPt = getRobot()->getCurrentPos();

    // Get 3 configurations at random along the trajectory
    vector<confPtr_t>  vectConf = get3RandSuccesConfAlongTraj(lPrev,lCurrent,lNext,m_step);
    confPtr_t qPrevPt = vectConf.at(0);
    confPtr_t qCurrPt = vectConf.at(1);
    confPtr_t qNextPt = vectConf.at(2);

    // Take a configuration in a certain direction
    confPtr_t qRandPt = getRobot()->shoot();

    // Direction path for perturbation
    LocalPath path( qCurrPt, qRandPt );

    // Get the New configuration in qtRand dirrenction
    confPtr_t qNewPt = perturbCurrent( qCurrPt, qRandPt, m_step, m_descent );
    if ( qNewPt->getConfigStruct() == NULL )
    {
        return false;
    }

    // Set the middle config in the configuration vector
    vectConf[1] = qNewPt;

    bool newConfigIsValid = true;

    if( PlanEnv->getBool(PlanParam::trajComputeCollision) )
    {
        newConfigIsValid = !qNewPt->isInCollision();
    }

    // If qNew is free, check the triangle localPath
    if (  newConfigIsValid )
    {
        LocalPath* FirstHalf  = new LocalPath(qPrevPt, qNewPt);
        LocalPath* SecondHalf = new LocalPath(qNewPt,  qNextPt);

        bool newPathAreValid = true;

        if (PlanEnv->getBool(PlanParam::trajComputeCollision))
        {
            newPathAreValid = (!FirstHalf->isValid()) || (!SecondHalf->isValid());
        }

        // If the path is valid, check for cost
        if ( newPathAreValid )
        {
            Trajectory newTraj(*this);

            vector<LocalPath*> newPortion;

            newPortion.push_back(FirstHalf);
            newPortion.push_back(SecondHalf);

            newTraj.replacePortion( lPrev, lNext, newPortion );

            if ( newTraj.isValid() )
            {
                double CurrentCost = 0.0;
                double NewTrajCost = 0.0;

                if( PlanEnv->getBool( PlanParam::trajNPoints ) )
                {
                    CurrentCost = m_currentCost;
                    NewTrajCost = newTraj.costNPoints( PlanEnv->getInt(PlanParam::nb_pointsOnTraj) );
                }
                else
                {
                    CurrentCost = this->costRecomputed();
                    NewTrajCost = newTraj.costRecomputed();
                }

                if ( NewTrajCost < CurrentCost )
                {
                    newPortion.clear();
                    newPortion.push_back(new LocalPath(*FirstHalf));
                    newPortion.push_back(new LocalPath(*SecondHalf));

                    this->replacePortion( lPrev, lNext, newPortion );

                    setSortedIndex();

                    if( PlanEnv->getBool( PlanParam::trajNPoints ) )
                    {
                        m_currentCost = NewTrajCost;
                    }

                    if ( *getBegin() != *configAtParam(0) )
                    {
                        cout << "Error in oneLoopDeform : !getBegin()->equal(*configAtParam(0))" << endl;
                    }

                    if ( *getEnd() != *configAtParam(getRangeMax()) )
                    {
                        cout << "Error in oneLoopDeform : !getEnd()->equal(*configAtParam(getRangeMax()))" << endl;
                    }

                    isOptimSuccess = true;
                }
            }

            // Store the trajectory for drawing
            if (PlanEnv->getBool(PlanParam::showExploration))
            {
                //cout << "showExploration" << endl;
                global_rePlanningEnv->store_exploration(*this,lPrev, lNext, qNewPt);
            }

            if (ENV.getBool(Env::debugCostOptim))
            {
                if (isOptimSuccess)
                {
                    debugShowTraj(lPrev, lNext, qNewPt, 1);
                }
                else
                {
                    debugShowTraj(lPrev, lNext, qNewPt, 2);
                }
            }
        }
        else
        {
            m_inCollision=true;

            if (ENV.getBool(Env::debugCostOptim))
            {
                debugShowTraj(lPrev, lNext, qNewPt, 3);
            }
        }
    }
    else
    {
        m_inCollision=true;
    }

    if( ENV.getBool(Env::drawTraj) )
    {
        if( isOptimSuccess )
        {
            replaceP3dTraj();
        }

        if( ENV.getBool(Env::drawGraph) || isOptimSuccess )
        {
            getRobot()->setAndUpdate(*qInitPt);
            g3d_draw_allwin_active();
        }
    }

    return isOptimSuccess;
}

//! Perturb the current configuration
//! Finds a conf between current and random that satifies the joint limits
confPtr_t CostOptimization::perturbCurrent( confPtr_t qCurrPt, confPtr_t qRandPt, double step, bool descent )
{
    confPtr_t qNewPt;
    LocalPath path( qCurrPt, qRandPt );

    const double minStep = 4.0; // PlanEnv->getDouble(PlanParam::MinStep)

    // Dividing the step n times to stay in the bounds
    const unsigned int	max_div = 4;
    unsigned int				ith_div = 0;
    double							divFactor = 1/minStep;
    bool								QIsOutOfBounds = true;

    for (ith_div=0; ith_div<max_div && QIsOutOfBounds; ith_div++)
    {
        if( !descent )
        {
            qNewPt = path.configAtParam( divFactor*step  );
            QIsOutOfBounds = qNewPt->isOutOfBounds();
            divFactor /= 2;
        }
        else {
            qNewPt = path.configAtParam( getLastDescendingConfParam(path) );
            QIsOutOfBounds = false;
        }
    }

    return qNewPt;
}

//! Compute descent on cost map
double CostOptimization::getLastDescendingConfParam(LocalPath& directionPath)
{
    bool failed(false);
    bool upHill(false);

    confPtr_t fromConfig = directionPath.getBegin();

    double praramAlongDirection( directionPath.getRobot()->getActiveScene()->getDMax() );

    double extensionCost(0.);
    double prevCost(0.);

    while ((!failed) && ( praramAlongDirection < directionPath.getParamMax() ) )
    {
        // Take one configuration along the path
        // then test if the new small portion is valid
        confPtr_t toConfig = directionPath.configAtParam( praramAlongDirection );
        LocalPath extensionLP( fromConfig , toConfig );
        failed = (!extensionLP.isValid());
        //nbOfExtend++;

        // Get the cost of the extremal configuration
        prevCost						= fromConfig->cost();
        extensionCost				= toConfig->cost();

        // Cost Test Control
        if ((!failed) && ( prevCost < extensionCost ))
        {
            upHill = true;
            failed = true;
        }

        // Check if it's going down hill for next iterations
        if ( !upHill )
        {
            praramAlongDirection += extensionLP.getParamMax();
            fromConfig = toConfig;
        }
    }

    return praramAlongDirection;
}
/*!
 * Bias to one specific configuration
 */
shared_ptr<Configuration> CostOptimization::cheat()
{
    double q[26];

    q[0] = 0.000000;
    q[1] = 0.000000;
    q[2] = 0.000000;
    q[3] = 0.000000;
    q[4] = 0.000000;
    q[5] = 0.000000;
    q[6] = 96.863319;
    q[7] = -13.028515;
    q[8] = 13.028515;
    q[9] = 0.000000;
    q[10] = 9.292039;
    q[11] = 9.655849;
    q[12] = -15.000000;
    q[13] = -46.000000;
    q[14] = -8.000000;
    q[15] = 119.000000;
    q[16] = 138.000000;
    q[17] = 62.000000;
    q[18] = 29.000000;
    q[19] = -1.671586;
    q[20] = -90.855453;
    q[21] = -56.833824;
    q[22] = 116.814163;
    q[23] = 30.000000;
    q[24] = 14.000000;
    q[25] = 0.000000;

    shared_ptr<Configuration> ptrConfig(new Configuration(this->getRobot(),q));
    ptrConfig->convertToRadian();
    return ptrConfig;
}


void CostOptimization::debugShowTraj(double lPrev, double lNext, confPtr_t qNew, int color)
{
    vector<confPtr_t> vectConf(3);
    vectConf.at(0) = configAtParam(lPrev);
    vectConf.at(1) = qNew;
    vectConf.at(2) = configAtParam(lNext);

    global_trajToDraw.resize(4);

    global_trajToDraw.at(0) = extractSubTrajectory(0, lPrev);
    global_trajToDraw.at(1) = extractSubTrajectory(lPrev, lNext);
    global_trajToDraw.at(2) = *new Trajectory(vectConf);
    global_trajToDraw.at(3) = extractSubTrajectory(lNext, getRangeMax());

    int color_base_traj;

    if( GroundCostObj == NULL )
    {
        color_base_traj = 0;
    }
    else
    {
        color_base_traj = 3;
    }

    global_trajToDraw.at(0).setColor(color_base_traj);
    global_trajToDraw.at(1).setColor(2);
    global_trajToDraw.at(2).setColor(color);
    global_trajToDraw.at(3).setColor(color_base_traj);

    //	g3d_draw_allwin_active();
    //	usleep(200000);

    vector<LocalPath*> pathsTmp(2);
    pathsTmp.at(0) = (new LocalPath(vectConf.at(0), vectConf.at(1)));
    pathsTmp.at(1) = (new LocalPath(vectConf.at(1), vectConf.at(2)));

    Trajectory tmpT(*this);
    tmpT.replacePortion(lPrev, lNext, pathsTmp);
    double newCost = tmpT.cost();

    if( m_mincost > newCost )
    {
        m_mincost = newCost;
    }

    double oldCost = cost();
    double sumOfCost = pathsTmp.at(0)->cost() + pathsTmp.at(1)->cost();
    double costOfPortion1 = this->costOfPortion(lPrev, lNext);
    double costOfPortion2 = this->extractCostPortion(lPrev,lNext);

    if( show_debug_terminal )
    {
        cout << "Difference between costOfPortion and extractCostPortion : "
             << (costOfPortion1 - costOfPortion2) << endl;

        cout << "Difference on the portion : " << (costOfPortion2 - sumOfCost) << endl;
        cout << "Difference on the trajectory: " << (oldCost - newCost) << endl;
        cout << "---------------------------------------------------------" << endl;
    }

    double diff = fabs((costOfPortion2 - sumOfCost) - (oldCost - newCost));

    if( diff > 0.001 )
    {
        m_Errors.push_back(diff);
        m_nbErrors++;
    }

    g3d_draw_allwin_active();
}

void CostOptimization::printDebugInfo()
{
    cout << "Errors : " << endl;
    for (int i=0; i < int(m_Errors.size()); i++)
    {
        cout << "Errors[" << i << "] = " << m_Errors.at(i) << endl;
    }

    if (m_Errors.size() > 2)
    {
        double max = *max_element(m_Errors.begin(), m_Errors.end());
        double min = *min_element(m_Errors.begin(), m_Errors.end());
        cout << "Error Max. =  " << max << endl;
        cout << "Error Min. =  " << min << endl;
    }

    cout << "Selected : " << endl;

    for (int i=0; i < int(m_Selected.size()); i++)
    {
        cout << "mSelected[" << i << "] = " << m_Selected.at(i) << endl;
    }

    cout << "nbBiased = " << m_nbBiased << endl;
    cout << "nbReallyBiased = " << m_nbReallyBiased << endl;
}

int nb_runs = 0;


vector<confPtr_t> CostOptimization::getClosestConfOnTraj(double& prevDistPt, double& randDistPt, double& nextDistPt, confPtr_t ptrConf, double step)
{
    const int N = 30;
    double delta = this->getRangeMax() / (double)N;

    // Gets 30 configurations at a step delta
    vector<confPtr_t> vectConf = getNConfAtParam(delta);
    vector<confPtr_t> threeConfVect;

    // Returns directly the vector if the path is smaller
    // than 3 deltas
    if (vectConf.size() < 2 )
    {
        if (vectConf.size() == 1)
        {
            threeConfVect.push_back(vectConf.at(0));
        }
        else if (vectConf.size() != 0 )
        {
            throw string("Error in CostOptimization::getClosestConfOnTraj");
        }
        return threeConfVect;
    }

    // Compute closest configuration on the trajectory
    double minDist = ptrConf->dist(*vectConf.at(0), GENERAL_CSPACE_DIST);
    unsigned int id = 0;

    for (unsigned int i=1; i < vectConf.size(); i++)
    {
        double dist = ptrConf->dist(*vectConf.at(i), GENERAL_CSPACE_DIST);

        if (dist < minDist)
        {
            minDist = dist;
            id = i;
        }
    }

    // Watch out for extreme values on the trajectory
    threeConfVect.resize(3);

    if (id == 0)
    {
        id = id + 1;
    }
    if (id == vectConf.size() - 1)
    {
        id = id - 1;
    }

    // Compute the id of the 3 configurations in the vector
    randDistPt = delta * (id);

    int nbOfDelta = (int) ((step / delta) + 0.5);

    int idPrev = id - nbOfDelta;
    if (idPrev < 0)
        idPrev = 0;

    int idNext = id + nbOfDelta;
    if (idNext > (int) vectConf.size() - 1)
        idNext = vectConf.size() - 1;

    step = ((double) nbOfDelta) * delta;

    prevDistPt = randDistPt - step;
    if (prevDistPt < 0)
        prevDistPt = 0;

    nextDistPt = randDistPt + step;
    if (nextDistPt > getRangeMax())
        nextDistPt = getRangeMax();

    if (ENV.getBool(Env::debugCostOptim))
    {
        cout << "nbOfDelta = " << nbOfDelta << endl;
        cout << "step = " << step << endl;
        cout << "getRangeMax() = " << getRangeMax() << endl;
        cout << "prevDistPt = " << prevDistPt << endl;
        cout << "randDistPt = " << randDistPt << endl;
        cout << "nextDistPt = " << nextDistPt << endl;
    }

    threeConfVect.at(0) = vectConf.at(idPrev);
    threeConfVect.at(1) = vectConf.at(id);
    threeConfVect.at(2) = vectConf.at(idNext);

    return threeConfVect;
}

//! Gets 3 random configurations on the trajectory
//! the selection is biased depending on an internal parameter
//! The process is as follows, first a configuration is sampled (the one in the middle)
//! then the two others are selected at a parameter prevDist, and nextDist away from the sampled configuration
vector<confPtr_t> CostOptimization::get3RandSuccesConfAlongTraj(double& prevDist, double& randDist, double& nextDist, double step)
{
    vector<confPtr_t> vectConf(3);

    vectConf[1] = getRandConfAlongTraj( randDist, m_DeformBiased );

    // The other configuration are selected at a given
    // step before and after the random configuration
    prevDist = std::max( 0.0, randDist - step/2 );
    nextDist = std::min( getRangeMax(), randDist + step/2);

    vectConf[0] = configAtParam(prevDist);
    vectConf[2] = configAtParam(nextDist);

    if (prevDist > nextDist)
    {
        cerr << "Error in get3RandSuccesConfAlongTraj" << endl;
    }

    return vectConf;
}

//! Connects a configuration to the trajectory
//! it starts at the begining of the trajectory
bool CostOptimization::connectConfigurationToClosestAtBegin( confPtr_t q, double step, bool consider_valid )
{
    double param=0.0;
    double range_max = getRangeMax();
    std::pair<double,LocalPath*> best_so_far;
    best_so_far.first = std::numeric_limits<double>::max();
    best_so_far.second = NULL;

    do
    {
        LocalPath* path = new LocalPath( q, configAtParam( param ) );

        bool is_valid = true;

        if( consider_valid )
        {
            is_valid = path->isValid();
        }

        if( is_valid && ( path->getParamMax() < best_so_far.first ) )
        {
            delete best_so_far.second;
            best_so_far.first = param;
            best_so_far.second = path;
        }
        else {
            delete path;
        }
        param += step;
    }
    while( param < range_max );

//    cout << "param : " << param << endl;
//    cout << "range_max : " << range_max << endl;

    if( best_so_far.second != NULL ) {
        std::vector<LocalPath*> portion;
        portion.push_back( best_so_far.second );
        return replaceBegin( best_so_far.first, portion );
    }
    else {
        cout << "All paths are invalid" << endl;
        return false;
    }
}

//! Connects a configuration to the trajectory
//! it starts at the begining of the trajectory
bool CostOptimization::connectConfigurationToBegin( confPtr_t q, double step, bool consider_cost )
{
    double param=0.0;
    double range_max = getRangeMax();
    std::pair<double,LocalPath*> best_so_far;
    best_so_far.first = 0.0;
    best_so_far.second = NULL;

    do
    {
        LocalPath* path = new LocalPath( q, configAtParam( param ) );

        bool better = false;

        if( path->isValid() )
        {
            better = true;

            if( consider_cost )
            {
                unsigned int loc1, loc2;
                double cost_sub = computeSubPortionCost( extractSubPortion( 0.0, param, loc1, loc2, false ).second );

                if( path->cost() > cost_sub ) {
                    better = false;
                }
            }
        }

        if( better )
        {
            delete best_so_far.second;
            best_so_far.first = param;
            best_so_far.second = path;
        }
        else {
            delete path;
        }
        param += step;
    }
    while( param < range_max );

    cout << "param : " << param << endl;
    cout << "range_max : " << range_max << endl;

    if( best_so_far.second != NULL ) {
        vector<LocalPath*> portion;
        portion.push_back( best_so_far.second );
        return replaceBegin( best_so_far.first, portion );
    }
    else {
        cout << "All paths are invalid" << endl;
        return false;
    }
}

//! Connects a configuration to the trajectory
//! it starts at the end of the trajectory
bool CostOptimization::connectConfigurationToEnd( confPtr_t q, double step, bool consider_cost )
{
    double param=step;
    double range_max = getRangeMax();
    std::pair<double,LocalPath*> best_so_far;
    best_so_far.first = 0.0;
    best_so_far.second = NULL;

    do
    {
        LocalPath* path = new LocalPath( configAtParam( range_max-param ), q );

        if( path->isValid() )
        {
            delete best_so_far.second;
            best_so_far.first = param;
            best_so_far.second = path;
        }
        else {
            delete path;
        }
        param += step;
    }
    while( param < range_max );

    if( best_so_far.second != NULL ) {
        vector<LocalPath*> portion;
        portion.push_back( best_so_far.second );
        return replaceEnd( range_max-best_so_far.first, portion );
    }
    else {
        cout << "All paths are invalid" << endl;
        return false;
    }
}

//! This is the main function to deform a trajectory
//! it iterativly modifies the current trajectory, with random perturbations
//! this function consisiton of n deformation rounds which sample a configuration
//! to create a deviation from the initial path, check its cost and
//! keep the best trajectory out of the current and newly created trajectory
//! @param nbIteration the number of iteration to go for
//! @param idRun the id of the run
void CostOptimization::runDeformation( int nbIteration, int idRun )
{
    if (this->getCourbe().empty()) {
        cout << "Trajectory is empty" << endl;
        return;
    }

    m_runId = idRun;
    m_MaxNumberOfIterations = nbIteration;

    //cout << "Before Deform : Traj cost = " << this->cost() << endl;
    if( global_costSpace == NULL )
    {
        cout << "global_costSpace not initialized!!!" << endl;
        return;
    }

    traj_convergence_with_time.clear();
    m_convergence_rate.clear();

    timeval tim;
    double ts(0.0);
    bool first_iteration = true;

    if( !global_rePlanningEnv )
    {
        gettimeofday(&tim, NULL);
        ts = tim.tv_sec+(tim.tv_usec/1000000.0);
        m_time = 0.0;
    }
    else {
        // Stores the init time in ts on first call
        m_time = global_rePlanningEnv->time_since_last_call( first_iteration , ts );
    }

    double costBeforeDeformation = this->cost();
    double initalRange = getRangeMax();

    if(PlanEnv->getBool(PlanParam::trajSaveCost))
        storeCostAndGain( costBeforeDeformation, costBeforeDeformation );

    if (PointsToDraw != NULL)
        delete PointsToDraw;

    PointsToDraw = new PointCloud;

    m_shortcutRatio = 4; // one out of 4
    m_GainOfIterations.clear();
    m_GainCost.clear();

    m_descent = PlanEnv->getBool(PlanParam::withDescent);
    m_Iteration=0;

    if( PlanEnv->getBool(PlanParam::trajNPoints) )
        m_currentCost = costNPoints( PlanEnv->getInt(PlanParam::nb_pointsOnTraj) );

    setSortedIndex();

    for ( ; !checkStopConditions(m_Iteration); m_Iteration++)
    {
        if( m_useAutoStep )
        {
            m_step = initalRange/PlanEnv->getDouble(PlanParam::MaxFactor);
        }

        if( !ENV.getBool(Env::isCostSpace) ) {
            resetIsValid();
        }

        double CurCost;

        if( PlanEnv->getBool(PlanParam::trajNPoints) )
        {
            CurCost = m_currentCost;
        }
        else {
            CurCost = cost();
        }

        //cout << "iteration : " << ith_deformation << " ";
        // When it's not the first iteration (ith_deformation is not a multiple of the shortcut ratio)
        if( m_Iteration==0 || (m_Iteration % m_shortcutRatio != 0) )
        {
            //cout << "oneLoopDeformRecompute" ;
            if( PlanEnv->getBool(PlanParam::trajCostRecompute) )
            {
                oneLoopDeformRecompute();
            }
            else {
                oneLoopDeform();
            }
        }
        else {
            // cout << "oneLoopShortCutRecompute" ;
            if( PlanEnv->getBool(PlanParam::trajCostRecompute) )
            {
                oneLoopShortCutRecompute();
            }
            else {
                oneLoopDeform();
            }
        }
        //cout << " , m_step = " << m_step << endl;

        // Then, compute success, gain and save cost to file
        double NewCost;

        if( PlanEnv->getBool(PlanParam::trajNPoints) )
        {
            NewCost = m_currentCost;
        }
        else {
            NewCost = cost();
        }

        traj_convergence_with_time.push_back(  make_pair( m_time, NewCost ) );

        m_IterationSucceded = ( CurCost > NewCost );

        if( m_IterationSucceded )
        {
            //cout << "new cost : " << NewCost << endl;
            if(PlanEnv->getBool(PlanParam::trajSaveCost))
            {
                gettimeofday(&tim, NULL);
                m_time = tim.tv_sec+(tim.tv_usec/1000000.0) - ts;

                storeCostAndGain( NewCost, CurCost );
            }

            if( (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) )
            {
                g3d_draw_allwin_active();
            }
        }

        if( global_rePlanningEnv )
        {
            //replan_store_traj_to_vect(*this,m_step);
            global_rePlanningEnv->store_traj_to_draw(*this,m_step);
            m_time += global_rePlanningEnv->time_since_last_call( first_iteration , ts );
        }
        else {
            gettimeofday(&tim, NULL);
            m_time = tim.tv_sec+(tim.tv_usec/1000000.0)-ts;
            //ChronoTimes( &m_time , &ts );
        }
    }

    //  if( !global_rePlanningEnv ) {
    //    ChronoOff();
    //  }

#ifdef DEBUG_STATUS
    if ( isValid() )
        cout << "Trajectory valid" << endl;
    else
        cout << "Trajectory not valid" << endl;
#endif

    if(PlanEnv->getBool(PlanParam::trajSaveCost))
    {
        stringstream s;
        s << "CostOptim_" << setfill('0') << setw(4) << idRun ;
        saveOptimToFile( s.str() );
    }

    cout << "Before : Traj cost = " << costBeforeDeformation << endl;
    this->resetCostComputed();
    cout << "After (" << m_Iteration << ") Deform : Traj cost = " << this->cost() << endl;
}
