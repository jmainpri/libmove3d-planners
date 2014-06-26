#include "HRICS_human_simulator.hpp"

#include "HRICS_play_motion.hpp"
#include "gestures/HRICS_gest_parameters.hpp"

#include "API/project.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/run_parallel_stomp.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/planEnvironment.hpp"

#include "collision_space/collision_space_factory.hpp"

#include <libmove3d/p3d/env.hpp>
#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Collision-pkg.h>

#include <iomanip>

using namespace HRICS;
using std::cout;
using std::endl;

HRICS::HumanTrajSimulator* global_ht_simulator = NULL;

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Human Trajectory Simulator

HumanTrajSimulator::HumanTrajSimulator( HumanTrajCostSpace* cost_space )  :
    cost_space_( cost_space ),
    human_active_( cost_space->getActiveHuman() ),
    human_passive_( cost_space->getPassiveHuman() ),
    init_scenario_(false)
{
    cout << "Human active : " << human_active_->getName() << endl;
    cout << "Human passive : " << human_passive_->getName() << endl;
}

bool HumanTrajSimulator::init()
{
//    if( !global_motionRecorders[0]->getStoredMotions().empty() )
//    {
//        cout << "Load init and goal from file" << endl;
//        const motion_t& motion_pas = global_motionRecorders[0]->getStoredMotions()[0];
//        const motion_t& motion_act = global_motionRecorders[1]->getStoredMotions()[0];

//        q_init_ = motion_act[0].second;
//        q_goal_ = motion_act.back().second;

//        // Adds the trajectory from the passive robot
//        // to the cost space
//        cost_space_->setPassiveTrajectory( motion_pas );
//    }
//    else
//    {
        q_init_ = human_active_->getInitPos();
        q_goal_ = human_active_->getGoalPos();
//    }

    // Set humans colors
    setHumanColor( human_active_, 3 );
    setHumanColor( human_passive_, 2 );

    // Sets the active robot as active for planning
    Move3D::global_Project->getActiveScene()->setActiveRobot( human_active_->getName() );

    // Change dofs limit
    // human_active_->se

    // Set init and goal config
    human_active_->setInitPos( *q_init_ );
    human_active_->setGoalPos( *q_goal_ );

    // Set to init pose
    human_active_->setAndUpdate( *q_init_ );

    // Set bounds and dofs
    setActiveJoints();

    // set the pointers to the motion recorder
    motion_recorders_ = global_motionRecorders;

    // Set minimal demonstration size
    minimal_demo_size_ = 10;
    trajectories_cut_ = false;

    // Store demonstrations, compute pelvis bounds
    // add cut motions
    setReplanningDemonstrations();
    addCutMotions();

    // Set the planning bounds
    setPelvisBounds();

    // Set the current frame to 0
    current_frame_ = 0;

    return true;
}

void HumanTrajSimulator::setReplanningDemonstrations()
{
    std::vector<std::string> good_motions_names;
    good_motions_names.push_back( "[0551-0602]motion_saved_00000_00000.csv" );
    good_motions_names.push_back( "[1186-1245]motion_saved_00000_00001.csv" );
    good_motions_names.push_back( "[1552-1581]motion_saved_00000_00000.csv" );
    good_motions_names.push_back( "[1873-1929]motion_saved_00000_00001.csv" );
    good_motions_names.push_back( "[3191-3234]motion_saved_00000_00000.csv" );
    good_motions_names.push_back( "[3913-3950]motion_saved_00000_00000.csv" );
    good_motions_names.push_back( "[4125-4169]motion_saved_00000_00001.csv" );
    good_motions_names.push_back( "[4422-4476]motion_saved_00000_00000.csv" );
    good_motions_names.push_back( "[4591-4640]motion_saved_00000_00000.csv" );
    good_motions_names.push_back( "[4753-4802]motion_saved_00000_00000.csv" );

    pelvis_max_ = Eigen::VectorXd::Zero(6);
    pelvis_min_ = Eigen::VectorXd::Zero(6);
    bool initialized = false;

    for( size_t k=0; k<good_motions_names.size(); k++)
    {
        for(size_t j=0; j<motion_recorders_[0]->getStoredMotions().size(); j++)
        {
            if( motion_recorders_[0]->getStoredMotionName(j) == good_motions_names[k] )
            {
                cout << "Add motion : " << good_motions_names[k] << endl;
                human_1_motions_.push_back( motion_recorders_[0]->getStoredMotions()[j] );
                human_2_motions_.push_back( motion_recorders_[1]->getStoredMotions()[j] );

                for( size_t s=0; s<human_2_motions_.back().size(); s++) // Only set active dofs on the configuration
                {
                    Move3D::confPtr_t q = human_2_motions_.back()[s].second;
                    Move3D::confPtr_t q_tmp = q->getRobot()->getInitPos();
                    q_tmp->setFromEigenVector( q->getEigenVector(active_dofs_), active_dofs_ );
                    human_2_motions_.back()[s].second = q_tmp;

                    if(s == 0)
                    {
                        for( size_t i=0; i<6; i++)
                        {
                            if( initialized )
                            {
                                if( pelvis_max_[i] < (*q_tmp)[6+i] )
                                    pelvis_max_[i] = (*q_tmp)[6+i];
                                if( pelvis_min_[i] > (*q_tmp)[6+i] )
                                    pelvis_min_[i] = (*q_tmp)[6+i];
                            }
                            else
                            {
                                pelvis_max_[i] = (*q_tmp)[6+i];
                                pelvis_min_[i] = (*q_tmp)[6+i];
                            }
                        }
                        initialized = true;
                    }
                }
            }
        }
    }
}

std::vector< std::vector<motion_t> > HumanTrajSimulator::getMotions()
{
    std::vector< std::vector<motion_t> > motions;
    motions.push_back( human_1_motions_ );
    motions.push_back( human_2_motions_ );
    return motions;
}

void HumanTrajSimulator::addCutMotions()
{
    cout << "size 1 before adding cut motions : " << human_1_motions_.size() << endl;
    cout << "size 2 before adding cut motions : " << human_2_motions_.size() << endl;

    std::vector<motion_t> human_1_motion_tmp;
    std::vector<motion_t> human_2_motion_tmp;

    for( size_t i=0; i<human_2_motions_.size(); i++ )
    {
        if( human_2_motions_[i].empty() )
            continue;
        if( human_2_motions_[i].size() != human_1_motions_[i].size() )
            continue;

        int max_nb_of_removed_frames = int(human_2_motions_[i].size()) - minimal_demo_size_;

        motion_t::const_iterator init_1 = human_1_motions_[i].begin();
        motion_t::const_iterator goal_1 = human_1_motions_[i].end();

        motion_t::const_iterator init_2 = human_2_motions_[i].begin();
        motion_t::const_iterator goal_2 = human_2_motions_[i].end();

        cout << "motion 2 size : " << human_2_motions_[i].size() << endl;
        cout << "min demo size : " << minimal_demo_size_ << endl;
        cout << "max_nb_of_removed_frames : " << max_nb_of_removed_frames << endl;

        // Add smaler cut trajectories on smaller chunks
        for( int j=0; j<max_nb_of_removed_frames; j++ )
        {
            motion_t motion_1( init_1++, goal_1 );
            motion_t motion_2( init_2++, goal_2 );

            human_1_motion_tmp.push_back( motion_1 );
            human_2_motion_tmp.push_back( motion_2 );
        }

        trajectories_cut_ = true;
    }

    if( trajectories_cut_ )
    {
        human_1_motions_ = human_1_motion_tmp;
        human_2_motions_ = human_2_motion_tmp;
    }

    cout << "size 1 after adding cut motions : " << human_1_motions_.size() << endl;
    cout << "size 2 after adding cut motions : " << human_2_motions_.size() << endl;
}

std::vector<Move3D::Trajectory> HumanTrajSimulator::getDemoTrajectories() const
{
    std::vector<Move3D::Trajectory> trajs;

    if( human_2_motions_.empty() )
        return trajs;

    for( size_t i=0; i<human_2_motions_.size(); i++ )
    {
        if( human_2_motions_[i].empty() )
            continue;

        Move3D::Robot* robot = human_2_motions_[i][0].second->getRobot();
        trajs.push_back( motion_to_traj( human_2_motions_[i], robot ) );
    }

    return trajs;
}

std::vector<Move3D::confPtr_t> HumanTrajSimulator::getContext() const
{
    std::vector<Move3D::confPtr_t> context;

    for( size_t i=0; i<human_1_motions_.size(); i++ )
    {
        if( human_1_motions_[i].empty() )
            continue;

        context.push_back( human_1_motions_[i][0].second->copy() );
    }

    return context;
}

void HumanTrajSimulator::setPelvisBounds()
{
    // Get first joint and change bounds
    Move3D::Joint* joint = human_active_->getJoint( "Pelvis" );

    bool use_user_defined = false;

    double dof[6][2];

    if( use_user_defined )
    {
        // take only (x, y, z) components of the base
        double bound_trans = 0.05;
        double bound_rotat = 0.1;

        for(int i = 0; i < 3; i++) { // Translation bounds
            dof[i][0] = joint->getJointDof(i) - bound_trans;
            dof[i][1] = joint->getJointDof(i) + bound_trans;
            cout << "PELVIS DOF: " << joint->getJointDof(i) << endl;
        }
        for(int i = 3; i < 6; i++) { // Rotation bounds
            dof[i][0] = joint->getJointDof(i) - bound_rotat;
            dof[i][1] = joint->getJointDof(i) + bound_rotat;
        }
    }
    else
    {
        double bound_trans = 0.02;
        double bound_rotat = 0.05;

        for(int i = 0; i < 3; i++) { // Translation bounds
            dof[i][0] = pelvis_min_(i) - bound_trans;
            dof[i][1] = pelvis_max_(i) + bound_trans;
        }
        for(int i = 3; i < 6; i++) { // Rotation bounds
            dof[i][0] = pelvis_min_(i) - bound_rotat;
            dof[i][1] = pelvis_max_(i) + bound_rotat;
        }
    }

    for(int i = 0; i < 6; i++){
        p3d_jnt_set_dof_rand_bounds( joint->getP3dJointStruct(), i, dof[i][0], dof[i][1] );
    }

//    Joint(0), Dof : 6, Pelvis
//    Is dof user : (min = 0.94, max = 1.04)
//    Joint(1), Dof : 7, Pelvis
//    Is dof user : (min = 0.61, max = 0.71)
//    Joint(2), Dof : 8, Pelvis
//    Is dof user : (min = 0.92, max = 1.02)
//    Joint(3), Dof : 9, Pelvis
//    Is dof user : (min = -0.10, max = 0.10)
//    Joint(4), Dof : 10, Pelvis
//    Is dof user : (min = -0.10, max = 0.10)
//    Joint(5), Dof : 11, Pelvis

//    Is dof user : (min = -0.77, max = -0.57)
//    Joint(0), Dof : 12, TorsoX
//    Is dof user : (min = -0.79, max = 0.79)
//    Joint(0), Dof : 13, TorsoY
//    Is dof user : (min = -0.35, max = 0.70)
//    Joint(0), Dof : 14, TorsoZ

//    Is dof user : (min = -0.79, max = 0.79)
//    Joint(0), Dof : 18, rShoulderX
//    Is dof user : (min = -3.14, max = 3.14)
//    Joint(0), Dof : 19, rShoulderZ
//    Is dof user : (min = -3.14, max = 3.14)
//    Joint(0), Dof : 20, rShoulderY

//    Is dof user : (min = -3.14, max = 3.14)
//    Joint(0), Dof : 22, rElbowZ
//    Is dof user : (min = -3.14, max = 3.14)
}

void HumanTrajSimulator::setActiveJoints()
{
    active_joints_.push_back( human_active_->getJoint( "Pelvis" ) ); // Pelvis

    active_joints_.push_back( human_active_->getJoint( "TorsoX" ) ); // TorsoX
    active_joints_.push_back( human_active_->getJoint( "TorsoY" ) ); // TorsoY
    active_joints_.push_back( human_active_->getJoint( "TorsoZ" ) ); // TorsoZ

    active_joints_.push_back( human_active_->getJoint( "rShoulderX" ) ); // rShoulderX
    active_joints_.push_back( human_active_->getJoint( "rShoulderZ" ) ); // rShoulderZ
    active_joints_.push_back( human_active_->getJoint( "rShoulderY" ) ); // rShoulderY

    active_joints_.push_back( human_active_->getJoint( "rArmTrans" ) ); // rArmTrans

    active_joints_.push_back( human_active_->getJoint( "rElbowZ" ) ); // rElbowZ
//    active_joints_.push_back(14); // joint name : rWristX
//    active_joints_.push_back(15); // joint name : rWristY
//    active_joints_.push_back(16); // joint name : rWristZ

    // SET COST SPACE ACTIVE DOFS

    active_dofs_.clear();

    for( size_t i=0; i<active_joints_.size(); i++ ) // get all active dofs
    {
        std::vector<unsigned int> jnt_active_dofs = active_joints_[i]->getDofIndices();
        active_dofs_.insert( active_dofs_.end(), jnt_active_dofs.begin(), jnt_active_dofs.end() );
    }

    for( size_t i=0; i<cost_space_->getNumberOfFeatureFunctions(); i++ ) // set all features active dofs
    {
        cost_space_->getFeatureFunction(i)->setActiveDoFs( active_dofs_ );
    }
}

void HumanTrajSimulator::setHumanColor(Move3D::Robot* human, int color)
{
    if( !human->getUseLibmove3dStruct() )
        return;

    for (int i=1; i<= human->getP3dRobotStruct()->njoints; i++)
    {
        p3d_obj* obj = human->getP3dRobotStruct()->joints[i]->o;

        if(obj == NULL)
            continue;

        for(int j = 0; j < obj->np; j++)
        {
            p3d_poly* poly = obj->pol[j];

            if(poly->color_vect == NULL)
                continue;

            if (         poly->color_vect[0] == 0.80
                      && poly->color_vect[1] == 0.00
                      && poly->color_vect[2] == 0.01) {

                double color_vect[4];

                g3d_get_color_vect(color, color_vect);

                poly->color = color;

                poly->color_vect[0] = color_vect[0];
                poly->color_vect[1] = color_vect[1];
                poly->color_vect[2] = color_vect[2];
                poly->color_vect[3] = color_vect[3];
            }
        }
    }
}

bool HumanTrajSimulator::updateMotion()
{
    // Advance on passive trajectory
    current_frame_ += human_passive_increment_;

    // If the current frame is farther than current motion
    // Return false and stop the simulation
    if( current_frame_ > int(human_passive_motion_.size()))
        return false;

    Move3D::confPtr_t q_cur = human_passive_motion_[ current_frame_ ].second;
    human_passive_->setAndUpdate(*q_cur);

    return true;
}

bool HumanTrajSimulator::loadActiveHumanGoalConfig()
{
    if( id_of_demonstration_ >= human_2_motions_.size()  )
        return false;

    q_init_ = human_2_motions_[ id_of_demonstration_ ][0].second;
    q_goal_ = human_2_motions_[ id_of_demonstration_ ].back().second;

    human_active_increments_per_exection_ = 10;
    human_active_step_ =  q_init_->dist( *q_goal_ ) / 200;

    return true;
}

Move3D::Trajectory HumanTrajSimulator::setTrajectoryToMainRobot(const Move3D::Trajectory& traj) const
{
    Move3D::Trajectory traj_tmp(human_active_);

    for(int i=0;i<traj.getNbOfViaPoints();i++)
    {
        Move3D::confPtr_t q(new Move3D::Configuration(human_active_));
        q->setConfiguration( traj[i]->getConfigStructCopy() );
        traj_tmp.push_back( q );
    }

    return traj_tmp;
}

void HumanTrajSimulator::runStandardStomp( int iter )
{
    PlanEnv->setInt( PlanParam::nb_pointsOnTraj, 100 );
//    traj_optim_set_discretize( true );
//    traj_optim_set_discretization( 0.015 );

    human_active_->setAndUpdate( *q_init_ );

    if( iter>0 )
    {
        const Move3D::Trajectory& current_traj = path_;
        const double parameter = human_active_increments_per_exection_*human_active_step_;
        Move3D::Trajectory optimi_traj;

//        if( id_goal == best_path_id_ )
//        {
            optimi_traj = current_traj.extractSubTrajectory( parameter, path_.getParamMax(), false );
//        }
//        else
//        {
//            Move3D::CostOptimization traj( paths_[id_goal] );
//            traj.connectConfigurationToBegin( path_.configAtParam( parameter ), parameter/5, true );
//            optimi_traj = traj;
//        }

        traj_optim_set_use_extern_trajectory( true );
        traj_optim_set_extern_trajectory( optimi_traj );
    }
    else
    {
        human_active_->setInitPos( *q_init_ );
        human_active_->setGoalPos( *q_goal_ );
        traj_optim_set_use_extern_trajectory( false );
    }

    traj_optim_set_use_iteration_limit(true);
    traj_optim_set_iteration_limit( PlanEnv->getInt(PlanParam::stompMaxIteration) );
    traj_optim_runStomp(0);

    path_ = global_optimizer->getBestTraj();
}

void HumanTrajSimulator::execute(const Move3D::Trajectory& path, bool to_end)
{
    double s = 0.0;

    path.replaceP3dTraj();

    Move3D::confPtr_t q;

    for( int i=0;
         (!to_end) ?
         ((i<human_active_increments_per_exection_) && (!PlanEnv->getBool(PlanParam::stopPlanner) )) :
         (s<path.getParamMax() && (!PlanEnv->getBool(PlanParam::stopPlanner)));
         i++ )
    {
        q = path.configAtParam( s ); executed_path_.push_back( q );
        human_active_->setAndUpdate( *q );
        g3d_draw_allwin_active();
        usleep(25000);
        s += human_active_step_;
    }

    q_init_ = q;
}

void HumanTrajSimulator::printCosts() const
{
//    for(int i =0;i<int(cost_.size());i++)
//    {
//        for(int j =0;j<int(cost_[i].size());j++)
//        {
//            cout << "cost_" << i << "_(" << j+1 << ") = " <<  cost_[i][j] << ";" <<endl;
//        }
//    }
}

double HumanTrajSimulator::run()
{
    current_human_traj_.resize( 0, 0 );
    executed_path_ = Move3D::Trajectory( human_active_ );
    best_path_id_ = -1;
    current_frame_ = 0;
    human_passive_increment_ = 1;

    id_of_demonstration_ = 0; // TODO add global variable

    if( !loadActiveHumanGoalConfig() ) {
        cout << "Error could not load active human start and goal configurations" << endl;
        return 0.0;
    }

    // Set human passive motion
    human_passive_motion_ = human_1_motions_[ id_of_demonstration_ ];

    cout << "Load human traj id  : " << GestEnv->getInt(GestParam::human_traj_id) << endl;

    // TODO REPLACE BY TRAJECTORY
    // loadHumanTrajectory( m_recorder->getStoredMotions()[GestEnv->getInt(GestParam::human_traj_id)] );

    cost_.clear();
    human_active_->setAndUpdate( *q_init_ );

    ENV.setBool( Env::isCostSpace, true );

    for(int i=0;(!PlanEnv->getBool(PlanParam::stopPlanner)) && updateMotion(); i++ )
    {
        g3d_draw_allwin_active();

        runStandardStomp( i );

        if( !PlanEnv->getBool(PlanParam::stopPlanner) )
        {
            execute( path_, false );
        }
    }

    if( !PlanEnv->getBool(PlanParam::stopPlanner) )
    {
        const double parameter =  double(human_active_increments_per_exection_) * human_active_step_;
        execute( path_.extractSubTrajectory( parameter, path_.getParamMax(), false ), true );
    }

    ENV.setBool( Env::isCostSpace, true );

    printCosts();

    cout << "executed_path_.cost() : " << executed_path_.cost() << endl;
    executed_path_.replaceP3dTraj();
    return executed_path_.cost();
}
