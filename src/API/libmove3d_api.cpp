#include "libmove3d_api.hpp"

#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/localpath.hpp"
#include "API/Device/robot.hpp"
#include "API/Device/joint.hpp"
#include "API/Trajectory/trajectory.hpp"
#include "API/Graphic/drawModule.hpp"
#include "API/scene.hpp"

#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Localpath-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"
#include "move3d-gui.h"
#include "move3d-headless.h"

#include <boost/bind.hpp>

using namespace std;
using namespace Eigen;
using namespace Move3D;

extern void* move3d_localpath_get_localpath_struct( LocalPath& path, bool multi_sol, int& type );
extern confPtr_t move3d_localpath_config_at_param( LocalPath& path, double param );
extern bool move3d_localpath_is_valid_test( LocalPath& path, int& nb_col_check );

// ****************************************************************************************************
// ****************************************************************************************************
//
// CONFIGURATION
//
// ****************************************************************************************************
// ****************************************************************************************************

double* move3d_configuration_constructor(Robot* robot)
{
    return p3d_alloc_config((p3d_rob*)robot->getP3dRobotStruct());
}

double* move3d_configuration_constructor_configstruct(Robot* R, double* C, bool noCopy)
{
    double* q;

    if(C==NULL)
    {
        q = C;
    }
    else
    {
        q = noCopy ? C : p3d_copy_config( (p3d_rob*)R->getP3dRobotStruct(), C );
        // this->initQuaternions();
    }
    return q;
}

void move3d_configuration_assignment( const Configuration& q_source, Configuration& q_target )
{
    double* q = q_target.getConfigStruct();
    p3d_copy_config_into( (p3d_rob*) q_target.getRobot()->getP3dRobotStruct(), q_source.getConfigStructConst(), &q ) ;
}

void move3d_configuration_clear( Robot* R, double* C )
{
    p3d_destroy_config( (p3d_rob*) R->getP3dRobotStruct(), C );
}

void move3d_configuration_convert_to_radian( Robot* R, double* C )
{
    configPt q = p3d_alloc_config( (p3d_rob*)R->getP3dRobotStruct());
    p3d_convert_config_deg_to_rad( (p3d_rob*)R->getP3dRobotStruct(),C,&q);
    p3d_destroy_config( (p3d_rob*)R->getP3dRobotStruct(),C);
    C = q;
}

confPtr_t move3d_configuration_convert_to_degrees( Robot* R, double* C )
{
    configPt q = p3d_alloc_config( (p3d_rob*)R->getP3dRobotStruct());
    p3d_convert_config_rad_to_deg( (p3d_rob*)R->getP3dRobotStruct(),C,&q);
    return (confPtr_t (new Configuration(R,q,true)));
}

double* move3d_configuration_copy_struct( Robot* R, double* C )
{
    return p3d_copy_config( (p3d_rob*)R->getP3dRobotStruct(), C );
}

double move3d_configuration_dist( Robot* R, double* q1, double* q2, bool print )
{
    // p3d_dist_config( (p3d_rob*)_Robot->getP3dRobotStruct(), _Configuration, Conf._Configuration , print );
    return p3d_dist_config( (p3d_rob*)R->getP3dRobotStruct(), q1, q2, print );
}

double move3d_configuration_dist_choice( Robot* R, const Configuration& q1, const Configuration& q2, int dist_choice )
{
    switch (dist_choice)
    {
    case ACTIVE_CONFIG_DIST:
        return (p3d_ActiveDistConfig( (p3d_rob*)R->getP3dRobotStruct(), q1.getConfigStructConst(), q2.getConfigStructConst()));
        //  case LIGAND_PROTEIN_DIST:
        //    return(bio_compute_ligand_dist(_Robot->getP3dRobotStruct(), _Configuration, q.getConfigurationStruct()));
        //    break;
    case MOBILE_FRAME_DIST:
        cout
                << "Warning: the MOBILE_FRAME_DIST can't be directly returned from the configurations"
                << endl;
        // hrm_mob_frame_dist(robotPt, mob_frame_ref,ListNode->N->rel_mob_frame);
#ifdef LIGHT_PLANNER
    case ONLY_ROBOT_BASE:
    {

        p3d_rob* robotPt = (p3d_rob*) R->getP3dRobotStruct();
        double ljnt=0.0;
        p3d_jnt* jntPt= (p3d_jnt*) R->getBaseJnt();
        for(int j=0; j<jntPt->dof_equiv_nbr; j++)
        {
            int k = jntPt->index_dof + j;

            if ( (p3d_jnt_get_dof_is_user(jntPt, j) && p3d_jnt_get_dof_is_active_for_planner(jntPt,j)) && (robotPt->cntrt_manager->in_cntrt[k] != 2) )
            {
                ljnt += SQR(p3d_jnt_calc_dof_dist(jntPt, j, q1.getConfigStructConst(), q2.getConfigStructConst() ));
            }
        }
        return ljnt;
    }
#endif
    case GENERAL_CSPACE_DIST:
    default:
        return (q1.dist(q2));
    }
}

bool move3d_configuration_in_collision(Robot* R)
{
    return p3d_col_test_robot( (p3d_rob*)R->getP3dRobotStruct(), JUST_BOOL );
}

bool move3d_configuration_is_out_of_bounds( Robot* R, double* C, bool print )
{
    return p3d_isOutOfBounds( (p3d_rob*)R->getP3dRobotStruct(), C, print);
}

void move3d_configuration_adapt_to_circular_joints( Robot* R, double* C )
{
    configPt q = p3d_alloc_config( (p3d_rob*)R->getP3dRobotStruct() );
    p3d_adaptConfigsForCircularDofs( (p3d_rob*)R->getP3dRobotStruct(), &C, &q );
    p3d_destroy_config( (p3d_rob*)R->getP3dRobotStruct(), q );
}

double move3d_configuration_dist_env(Robot* R)
{
    int settings = get_kcd_which_test();
    set_kcd_which_test((p3d_type_col_choice) (40 + 3));
    // 40 = KCD_ROB_ENV
    // 3 = DISTANCE_EXACT
    p3d_col_test_choice();
    // Collision detection with other robots only

    int nof_bodies = static_cast<p3d_rob*>( R->getP3dRobotStruct() )->no;

    double* distances = new double[nof_bodies];

    p3d_vector3 *body = new p3d_vector3[nof_bodies];
    p3d_vector3 *other = new p3d_vector3[nof_bodies];

    p3d_kcd_closest_points_robot_environment( (p3d_rob*)R->getP3dRobotStruct(), body, other, distances );
    // Get robot closest points to human for each body

    set_kcd_which_test((p3d_type_col_choice) settings);

    int i = (int)(std::min_element( distances, distances+nof_bodies-1 )-distances);
    double min_dist = distances[i];
    delete distances;

    return min_dist;
}

bool move3d_configuration_equal( Robot* R, double* C, double* q, bool print )
{
    return p3d_equal_config( (p3d_rob*)R->getP3dRobotStruct(), C, q, print );
}

void move3d_configuration_copy_passive( Robot* R, double* C, double* q )
{
    for( int i(0); i <= static_cast<p3d_rob*>(R->getP3dRobotStruct())->njoints; i++ )
    {
        p3d_jnt* joint( static_cast<p3d_rob*>(R->getP3dRobotStruct())->joints[i]);
        for (int j(0); j < joint->dof_equiv_nbr; j++)
        {
            int k = joint->index_dof + j;
            if ( (!p3d_jnt_get_dof_is_user(joint, j)) || (!p3d_jnt_get_dof_is_active_for_planner(joint, j) ))
                q[k] = C[k];
        }
    }
}

void move3d_configuration_add( Robot* R, double* C, double* q1, double* q2 )
{
    p3d_addConfig( R->getP3dRobotStruct(), C, q1, q2 );
}

void move3d_configuration_sub( Robot* R, double* C, double* q1, double* q2 )
{
    p3d_subConfig( R->getP3dRobotStruct(), C, q1, q2 );
}

void move3d_configuration_mult( Robot* R, double* C, double* q2, double coeff )
{
    p3d_rob* robotPt = (p3d_rob*)R->getP3dRobotStruct();

    int njnt = robotPt->njoints;
    int k = 0;
    for (int i = 0; i <= njnt; i++)
    {
        p3d_jnt *jntPt = robotPt->joints[i];
        for (int j = 0; j < jntPt->dof_equiv_nbr; j++)
        {
            q2[k] *= coeff;
            k ++;
        }
    }
}

void move3d_configuration_print( Robot* R, double* C, bool withPassive )
{
    cout << "Print Configuration; Robot: " << R->getP3dRobotStruct() << endl;

    //	print_config(_Robot->getP3dRobotStruct(),_Configuration);

    //    configPt degConf = getConfigInDegree()->getConfigStruct();
    //
    //    for (int i = 0; i < _Robot->getP3dRobotStruct()->nb_dof; i++)
    //    {
    //        //	    cout << "q["<<i<<"]"<<" = "<< _Configuration[i] << endl;
    //        cout << "degConf["<< i <<"] = " << degConf[i] << endl;
    //    }

    //	int nb_dof;
    //
    //	if(robotPt != NULL){
    //		nb_dof = mR->getP3dRob()->nb_user_dof;
    //	}

    //	for(int i=0; i<nb_dof;i++){
    //		PrintInfo(("q[%d] = %f\n", i, q[i]));
    //	}

    p3d_rob* robotPt = (p3d_rob*) R->getP3dRobotStruct();

    int njnt = robotPt->njoints, k;
    p3d_jnt * jntPt;
    for(int i=0; i<=njnt; i++)
    {
        jntPt = robotPt->joints[i];
        for(int j=0; j<jntPt->dof_equiv_nbr; j++)
        {
            k = jntPt->index_dof + j;
            //      cout << "p3d_jnt_get_dof_is_user : " << p3d_jnt_get_dof_is_user(jntPt, j) << endl;
            //      cout << "_Robot->getP3dRobotStruct()->cntrt_manager->in_cntrt[" << k << "] = " << _Robot->getP3dRobotStruct()->cntrt_manager->in_cntrt[k] << endl;
            if (withPassive || ( p3d_jnt_get_dof_is_user(jntPt, j)
                                 /*&& (p3d_jnt_get_dof_is_active_for_planner(jntPt,j) */
                                 && (robotPt->cntrt_manager->in_cntrt[k] != DOF_PASSIF )))
            {
                cout << "q["<<k<<"] = "<< C[k] <<endl;
            }
        }
    }
    cout << "\n--------------------------------" << endl;
}

// ****************************************************************************************************
// ****************************************************************************************************
//
// LOCAL PATHS
//
// ****************************************************************************************************
// ****************************************************************************************************

void* move3d_localpath_copy(const LocalPath& path, Robot* R)
{
    return path.getP3dLocalpathStructConst() ? path.getP3dLocalpathStructConst()->copy( (p3d_rob*) R->getP3dRobotStruct(), path.getP3dLocalpathStructConst()) : NULL;
}

void move3d_localpath_destructor( LocalPath& path )
{
    if ( path.getP3dLocalpathStructConst() )
    {
        path.getP3dLocalpathStructConst()->destroy( path.getRobot()->getP3dRobotStruct(), path.getP3dLocalpathStructConst());
    }
}

void* move3d_localpath_get_localpath_struct( LocalPath& path, bool multi_sol, int& type )
{
    p3d_localpath* path_struct = NULL;

    if ( !path.getP3dLocalpathStructConst() )
    {
        if( !multi_sol )
        {
            path_struct = p3d_local_planner( (p3d_rob*) path.getRobot()->getP3dRobotStruct(), path.getBegin()->getConfigStruct(), path.getEnd()->getConfigStruct() );
        }
        else
        {
            path_struct = p3d_local_planner_multisol( (p3d_rob*) path.getRobot()->getP3dRobotStruct(), path.getBegin()->getConfigStruct(), path.getEnd()->getConfigStruct(), path.getIkSol() );
        }

        if ( path_struct )
        {
            type = path_struct->type_lp;
        }
    }
    else {
        path_struct = path.getP3dLocalpathStructConst();
    }

    return path_struct;
}

void* move3d_localpath_init_from_struct( LocalPath& path, void* lpPtr, confPtr_t& begin, confPtr_t& end )
{
    p3d_localpath* path_struct = NULL;

    // TODO : check which copy are/are not necessary.
    if (lpPtr)
    {
        path_struct = static_cast<p3d_localpath*>(lpPtr)->copy( static_cast<p3d_rob*>( path.getRobot()->getP3dRobotStruct()), static_cast<p3d_localpath*>(lpPtr));

        begin = confPtr_t (
                    new Configuration( path.getRobot(), path_struct->config_at_param(
                                           static_cast<p3d_rob*>(path.getRobot()->getP3dRobotStruct()),
                                           path_struct, 0),true));

        begin->setConstraints();

        end = confPtr_t (
                    new Configuration( path.getRobot(), path_struct->config_at_param(
                                           static_cast<p3d_rob*>(path.getRobot()->getP3dRobotStruct()),
                                           path_struct,
                                           path_struct->range_param),true));

        end->setConstraints();

        path.setType( static_cast<p3d_localpath*>(lpPtr)->type_lp );
    }
    else
    {
        cout << "Warning : creating Localpath from uninitialized p3d_localpath"
             << endl;
    }

    return path_struct;
}

bool move3d_localpath_classic_test( LocalPath& path, confPtr_t q_last_valid, int& nb_col_check, double& last_valid_param )
{
    double* q = q_last_valid->getConfigStruct();
    double** q_atKpath = &q;

    return !p3d_unvalid_localpath_classic_test( (p3d_rob*)path.getRobot()->getP3dRobotStruct(), path.getP3dLocalpathStruct(),
                                                &nb_col_check, &last_valid_param, q_atKpath );
}

bool move3d_localpath_is_valid_test( LocalPath& path, int& nb_col_check )
{
    return !p3d_unvalid_localpath_test( (p3d_rob*)path.getRobot()->getP3dRobotStruct(), path.getP3dLocalpathStruct(), &nb_col_check);
}

double move3d_localpath_lenth( LocalPath& path )
{
    if( path.getP3dLocalpathStruct() )
    {
        return path.getP3dLocalpathStruct()->length_lp;
    }
    else
    {
        if( *path.getBegin() == *path.getEnd() )
            return 0;
        else
        {
            std::cout << "ERROR : in Localpath::length() : this->getP3dLocalpathStruct() is NULL" << std::endl;
            return(-1);
        }
    }
}

double move3d_localpath_max_param( LocalPath& path )
{
    if( *path.getBegin() == *path.getEnd() )
    {
        return 0;
    }

    return path.getP3dLocalpathStruct()->range_param;
}

confPtr_t move3d_localpath_config_at_dist( LocalPath& path, double dist )
{
    //fonction variable en fonction du type de local path
    configPt q(NULL);
    switch (path.getType())
    {
    case HILFLAT://hilare
        q = p3d_hilflat_config_at_distance( (p3d_rob*)path.getRobot()->getP3dRobotStruct(), path.getP3dLocalpathStruct(), dist);
        break;
    case LINEAR://linear
        q = p3d_lin_config_at_distance( (p3d_rob*)path.getRobot()->getP3dRobotStruct(), path.getP3dLocalpathStruct(), dist);
        break;
    case MANHATTAN://manhatan
        q = p3d_manh_config_at_distance( (p3d_rob*)path.getRobot()->getP3dRobotStruct(), path.getP3dLocalpathStruct(), dist);
        break;
    case REEDS_SHEPP://R&S
        q = p3d_rs_config_at_distance( (p3d_rob*)path.getRobot()->getP3dRobotStruct(), path.getP3dLocalpathStruct(), dist);
        break;
    case TRAILER:
        q = p3d_trailer_config_at_distance( (p3d_rob*)path.getRobot()->getP3dRobotStruct(), path.getP3dLocalpathStruct(), dist);
        break;
    default:
        // TODO : implement those methods !
        std::cout << "ERROR : LocalPath::configAtDist : the TRAILER_FORWARD, HILFLAT_FORWARD, and DUBINS localpath types are not implemented." << std::endl;
    }
    return confPtr_t(new Configuration(path.getRobot(), q));
}

confPtr_t move3d_localpath_config_at_param( LocalPath& path, double param )
{
    //fonction variable en fonction du type de local path
    configPt q;

    if( param > path.getParamMax() )
    {
        return path.getEnd();
    }
    if( param <= 0 )
    {
        return path.getBegin();
    }

    q = path.getP3dLocalpathStruct()->config_at_param( static_cast<p3d_rob*>(path.getRobot()->getP3dRobotStruct()), path.getP3dLocalpathStruct(), param);

    if( q == NULL )
    {
        throw string("Could not find configuration along path");
    }

    confPtr_t ptrQ(new Configuration( path.getRobot(), q, true ));
    ptrQ->setConstraints();
    return ptrQ;
}

double move3d_stay_within_dist( LocalPath& path, double u, bool goForward, double distance )
{
    int way;

    if (goForward) {
        way = 1;
    }
    else {
        way = -1;
    }

    double du = path.getP3dLocalpathStruct()->stay_within_dist( static_cast<p3d_rob*>( path.getRobot()->getP3dRobotStruct()), path.getP3dLocalpathStruct(), u, way, &distance);
    return du;
}

// ****************************************************************************************************
// ****************************************************************************************************
//
// ROBOT
//
// ****************************************************************************************************
// ****************************************************************************************************

void* move3d_robot_constructor( Robot* R, void* robotPt, unsigned int& nb_dofs, bool copy, std::string& name, std::vector<Joint*>& joints )
{
    void* robot;

    if (copy) {
        robot = copyRobStructure( static_cast<p3d_rob*>(robotPt) );
    }
    else {
        robot = static_cast<p3d_rob*>(robotPt);
    }

    name = static_cast<p3d_rob*>(robotPt)->name;

    joints.clear();

    for (int i=0; i<=static_cast<p3d_rob*>(robot)->njoints; i++)
    {
        joints.push_back( new Joint( R, static_cast<p3d_rob*>(robot)->joints[i] , i , copy ) );
    }

    nb_dofs = static_cast<p3d_rob*>(robotPt)->nb_dof;

    return robot;
}

Move3D::Trajectory move3d_robot_get_current_trajectory(Robot* R)
{
    Move3D::Trajectory traj( R, static_cast<p3d_rob*>(R->getP3dRobotStruct())->tcur );
    return traj;
}

confPtr_t move3d_robot_shoot( confPtr_t q, bool sample_passive )
{
    Robot* R = q->getRobot();

#ifdef LIGHT_PLANNER
    if(ENV.getBool(Env::FKShoot))
    {
        R->deactivateCcConstraint();
        p3d_shoot( static_cast<p3d_rob*>( R->getP3dRobotStruct() ), q->getConfigStruct(), false);
        R->setAndUpdate(*q);
        q = R->getCurrentPos();
        R->activateCcConstraint();
        //    cout << "FKShoot" <<endl;
        return q;
    }
    else
    {
        p3d_shoot( static_cast<p3d_rob*>(R->getP3dRobotStruct()), q->getConfigStruct(), sample_passive );
        return q;
    }
#else
    p3d_shoot( static_cast<p3d_rob*>(R->getP3dRobotStruct()), q->getConfigStruct(), sample_passive );
    return q;
#endif
}

bool move3d_robot_set_and_update( Robot* R, const Configuration& q, bool without_free_flyers )
{
    if (!without_free_flyers) {
        p3d_set_robot_config(static_cast<p3d_rob*>( R->getP3dRobotStruct() ), q.getConfigStructConst());
    } else {
        p3d_set_robot_config_without_free_flyers(static_cast<p3d_rob*>( R->getP3dRobotStruct() ), q.getConfigStructConst());
    }
    return p3d_update_this_robot_pos(static_cast<p3d_rob*>( R->getP3dRobotStruct() ));
}

bool move3d_robot_set_and_update_multisol( Robot* R, const Configuration& q )
{
    int *ikSol = NULL;
    p3d_set_robot_config(static_cast<p3d_rob*>(R->getP3dRobotStruct()), q.getConfigStructConst());
    return p3d_update_this_robot_pos_multisol(static_cast<p3d_rob*>(R->getP3dRobotStruct()), NULL, 0, ikSol);
}

void move3d_robot_set_and_update_with_constraints( Robot* R, const Configuration& q )
{
    p3d_set_robot_config(static_cast<p3d_rob*>(R->getP3dRobotStruct()), q.getConfigStructConst());
    p3d_update_this_robot_pos_without_cntrt( static_cast<p3d_rob*>(R->getP3dRobotStruct()) );
}

bool move3d_robot_is_in_collision( Robot* R )
{
    bool ncol = false;

    /* collision checking */
    if( g3d_get_KCD_CHOICE_IS_ACTIVE() )
    {
        //		if(G3D_ACTIVE_CC)
        {
            ncol = p3d_col_test_choice();
        }
    }
    else
    {
        //		if(G3D_ACTIVE_CC)
        {
            //cout << "p3d_col_test_all()" << endl;
            ncol = p3d_col_test_all();
        }
    }
    return ncol;
}

bool move3d_robot_is_in_collision_with_env( Robot* R )
{
    return pqp_robot_all_no_self_collision_test(static_cast<p3d_rob*>(R->getP3dRobotStruct()));
}

double move3d_robot_distance_to_env( Robot* R )
{
    p3d_vector3 closest_point_rob, closest_point_obst;
    return pqp_robot_environment_distance( static_cast<p3d_rob*>(R->getP3dRobotStruct()), closest_point_rob, closest_point_obst);
}

double move3d_robot_distance_to_robot( Robot* R1 , Robot* R2 )
{
    p3d_vector3 closest_point_rob1, closest_point_rob2;
    return pqp_robot_robot_distance( static_cast<p3d_rob*>(R1->getP3dRobotStruct()), static_cast<p3d_rob*>(R2->getP3dRobotStruct()), closest_point_rob1, closest_point_rob2 );
}

confPtr_t move3d_robot_get_init_pos( Robot* R )
{
    return (confPtr_t (new Configuration(R, static_cast<p3d_rob*>(R->getP3dRobotStruct())->ROBOT_POS)));
}

void move3d_robot_set_init_pos( Robot* R, const Configuration& q )
{
    configPt q_tmp = static_cast<p3d_rob*>(R->getP3dRobotStruct())->ROBOT_POS;
    p3d_copy_config_into(static_cast<p3d_rob*>(R->getP3dRobotStruct()), q.getConfigStructConst(), &q_tmp);
}

confPtr_t move3d_robot_get_goal_pos( Robot* R )
{
    return (confPtr_t (new Configuration(R,static_cast<p3d_rob*>(R->getP3dRobotStruct())->ROBOT_GOTO)));
}

void move3d_robot_set_goal_pos( Robot* R, const Configuration& q )
{
    configPt q_tmp = static_cast<p3d_rob*>(R->getP3dRobotStruct())->ROBOT_GOTO;
    p3d_copy_config_into(static_cast<p3d_rob*>(R->getP3dRobotStruct()), q.getConfigStructConst(), &q_tmp );
}

confPtr_t move3d_robot_get_current_pos( Robot* R )
{
    return (confPtr_t (new Configuration( R, p3d_get_robot_config(static_cast<p3d_rob*>(R->getP3dRobotStruct())),true)));
}

confPtr_t move3d_robot_get_new_pos( Robot* R )
{
    return (confPtr_t (new Configuration( R, p3d_alloc_config(static_cast<p3d_rob*>(R->getP3dRobotStruct())),true)));
}

unsigned int move3d_robot_get_nb_active_joints( Robot* R )
{
    unsigned int nbDoF(0);

    const std::vector<Joint*> joints = R->getJoints();

    for(unsigned int i=0;i<joints.size();i++)
    {
        p3d_jnt* jntPt = static_cast<p3d_jnt*>(joints[i]->getP3dJointStruct());

        for(int j=0; j<jntPt->dof_equiv_nbr; j++)
        {
            int k = jntPt->index_dof + j;

            if (
                    (p3d_jnt_get_dof_is_user(jntPt, j) && p3d_jnt_get_dof_is_active_for_planner(jntPt,j)) &&
                    (static_cast<p3d_rob*>(R->getP3dRobotStruct())->cntrt_manager->in_cntrt[k] != 2) )
            {
                nbDoF++;
            }
        }
    }

    return nbDoF;
}

Joint* move3d_robot_get_ith_active_joint( Robot* R,  unsigned int ithActiveDoF, unsigned int& ithDofOnJoint )
{
    unsigned int activeDof = 0;

    const std::vector<Joint*> joints = R->getJoints();

    for(unsigned int i=0;i<joints.size();i++)
    {
        p3d_jnt* jntPt = static_cast<p3d_jnt*>(joints[i]->getP3dJointStruct());

        for(int j=0; j<jntPt->dof_equiv_nbr; j++)
        {
            int k = jntPt->index_dof + j;

            if (
                    (p3d_jnt_get_dof_is_user(jntPt, j) && p3d_jnt_get_dof_is_active_for_planner(jntPt,j)) &&
                    (static_cast<p3d_rob*>(R->getP3dRobotStruct())->cntrt_manager->in_cntrt[k] != 2) )
            {
                if( activeDof == ithActiveDoF )
                {
                    ithDofOnJoint = j;
                    return joints[i];
                }

                activeDof++;
            }
        }
    }

    return NULL;
}

// ****************************************************************************************************
// ****************************************************************************************************
//
// JOINT
//
// ****************************************************************************************************
// ****************************************************************************************************

void* move3d_joint_constructor( Joint* J, void* jntPt, std::string& name )
{
    p3d_jnt* jntPt_2 = (p3d_jnt*)(jntPt);
    name = jntPt_2->name;
    return (p3d_jnt*)(jntPt_2);
}

Vector3d move3d_joint_get_vector_pos( const Joint* J )
{
    pp3d_matrix4 mat = &(J->getP3dJointStruct()->abs_pos);

    Vector3d v;

    v(0) = (*mat)[0][3];
    v(1) = (*mat)[1][3];
    v(2) = (*mat)[2][3];

    return v;
}

Transform3d move3d_joint_get_matrix_pos( const Joint* J )
{
    pp3d_matrix4 mat = &(J->getP3dJointStruct()->abs_pos);

    Transform3d t;

    t(0,0) = (*mat)[0][0];
    t(1,0) = (*mat)[1][0];
    t(2,0) = (*mat)[2][0];
    t(3,0) = 0.0;

    t(0,1) = (*mat)[0][1];
    t(1,1) = (*mat)[1][1];
    t(2,1) = (*mat)[2][1];
    t(3,1) = 0.0;

    t(0,2) = (*mat)[0][2];
    t(1,2) = (*mat)[1][2];
    t(2,2) = (*mat)[2][2];
    t(3,2) = 0.0;

    t(0,3) = (*mat)[0][3];
    t(1,3) = (*mat)[1][3];
    t(2,3) = (*mat)[2][3];
    t(3,3) = 1.0;

    return t;
}

void move3d_joint_shoot( Joint* J, Configuration& q, bool sample_passive )
{
    p3d_jnt* jntPt = (p3d_jnt*)( J->getP3dJointStruct() );
    p3d_rob* robotPt = (p3d_rob*)(q.getRobot()->getP3dRobotStruct());

    for(int j=0; j<jntPt->dof_equiv_nbr; j++)
    {
        int k = jntPt->index_dof + j;

        if ( sample_passive || ((p3d_jnt_get_dof_is_user(jntPt, j) && p3d_jnt_get_dof_is_active_for_planner(jntPt,j)) && (robotPt->cntrt_manager->in_cntrt[k] != 2) ) )
        {
            double vmin,vmax;
            p3d_jnt_get_dof_rand_bounds(jntPt, j, &vmin, &vmax);
            q[k] = p3d_random(vmin, vmax);
            //std::cout << "Sample Passive = "<<sample_passive<<" , Sampling q["<<k<<"] = "<<q[k]<< std::endl;
        }
        else
        {
            q[k] = p3d_jnt_get_dof(jntPt, j);
        }
    }
}

double move3d_joint_get_joint_dof( const Joint* J, int ithDoF )
{
    return p3d_jnt_get_dof( J->getP3dJointStruct(), ithDoF );
}

double move3d_joint_is_joint_dof_angular( const Joint* J, int ithDoF )
{
    return p3d_jnt_is_dof_angular( J->getP3dJointStruct(), ithDoF );
}

double move3d_joint_is_joint_dof_circular( const Joint* J, int ithDoF )
{
    return p3d_jnt_is_dof_circular( J->getP3dJointStruct(), ithDoF );
}

void move3d_joint_set_joint_dof( const Joint* J, int ithDoF, double value )
{
    p3d_jnt_set_dof( J->getP3dJointStruct(), ithDoF, value );
}

bool move3d_joint_is_joint_user( const Joint* J, int ithDoF )
{
    return p3d_jnt_get_dof_is_user( J->getP3dJointStruct(), ithDoF );
}

void move3d_joint_get_joint_bounds( const Joint* J, int ithDoF, double& vmin, double& vmax )
{
    p3d_jnt* jntPt = (p3d_jnt*)(J->getP3dJointStruct());
    vmin = jntPt->dof_data[ithDoF].vmin;
    vmax = jntPt->dof_data[ithDoF].vmax;
}

void move3d_joint_get_joint_rand_bounds( const Joint* J, int ithDoF, double& vmin, double& vmax )
{
    p3d_jnt* jntPt = (p3d_jnt*)(J->getP3dJointStruct());
    vmin = jntPt->dof_data[ithDoF].vmin_r;
    vmax = jntPt->dof_data[ithDoF].vmax_r;
}

int move3d_joint_get_nb_of_dofs( const Joint* J )
{
    p3d_jnt* jntPt = (p3d_jnt*)(J->getP3dJointStruct());
    return jntPt->dof_equiv_nbr;
}

int move3d_joint_get_index_of_first_joint( const Joint* J )
{
    p3d_jnt* jntPt = (p3d_jnt*)(J->getP3dJointStruct());
    return jntPt->index_dof;
}

Joint* move3d_joint_get_previous_joint( const Joint* J, Robot* R )
{
    p3d_jnt* jntPt = (p3d_jnt*)(J->getP3dJointStruct());
    Joint* prevJnt=NULL; int found=0;

    for (unsigned int i=0; i<R->getNumberOfJoints(); i++ )
    {
        Joint* jnt = R->getJoint(i);

        if ( jntPt->prev_jnt == jnt->getP3dJointStruct() )
        {
            found++;
            prevJnt = jnt;
        }
    }

    if (found == 1) {
        return prevJnt;
    }
    else if (found > 1 ) {
        cout << "Found : " << found << " prev. joints!!!" << endl;
    }

    return NULL;
}

double move3d_joint_get_dist( const Joint* J )
{
    return J->getP3dJointStruct()->dist;
}

// ****************************************************************************************************
// ****************************************************************************************************
//
// DAW
//
// ****************************************************************************************************
// ****************************************************************************************************

void move3d_draw_sphere_fct( double x, double y, double z, double radius )
{
    g3d_draw_solid_sphere( x, y, z, radius, 10 );
}

void move3d_draw_one_line_fct( double x1, double y1, double z1, double x2, double y2, double z2, int color, double *color_vect )
{
    g3d_drawOneLine( x1, y1, z1, x2, y2, z2, color, color_vect );
}

void move3d_draw_clear_handles_fct()
{

}

// ****************************************************************************************************
// ****************************************************************************************************
//
// SCENE
//
// ****************************************************************************************************
// ****************************************************************************************************

void* move3d_scene_constructor_fct( void* penv, std::string& name, std::vector<Robot*>& robots, std::string& active_robot )
{
    p3d_env* scene = static_cast<p3d_env*>( penv );

    name = scene->name;

    robots.clear();

    for (int i=0; i<scene->nr; i++)
    {
        cout << "Add new Robot to Scene : " << scene->robot[i]->name << endl;
        robots.push_back( new Robot( scene->robot[i] ) );
    }

    // For all HRI planner the robot
    // Will be set here
    if ( scene->active_robot )
    {
        active_robot = scene->active_robot->name;
        cout << "The Scene global_ActiveRobotName is : " << active_robot << endl;
    }
    else
    {
        active_robot = "";
        cout << "WARNING : the Scene global_ActiveRobotName has not been set!!!!" << endl;
    }

    return scene;
}

void move3d_scene_set_active_robot( Scene* sce, void* penv, const std::string& name )
{
    p3d_env* scene = static_cast<p3d_env*>( penv );
    unsigned int id = sce->getRobotId( name );
    p3d_sel_desc_id( P3D_ROBOT, scene->robot[id] );
}

Robot* move3d_scene_get_active_robot( Scene* sce, void* penv )
{
    p3d_env* scene = static_cast<p3d_env*>( penv );
    cout << "scene->cur_robot->name : " << scene->cur_robot->name << endl;
    return sce->getRobotByName( scene->cur_robot->name );
}

double move3d_scene_get_dmax( void* penv )
{
    p3d_env* scene = static_cast<p3d_env*>( penv );
    return scene->dmax;
}

std::vector<double> move3d_scene_get_bounds( void* penv )
{
    p3d_env* scene = static_cast<p3d_env*>( penv );

    vector<double> bounds(6);
    bounds[0] = scene->box.x1;
    bounds[1] = scene->box.x2;
    bounds[2] = scene->box.y1;
    bounds[3] = scene->box.y2;
    bounds[4] = scene->box.z1;
    bounds[5] = scene->box.z2;

    return bounds;
}

// ****************************************************************************************************
// ****************************************************************************************************
//
// SETTER
//
// ****************************************************************************************************
// ****************************************************************************************************

void move3d_set_api_scene()
{
    move3d_set_fct_scene_constructor( boost::bind( move3d_scene_constructor_fct, _1, _2, _3, _4 ) );
    move3d_set_fct_set_active_robot( boost::bind( move3d_scene_set_active_robot, _1, _2, _3 ) );
    move3d_set_fct_get_active_robot( boost::bind( move3d_scene_get_active_robot, _1, _2 ) );
    move3d_set_fct_get_dmax( boost::bind( move3d_scene_get_dmax, _1 ) );
    move3d_set_fct_get_bounds( boost::bind( move3d_scene_get_bounds, _1 ) );
}

void move3d_set_api_functions_configuration()
{
    move3d_set_fct_configuration_constructor_robot( boost::bind( move3d_configuration_constructor, _1 ));
    move3d_set_fct_configuration_constructor_config_struct( boost::bind( move3d_configuration_constructor_configstruct, _1, _2, _3 ));
    move3d_set_fct_configuration_assignment( boost::bind( move3d_configuration_assignment, _1, _2 ) );
    move3d_set_fct_configuration_clear( boost::bind( move3d_configuration_clear, _1, _2 ) );
    move3d_set_fct_configuration_convert_to_radians( boost::bind( move3d_configuration_convert_to_radian, _1, _2 ) );
    move3d_set_fct_configuration_convert_to_degrees( boost::bind( move3d_configuration_convert_to_degrees, _1, _2 ) );
    move3d_set_fct_configuration_get_struct_copy( boost::bind( move3d_configuration_copy_struct, _1, _2 ));
    move3d_set_fct_configuration_dist( boost::bind( move3d_configuration_dist, _1, _2, _3, _4 ) );
    move3d_set_fct_configuration_dist_choice( boost::bind( move3d_configuration_dist_choice, _1, _2, _3, _4 ) );
    move3d_set_fct_configuration_in_collision( boost::bind( move3d_configuration_in_collision, _1 ) );
    move3d_set_fct_configuration_is_out_of_bounds( boost::bind( move3d_configuration_is_out_of_bounds, _1, _2, _3 ) );
    move3d_set_fct_configuration_adapt_circular_joint_limits( boost::bind( move3d_configuration_adapt_to_circular_joints, _1, _2 ) );
    move3d_set_fct_configuration_dist_env( boost::bind( move3d_configuration_dist_env, _1 ) );
    move3d_set_fct_configuration_equal( boost::bind( move3d_configuration_equal, _1, _2, _3, _4 ) );
//    move3d_set_fct_configuration_copy( boost::bind( move3d_configuration_copy, _1, _2, _3 ) );
    move3d_set_fct_configuration_copy_passive( boost::bind( move3d_configuration_copy_passive, _1, _2, _3 ) );
    move3d_set_fct_configuration_add( boost::bind( move3d_configuration_add, _1, _2, _3, _4 ) );
    move3d_set_fct_configuration_sub( boost::bind( move3d_configuration_sub, _1, _2, _3, _4 ) );
    move3d_set_fct_configuration_mult( boost::bind( move3d_configuration_mult, _1, _2, _3, _4 ) );
    move3d_set_fct_configuration_print( boost::bind( move3d_configuration_print, _1, _2, _3 ) );
}

void move3d_set_api_functions_localpath()
{
    move3d_set_fct_localpath_copy( boost::bind( move3d_localpath_copy, _1, _2 ));
    // move3d_set_fct_localpath_copy_p3d( boost::bind( move3d_localpath_destructor, _1 ) );
    move3d_set_fct_localpath_path_destructor( boost::bind( move3d_localpath_destructor, _1 ) );
    move3d_set_fct_localpath_copy_from_struct( boost::bind( move3d_localpath_init_from_struct, _1, _2, _3, _4 ) );
    move3d_set_fct_localpath_get_struct( boost::bind( move3d_localpath_get_localpath_struct, _1, _2, _3 ) );
    move3d_set_fct_localpath_classic_test( boost::bind( move3d_localpath_classic_test, _1, _2, _3, _4 ) );
    move3d_set_fct_localpath_is_valid( boost::bind( move3d_localpath_is_valid_test, _1, _2 ) );
    move3d_set_fct_localpath_get_length( boost::bind( move3d_localpath_lenth, _1 ) );
    move3d_set_fct_localpath_get_param_max( boost::bind( move3d_localpath_max_param, _1 ) );
    move3d_set_fct_localpath_config_at_dist( boost::bind( move3d_localpath_config_at_dist, _1, _2 ) );
    move3d_set_fct_localpath_config_at_param( boost::bind( move3d_localpath_config_at_param, _1, _2 ) );
    move3d_set_fct_localpath_stay_within_dist( boost::bind( move3d_stay_within_dist, _1, _2, _3, _4 ) );
}

void move3d_set_api_functions_robot()
{
    move3d_set_fct_robot_constructor( boost::bind( move3d_robot_constructor, _1, _2, _3, _4, _5, _6 ) );
    move3d_set_fct_robot_get_current_trajectory( boost::bind( move3d_robot_get_current_trajectory, _1 ) );
    move3d_set_fct_robot_shoot( boost::bind( move3d_robot_shoot, _1, _2 ) );
    // move3d_set_fct_robot_shoot_dir( boost::bind( move3d_robot_set_and_update, _1, _2, _3 ));
    move3d_set_fct_robot_set_and_update( boost::bind( move3d_robot_set_and_update, _1, _2, _3 ));
    move3d_set_fct_robot_set_and_update_multi_sol( boost::bind( move3d_robot_set_and_update_multisol, _1 , _2 ) );
    move3d_set_fct_robot_without_constraints( boost::bind( move3d_robot_set_and_update_with_constraints, _1, _2 ) );
    move3d_set_fct_robot_is_in_collision( boost::bind( move3d_robot_is_in_collision, _1 ));
    move3d_set_fct_robot_is_in_collision_with_others_and_env( boost::bind( move3d_robot_is_in_collision_with_env, _1 ) );
    move3d_set_fct_robot_distance_to_env( boost::bind( move3d_robot_distance_to_env, _1 ) ) ;
    move3d_set_fct_robot_distance_to_robot( boost::bind( move3d_robot_distance_to_robot, _1, _2 ) );
    move3d_set_fct_robot_get_init_pos( boost::bind( move3d_robot_get_init_pos, _1 ) );
    move3d_set_fct_robot_set_init_pos( boost::bind( move3d_robot_set_init_pos, _1, _2 ) );
    move3d_set_fct_robot_get_goal_pos( boost::bind( move3d_robot_get_goal_pos, _1 ) );
    move3d_set_fct_robot_set_goal_pos( boost::bind( move3d_robot_set_goal_pos, _1, _2 ) );
    move3d_set_fct_robot_get_current_pos( boost::bind( move3d_robot_get_current_pos, _1 ) );
    move3d_set_fct_robot_get_new_pos( boost::bind( move3d_robot_get_new_pos, _1 ) );
    move3d_set_fct_robot_get_number_of_active_dofs( boost::bind( move3d_robot_get_nb_active_joints, _1 ) );
    move3d_set_fct_robot_get_ith_active_dof_joint( boost::bind( move3d_robot_get_ith_active_joint, _1, _2 , _3 ) );
}

void move3d_set_api_functions_joint()
{
    move3d_set_fct_joint_constructor( boost::bind( move3d_joint_constructor, _1, _2, _3 ) );
    move3d_set_fct_joint_get_vector_pos( boost::bind( move3d_joint_get_vector_pos, _1 ) );
    move3d_set_fct_joint_get_matrix_pos( boost::bind( move3d_joint_get_matrix_pos, _1 ) );
    move3d_set_fct_joint_joint_shoot( boost::bind( move3d_joint_shoot, _1, _2, _3 ) );
    move3d_set_fct_joint_get_joint_dof( boost::bind( move3d_joint_get_joint_dof, _1, _2 ) );
    move3d_set_fct_joint_is_joint_dof_angular( boost::bind( move3d_joint_is_joint_dof_angular, _1, _2 ) );
    move3d_set_fct_joint_is_joint_dof_circular( boost::bind( move3d_joint_is_joint_dof_circular, _1, _2 ) );
    move3d_set_fct_joint_set_joint_dof( boost::bind( move3d_joint_set_joint_dof, _1, _2, _3 ) );
    move3d_set_fct_joint_is_joint_user( boost::bind( move3d_joint_is_joint_user, _1, _2 ) );
    move3d_set_fct_joint_get_bound( boost::bind( move3d_joint_get_joint_bounds, _1, _2, _3, _4 ) );
    move3d_set_fct_joint_get_bound_rand( boost::bind( move3d_joint_get_joint_rand_bounds, _1, _2, _3, _4 ) );
    move3d_set_fct_joint_get_nb_of_dofs( boost::bind( move3d_joint_get_nb_of_dofs, _1 ) );
    move3d_set_fct_joint_get_index_of_first_dof( boost::bind( move3d_joint_get_index_of_first_joint, _1 ) );
    move3d_set_fct_joint_get_previous_joint( boost::bind( move3d_joint_get_previous_joint, _1, _2 ) );
    move3d_set_fct_joint_joint_dist( boost::bind( move3d_joint_get_dist, _1 ) );
}

void move3d_set_api_functions_draw()
{
    move3d_set_fct_draw_sphere( boost::bind( move3d_draw_sphere_fct, _1, _2, _3, _4 ) );
    move3d_set_fct_draw_one_line( boost::bind( move3d_draw_one_line_fct, _1, _2, _3, _4, _5, _6, _7, _8 ) );
    move3d_set_fct_draw_clear_handles( boost::bind( move3d_draw_clear_handles_fct ) );
}
