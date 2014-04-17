#include "libmove3d_simple_api.hpp"

#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/localpath.hpp"
#include "API/Device/robot.hpp"
#include "API/Device/joint.hpp"

#include <boost/bind.hpp>
#include <libmove3d/p3d/env.hpp>
#include <stdio.h>

using namespace Move3D;
using std::cout;
using std::endl;

#define EPS6 0.000001
#define RTOD(r)      ((r)*180.0/M_PI)
#define DTOR(d)      ((d)*M_PI/180.0)

double angle_limit_180(double angle){

    while (angle < -180){
        angle += 360;
    }
    while (angle > 180){
        angle -= 360;
    }
    return angle;
}

double angle_limit_360(double angle){

    while (angle < 0){
        angle += 360;
    }
    while (angle > 360){
        angle -= 360;
    }
    return angle;
}

double angle_limit_PI(double angle){

    while (angle < -M_PI){
        angle += 2*M_PI;
    }
    while (angle > M_PI){
        angle -= 2*M_PI;
    }
    return angle;
}

double angle_limit_2PI(double angle){

    while (angle < 0){
        angle += 2*M_PI;
    }
    while (angle > 2*M_PI){
        angle -= 2*M_PI;
    }
    return angle;
}

double dist_circle(double theta1, double theta2)
{
    double dtheta = fabs(angle_limit_PI(theta2 - theta1));
    return dtheta;
}

double diff_angle(double theta1, double theta2)
{
    double dtheta = angle_limit_PI(theta2 - theta1);
    return dtheta;
}

// ****************************************************************************************************
// ****************************************************************************************************
//
// CONFIGURATION SIMPLE
//
// ****************************************************************************************************
// ****************************************************************************************************

double* move3d_configuration_simple_constructor(Robot* robot)
{
    return new double[robot->getNumberOfDofs()];
}

double* move3d_configuration_simple_constructor_configstruct( Robot* R, double* C, bool noCopy )
{
    double* q;

    if(C==NULL)
    {
        q = C;
    }
    else
    {
        if( noCopy) {
            q = C;
        }
        else {
            int nb_dof = R->getNumberOfDofs();
            q = new double[nb_dof];
            memcpy((void*)q, (void*)C, (size_t)nb_dof*sizeof(double));
        }
    }
    return q;
}

void move3d_configuration_simple_assignment( const Configuration& q_source, Configuration& q_target )
{
    int nb_dof = q_target.getRobot()->getNumberOfDofs();
    memcpy((void*)q_target.getConfigStruct(), (void*)q_source.getConfigStructConst(), (size_t)(nb_dof*sizeof(double)));
}

void move3d_configuration_simple_clear( Robot* R, double* C )
{
    delete[] C;
}

void move3d_configuration_simple_convert_to_radian( Robot* R, double* C )
{
    double* q_rad = new double[R->getNumberOfDofs()];
    // p3d_convert_config_deg_to_rad( (p3d_rob*)R->getP3dRobotStruct(),C,&q);
    int i, j;
    int k;
    Joint* jntPt;

    k = 0;
    for (i = 0; i<int(R->getNumberOfJoints()); i++) {

        jntPt = R->getJoint( i );

        for (j = 0; j < int(jntPt->getNumberOfDof()); j++) {

            // TODO
            if( jntPt->isJointDofAngular(j) ) {
                q_rad[k] = DTOR(C[k]);
            } else {
                q_rad[k] = C[k];
            }
            k ++;
        }
    }

    delete[] C;
    C = q_rad;
}

confPtr_t move3d_configuration_simple_convert_to_degrees( Robot* R, double* C )
{
    double* q_deg = new double[R->getNumberOfDofs()];

    int i, j, k;
    Joint* jntPt;

    k = 0;
    for (i = 0; i<int(R->getNumberOfJoints()); i++) {

        jntPt = R->getJoint( i );

        for (j = 0; j < int(jntPt->getNumberOfDof()); j++) {

            // TODO
            if( jntPt->isJointDofAngular(j) ) {
                q_deg[k] = RTOD(C[k]);
            } else {
                q_deg[k] = C[k];
            }

            k ++;
        }
    }

    return (confPtr_t (new Configuration(R,q_deg,true)));
}

double* move3d_configuration_simple_copy_struct( Robot* R, double* C )
{
    int nb_dof = R->getNumberOfDofs();
    double* copy_q = new double[nb_dof];
    memcpy((void*)copy_q, (void*)C, (size_t)nb_dof*sizeof(double));
    return copy_q;
}

double move3d_jnt_calc_dof_dist( Joint* jntPt, int i_dof, double* q_init, double* q_end)
{
    int k = jntPt->getIndexOfFirstDof() + i_dof;
    double WeightRota = 1.;

    if ( jntPt->isJointDofAngular(i_dof) ) {
        /* multiply by jntPt->dist to be able to compare angle with length */
        if ( jntPt->isJointDofCircular(i_dof) )  {
            return std::fabs( WeightRota * jntPt->getDist() * dist_circle(q_init[k], q_end[k]) );
        }
        else {
            return std::fabs( WeightRota * jntPt->getDist() * (q_end[k] - q_init[k]) );
        }
    }
    return std::fabs( q_end[k] - q_init[k] );
}

double move3d_configuration_simple_dist( Robot* R, double* qi, double* qf, bool print )
{
    double ljnt = 0.;

    for (int i=0; i<int(R->getNumberOfJoints()); i++)
    {
        Joint* jntPt = R->getJoint(i);

        for (int j=0; j<int(jntPt->getNumberOfDof()); j++)
        {
            // if (robotPt->cntrt_manager->in_cntrt[jntPt->getIndexOfFirstDof() + j] != DOF_PASSIF)
            // {

            double dist = move3d_jnt_calc_dof_dist( jntPt, j, qi, qf );
            dist = dist * dist;

            if( std::isnan(dist) ) {
                printf("Distance computation error !!!\n");
                return std::numeric_limits<double>::max();
            }
            if(print)
                printf(" dist[%d] = %f\n",jntPt->getIndexOfFirstDof() + j, dist );
            ljnt += ( dist );
            // }
        }
    }

    return std::sqrt(ljnt);
    // return p3d_dist_config( (p3d_rob*)R->getP3dRobotStruct(), q1, q2, print );
}

double move3d_configuration_simple_dist_choice( Robot* R, const Configuration& q1, const Configuration& q2, int dist_choice )
{
    // NO other choice here!!!
    return (q1.dist(q2));
}

bool move3d_configuration_simple_in_collision(Robot* R)
{
//    return p3d_col_test_robot( (p3d_rob*)R->getP3dRobotStruct(), JUST_BOOL );
    return R->isInCollision();
}

bool move3d_configuration_simple_is_out_of_bounds( Robot* R, double* C, bool print )
{
    // return p3d_isOutOfBounds( (p3d_rob*)R->getP3dRobotStruct(), C, print);
    int is_out_of_bounds = false;

    // double EPS6 = 1e-6;

    for(int i = 0; i < int(R->getNumberOfJoints()); i++)
    {
        Joint* jntPt = R->getJoint( i );

        for(int j = 0; j < int(jntPt->getNumberOfDof()); j++)
        {
            double vmin = std::numeric_limits<double>::min(), vmax = std::numeric_limits<double>::max();

            jntPt->getDofBounds( j, vmin, vmax );

            if ( ( C[jntPt->getIndexOfFirstDof() + j] < (vmin - EPS6) ) || (C[jntPt->getIndexOfFirstDof() + j] > (vmax + EPS6) ))
            {
                if( print )
                {
                    printf("The joint %s is outside the joint bounds. Vmin : %f, Value : %f, Vmax = %f\n", jntPt->getName().c_str(), vmin, C[jntPt->getIndexOfFirstDof() + j], vmax);
                }
                is_out_of_bounds = true;
            }
        }
    }

    return is_out_of_bounds;
}

void move3d_configuration_simple_adapt_to_circular_joints( Robot* R, double* C )
{
//    configPt q = p3d_alloc_config( (p3d_rob*)R->getP3dRobotStruct() );
//    p3d_adaptConfigsForCircularDofs( (p3d_rob*)R->getP3dRobotStruct(), &C, &q );
//    p3d_destroy_config( (p3d_rob*)R->getP3dRobotStruct(), q );
    // TODO
    return;
}

double move3d_configuration_simple_dist_env(Robot* R)
{
    /*
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
    */
    double min_dist = 0.0;
    return min_dist;
}

bool move3d_configuration_simple_equal( Robot* R, double* q_i, double* q_f, bool print )
{
    int equal = true;
    // double EPS6 = 1e-6;

    int k = 0;
    for (int i = 0; i<int(R->getNumberOfJoints()); i++) {
        Joint* jntPt = R->getJoint( i );
        for (int j = 0; j < int(jntPt->getNumberOfDof()); j++) {
            if (/*p3d_jnt_get_dof_is_user(jntPt, j)*/ true) {
                if ( jntPt->isJointDofCircular(j) ) {
                    if (dist_circle(q_i[k], q_f[k]) > EPS6) {
                        if(print) {
                            printf("diff[%d] : %f\n",k,dist_circle(q_i[k], q_f[k]));
                            equal = false;
                        }
                        else {
                            return false;
                        }
                    }
                } else {
                    if (fabs(q_i[k] - q_f[k]) > EPS6) {
                        if(print) {
                            printf("diff[%d] : %f\n",k,fabs(q_i[k] - q_f[k]));
                            equal = false;
                        }
                        else {
                            return false;
                        }
                    }
                }
            }
            k ++;
        }
    }
    return equal;

    // return p3d_equal_config( (p3d_rob*)R->getP3dRobotStruct(), C, q, print );
}

void move3d_configuration_simple_copy_passive( Robot* R, double* C, double* q )
{
    for( int i = 0; i<int(R->getNumberOfJoints()); i++ )
    {
        Joint* joint = R->getJoint( i );

        for (int j  =0; j<int(joint->getNumberOfDof()); j++)
        {
            int k = joint->getIndexOfFirstDof() + j;
            // if ( (!p3d_jnt_get_dof_is_user(joint, j)) || (!p3d_jnt_get_dof_is_active_for_planner(joint, j) ))
            q[k] = C[k];
        }
    }
}

void move3d_configuration_simple_add( Robot* R, double* C, double* q1, double* q2 )
{
    // p3d_addConfig( R->getP3dRobotStruct(), C, q1, q2 );
}

void move3d_configuration_simple_sub( Robot* R, double* C, double* q1, double* q2 )
{
    // p3d_subConfig( R->getP3dRobotStruct(), C, q1, q2 );
}

void move3d_configuration_simple_mult( Robot* R, double* C, double* q2, double coeff )
{
    int k = 0;
    for (int i = 0; i<int(R->getNumberOfJoints()); i++)
    {
        Joint *jntPt = R->getJoint( i );

        for (int j=0; j < int(jntPt->getNumberOfDof()); j++)
        {
            q2[k] *= coeff;
            k ++;
        }
    }
}

void move3d_configuration_simple_print( Robot* R, double* C, bool withPassive )
{
    cout << "Print Configuration; Robot: " << R->getP3dRobotStruct() << endl;

    int k = 0;
    for(int i=0; i<int(R->getNumberOfJoints()); i++)
    {
        Joint* jntPt = R->getJoint( i );

        for(int j=0; j< int(jntPt->getNumberOfDof()); j++)
        {
            k = jntPt->getIndexOfFirstDof() + j;

            if( true )
            // if ( withPassive || ( p3d_jnt_get_dof_is_user(jntPt, j) && (robotPt->cntrt_manager->in_cntrt[k] != DOF_PASSIF )))
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
// LOCAL PATHS SIMPLE
//
// ****************************************************************************************************
// ****************************************************************************************************

void* move3d_localpath_simple_copy( const LocalPath& path, Robot* R )
{
    return NULL;
    // return path.getP3dLocalpathStructConst() ? path.getP3dLocalpathStructConst()->copy( (p3d_rob*) R->getP3dRobotStruct(), path.getP3dLocalpathStructConst()) : NULL;
}

void move3d_localpath_simple_destructor( LocalPath& path )
{
//    if ( path.getP3dLocalpathStructConst() )
//    {
//        path.getP3dLocalpathStructConst()->destroy( path.getRobot()->getP3dRobotStruct(), path.getP3dLocalpathStructConst());
//    }
}

void* move3d_localpath_simple_get_localpath_struct( LocalPath& path, bool multi_sol, int& type )
{
//    p3d_localpath* path_struct = NULL;

//    if ( !path.getP3dLocalpathStructConst() )
//    {
//        if( !multi_sol )
//        {
//            path_struct = p3d_local_planner( (p3d_rob*) path.getRobot()->getP3dRobotStruct(), path.getBegin()->getConfigStruct(), path.getEnd()->getConfigStruct() );
//        }
//        else
//        {
//            path_struct = p3d_local_planner_multisol( (p3d_rob*) path.getRobot()->getP3dRobotStruct(), path.getBegin()->getConfigStruct(), path.getEnd()->getConfigStruct(), path.getIkSol() );
//        }

//        if ( path_struct )
//        {
//            type = path_struct->type_lp;
//        }
//    }
//    else {
//        path_struct = path.getP3dLocalpathStructConst();
//    }

    return NULL;
}

void* move3d_localpath_simple_init_from_struct( LocalPath& path, void* lpPtr, confPtr_t& begin, confPtr_t& end )
{
//    p3d_localpath* path_struct = NULL;

    // TODO : check which copy are/are not necessary.
//    if (lpPtr)
//    {
//        path_struct = static_cast<p3d_localpath*>(lpPtr)->copy( static_cast<p3d_rob*>( path.getRobot()->getP3dRobotStruct()), static_cast<p3d_localpath*>(lpPtr));

//        begin = confPtr_t (
//                    new Configuration( path.getRobot(), path_struct->config_at_param(
//                                           static_cast<p3d_rob*>(path.getRobot()->getP3dRobotStruct()),
//                                           path_struct, 0),true));

//        begin->setConstraints();

//        end = confPtr_t (
//                    new Configuration( path.getRobot(), path_struct->config_at_param(
//                                           static_cast<p3d_rob*>(path.getRobot()->getP3dRobotStruct()),
//                                           path_struct,
//                                           path_struct->range_param),true));

//        end->setConstraints();

//        path.setType( static_cast<p3d_localpath*>(lpPtr)->type_lp );
//    }
//    else
//    {
//        cout << "Warning : creating Localpath from uninitialized p3d_localpath"
//             << endl;
//    }

    return NULL;
}

//! returns true if valid
bool move3d_localpath_simple_classic_test( LocalPath& path, confPtr_t q_last_valid, int& nb_col_check, double& last_valid_param )
{
    // double EPS6 = 1e-6;
    Robot* robot = path.getRobot();
    nb_col_check = 0;

    double u = 0., du, umax; /* parameters along the local path */
    //	configPt qp;
    // int njnt = robot->getNumberOfJoints();
    // double* distances;
    // double tolerance, newtol, dist0;
    double dmax;
    int end_localpath = 0;

    umax = path.getParamMax();

    // distances = new double[njnt+1];
    // tolerance = 0.0;

    dmax = ENV.getDouble(Env::dmax);

//    cout << "umax : " << umax << endl;
//    cout << "dmax : " << dmax << endl;

//    if (/*p3d_col_get_tolerance(&tolerance) &&*/ false )
//    {
//        // p3d_col_set_tolerance(tolerance + dmax);
//        newtol = tolerance + dmax;
//        dist0 = dmax;
//    }
//    else
//    {
//        newtol = 0;
//        dist0 = 2 * dmax;
//    }

    /* current position of robot is saved */
    confPtr_t qp = path.configAtParam(0.0);
    if ( !robot->setAndUpdate(*qp) )
    {
        return false;
    }

    /* We suppose that the fisrt and the last configurations are valid so
     we could test the configuration at dist0 from the bounds */
    /* Remove the end of interval */
    u = umax;

    confPtr_t q = path.configAtParam( u );
    if ( !robot->setAndUpdate(*q) )
    {
        return false;
    }

//    for (int j = 0; j <= njnt; j++)
//    {
//        distances[j] = dist0;
//    }

    if( umax < EPS6 )
    {
        /* The initial position of the robot is recovered */
        robot->setAndUpdateWithoutConstraints(*qp);
        return true;
    }

    /* Remove the beginning of interval */
    u = 0;

//    for (int j = 0; j<=njnt; j++)
//    {
//        distances[j] = dist0;
//    }

    // du = this->stayWithInDistance(u,forward_s,distances);
    du = dmax;
    u = du;

    if (u > umax - EPS6) {
        u = umax;
        end_localpath++;
    }

    // dist0 = 2 * dmax - newtol; /* Minimal distance we could cross at each step */

    confPtr_t q_last_valid_tmp = robot->getNewConfig();
    double u_last_valid_tmp = 0.0;

    while( end_localpath < 2 )
    {
        /* position of the robot corresponding to parameter u */
        qp = path.configAtParam( u );

        if ( !robot->setAndUpdate(*qp) )
        {
            /* The initial position of the robot is recovered */
            robot->setAndUpdateWithoutConstraints(*qp);
            return false;
        }

        // qp = robot->getCurrentPos();

        // TEMP MODIF : PROBLEM WITH SELF COLLISION : CONSTANT STEP
        // p3d_BB_dist_robot( robot->getP3dRobotStruct(), distances );

//        int test = true;
//        for (int j=0; j<=njnt; j++)
//        {
//            if (distances[j] < newtol + EPS6)
//            {
//                test = true;
//            }
//            distances[j] += dist0;
//            //cout << "distance["<<j<<"]=" << distances[j] << endl;
//        }

        if ( true )
        {
            /////////////////////////
            /* collision checking */
            //cout << "Test" << endl;
            nb_col_check++;
            if( robot->isInCollision() )
            {
                /* The initial position of the robot is recovered */
                // p3d_set_current_q_inv((p3d_rob*)_Robot->getP3dRobotStruct(), getP3dLocalpathStruct(), qp->getConfigStruct());
                robot->setAndUpdateWithoutConstraints(*qp);
                // p3d_col_set_tolerance(tolerance);
                // cout << "local path not valid" << endl;
                q_last_valid->Clear();
                q_last_valid->setConfigurationCopy( *q_last_valid_tmp );

                last_valid_param = u_last_valid_tmp;
                return false;
            }
        }

        // double Kpath = u / this->getParamMax();
        q_last_valid_tmp = qp;

        u_last_valid_tmp = u;
        // du = robot->stayWithInDistance(u,forward_s,distances);

        u += du;

        if ( u > umax - EPS6 )
        {
            u = umax;
            end_localpath++;
        }
    }

    q_last_valid->Clear();
    q_last_valid->setConfigurationCopy( *q_last_valid_tmp );

    last_valid_param = u_last_valid_tmp;

    // UNCOMMENT TO PRINT NB OF COLLISION CHECKS
    // cout << "nb_col_check : " << nb_col_check << endl;

    return true;
}

bool move3d_localpath_simple_is_valid_test( LocalPath& path, int& nb_col_check )
{
    double last_valid_param = 0.0;
    confPtr_t q_last_valid = path.getRobot()->getNewConfig();
    return move3d_localpath_simple_classic_test( path, q_last_valid, nb_col_check, last_valid_param );
}

double move3d_localpath_simple_length( LocalPath& path )
{
    return path.getBegin()->dist( *path.getEnd() );
}

double move3d_localpath_simple_max_param( LocalPath& path )
{
     return path.getBegin()->dist( *path.getEnd() );
}

double move3d_simple_jnt_calc_dof_value( Joint* jntPt, int j, double* q_init, double* q_end, double alpha )
{
    int k = jntPt->getIndexOfFirstDof() + j;
    double vmin, vmax;
    // double vmax ;

    alpha = std::max(0., std::min(1.,alpha));

    if ( jntPt->isJointDofCircular( j ) )
    {
        // cout << "Dof is circular" << endl;
        // TODO
        jntPt->getDofBounds( j, vmin, vmax );
        // p3d_jnt_get_dof_bounds_deg(jntPt, i_dof, &vmin, &vmax);
        if(vmin < 0.0) {
            return angle_limit_PI( q_init[k] + alpha * diff_angle(q_init[k], q_end[k]) );
        } else {
            return angle_limit_2PI( q_init[k] + alpha * diff_angle(q_init[k], q_end[k]) );
        }
    }
    return (q_init[k] + alpha * (q_end[k] - q_init[k]));
}


confPtr_t move3d_localpath_simple_config_at_dist( LocalPath& path, double dist )
{
    Robot* robot = path.getRobot();

    double length = path.length();

    if (dist < 0)
      dist = 0;
    if (dist > length )
      dist = length;

    confPtr_t q_init = path.getBegin();
    if(length == 0){
      return q_init;
    }

    confPtr_t q_end = path.getEnd();
    if (dist == length ) {
      return q_end;
    }

    double alpha = dist / length;

    confPtr_t q = robot->getNewConfig();

    for (int i=0; i<int(robot->getNumberOfJoints()); i++)
    {
        Joint* jntPt = robot->getJoint( i );

        for (int j=0; j< int(jntPt->getNumberOfDof()); j++)
        {
            int k = jntPt->getIndexOfFirstDof()+j;

            (*q)[k] = move3d_simple_jnt_calc_dof_value( jntPt, j, q_init->getConfigStruct(), q_end->getConfigStruct(), alpha );
        }
    }

    return q;
}

confPtr_t move3d_localpath_simple_config_at_param( LocalPath& path, double param )
{
    return move3d_localpath_simple_config_at_dist( path, param );
}

double move3d_localpath_simple_stay_within_dist( LocalPath& path, double u, bool goForward, double distance )
{
    double du = 0.01;
    return du;
}

// ****************************************************************************************************
// ****************************************************************************************************
//
// SETTER
//
// ****************************************************************************************************
// ****************************************************************************************************

void move3d_set_api_functions_configuration_simple()
{
    move3d_set_fct_configuration_constructor_robot( boost::bind( move3d_configuration_simple_constructor, _1 ));
    move3d_set_fct_configuration_constructor_config_struct( boost::bind( move3d_configuration_simple_constructor_configstruct, _1, _2, _3 ));
    move3d_set_fct_configuration_assignment( boost::bind( move3d_configuration_simple_assignment, _1, _2 ) );
    move3d_set_fct_configuration_clear( boost::bind( move3d_configuration_simple_clear, _1, _2 ) );
    move3d_set_fct_configuration_convert_to_radians( boost::bind( move3d_configuration_simple_convert_to_radian, _1, _2 ) );
    move3d_set_fct_configuration_convert_to_degrees( boost::bind( move3d_configuration_simple_convert_to_degrees, _1, _2 ) );
    move3d_set_fct_configuration_get_struct_copy( boost::bind( move3d_configuration_simple_copy_struct, _1, _2 ));
    move3d_set_fct_configuration_dist( boost::bind( move3d_configuration_simple_dist, _1, _2, _3, _4 ) );
    move3d_set_fct_configuration_dist_choice( boost::bind( move3d_configuration_simple_dist_choice, _1, _2, _3, _4 ) );
    move3d_set_fct_configuration_in_collision( boost::bind( move3d_configuration_simple_in_collision, _1 ) );
    move3d_set_fct_configuration_is_out_of_bounds( boost::bind( move3d_configuration_simple_is_out_of_bounds, _1, _2, _3 ) );
    move3d_set_fct_configuration_adapt_circular_joint_limits( boost::bind( move3d_configuration_simple_adapt_to_circular_joints, _1, _2 ) );
    move3d_set_fct_configuration_dist_env( boost::bind( move3d_configuration_simple_dist_env, _1 ) );
    move3d_set_fct_configuration_equal( boost::bind( move3d_configuration_simple_equal, _1, _2, _3, _4 ) );
//    move3d_set_fct_configuration_copy( boost::bind( move3d_configuration_copy, _1, _2, _3 ) );
    move3d_set_fct_configuration_copy_passive( boost::bind( move3d_configuration_simple_copy_passive, _1, _2, _3 ) );
    move3d_set_fct_configuration_add( boost::bind( move3d_configuration_simple_add, _1, _2, _3, _4 ) );
    move3d_set_fct_configuration_sub( boost::bind( move3d_configuration_simple_sub, _1, _2, _3, _4 ) );
    move3d_set_fct_configuration_mult( boost::bind( move3d_configuration_simple_mult, _1, _2, _3, _4 ) );
    move3d_set_fct_configuration_print( boost::bind( move3d_configuration_simple_print, _1, _2, _3 ) );
}

// Function that creates a move3d trajectory
extern void* move3d_localpath_get_localpath_struct( LocalPath& path, bool multi_sol, int& type );
extern void* move3d_localpath_init_from_struct( LocalPath& path, void* lpPtr, confPtr_t& begin, confPtr_t& end );
//extern confPtr_t move3d_localpath_config_at_param( LocalPath& path, double param );
//extern bool move3d_localpath_is_valid_test( LocalPath& path, int& nb_col_check );

void move3d_set_api_functions_localpath_simple()
{
    move3d_set_fct_localpath_copy( boost::bind( move3d_localpath_simple_copy, _1, _2 ));
    // move3d_set_fct_localpath_copy_p3d( boost::bind( move3d_localpath_destructor, _1 ) );
    move3d_set_fct_localpath_path_destructor( boost::bind( move3d_localpath_simple_destructor, _1 ) );
    // move3d_set_fct_localpath_copy_from_struct( boost::bind( move3d_localpath_simple_init_from_struct, _1, _2, _3, _4 ) );
    move3d_set_fct_localpath_copy_from_struct( boost::bind( move3d_localpath_init_from_struct, _1, _2, _3, _4 ) );

    move3d_set_fct_localpath_get_struct( boost::bind( move3d_localpath_get_localpath_struct, _1, _2, _3 ) );
    // move3d_set_fct_localpath_get_struct( boost::bind( move3d_localpath_simple_get_localpath_struct, _1, _2, _3 ) );

    move3d_set_fct_localpath_classic_test( boost::bind( move3d_localpath_simple_classic_test, _1, _2, _3, _4 ) );
    move3d_set_fct_localpath_is_valid( boost::bind( move3d_localpath_simple_is_valid_test, _1, _2 ) );
    move3d_set_fct_localpath_get_length( boost::bind( move3d_localpath_simple_length, _1 ) );
    move3d_set_fct_localpath_get_param_max( boost::bind( move3d_localpath_simple_max_param, _1 ) );
    move3d_set_fct_localpath_config_at_dist( boost::bind( move3d_localpath_simple_config_at_dist, _1, _2 ) );
    move3d_set_fct_localpath_config_at_param( boost::bind( move3d_localpath_simple_config_at_param, _1, _2 ) );
    move3d_set_fct_localpath_stay_within_dist( boost::bind( move3d_localpath_simple_stay_within_dist, _1, _2, _3, _4 ) );
}
