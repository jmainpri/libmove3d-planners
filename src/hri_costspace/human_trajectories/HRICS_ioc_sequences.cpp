#include "HRICS_ioc_sequences.hpp"

#include "HRICS_parameters.hpp"
#include "HRICS_ioc.hpp"

#include "API/project.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "utils/misc_functions.hpp"
#include "utils/NumsAndStrings.hpp"

#include "feature_space/spheres.hpp"
#include "feature_space/squares.hpp"

#include "planner/planEnvironment.hpp"

#include <libmove3d/include/Graphic-pkg.h>

using namespace Move3D;
using namespace HRICS;
using std::cout;
using std::endl;

static std::string move3d_demo_folder("/home/jmainpri/Dropbox/move3d/assets/IOC/TRAJECTORIES/");
static std::string move3d_traj_folder("/home/jmainpri/Dropbox/move3d/move3d-launch/matlab/stomp_trajs_home/per_feature_square/");
static std::string move3d_tmp_data_folder("matlab/move3d_tmp_data_home/");

IocSequences::IocSequences()
{

}

bool IocSequences::run()
{
    cout << "************************************************************" << endl;
    cout << __PRETTY_FUNCTION__ << endl;
    cout << "************************************************************" << endl;

    Robot* rob = global_Project->getActiveScene()->getActiveRobot();
    if (!rob) {
        cout << "robot not initialized in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    confPtr_t q_init( rob->getInitPos() );
    confPtr_t q_goal( rob->getGoalPos() );
    if( *q_init == *q_goal )
    {
        cout << "init equal q_goal in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    if( global_PlanarCostFct == NULL )
    {
        cout << "global_PlanarCostFct not initialized in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    // IOC PHASES
    enum phase_t { generate=0, sample=1, compare=2, run_planner=3, default_phase=4, monte_carlo=5 };

    // LOAD PARAMETERS FROM SETTING FILE
    int nb_way_points       = HriEnv->getInt(HricsParam::ioc_nb_of_way_points);
    bool single_iteration   = HriEnv->getBool(HricsParam::ioc_single_iteration);
    int nb_iterations       = HriEnv->getInt(HricsParam::ioc_sample_iteration);
    bool sample_from_file   = HriEnv->getBool(HricsParam::ioc_load_samples_from_file);
    int file_offset         = HriEnv->getInt(HricsParam::ioc_from_file_offset);
    phase_t phase           = (phase_t)HriEnv->getInt(HricsParam::ioc_phase);

    cout << "ioc phase : " << phase << endl;

    int nb_demos = 1;
    int nb_sampling_phase = 10;
    //    int min_samples = 10;
    //    int max_samples = 100;

    bool StopRun = false;
    std::vector<Eigen::VectorXd> results;
    int iteration = 0;

    MultiplePlanners planners(rob);
    if( sample_from_file )
    // planners.loadTrajsFromFile( move3d_traj_folder + "FIRST_TRY" );
    // planners.loadTrajsFromFile( move3d_traj_folder + "THIRD_TRY" );
    // planners.loadTrajsFromFile( move3d_traj_folder + "STOMP_LARGE" );
    // planners.loadTrajsFromFile( move3d_traj_folder + "GENERAL_COSTMAP");
    // planners.loadTrajsFromFile( move3d_traj_folder + "STOMP_COMBINE" );
    planners.loadTrajsFromFile( move3d_traj_folder + "STOMP_VARIANCE_F1" );
    // planners.loadTrajsFromFile( move3d_traj_folder + "RANDOM_05" );

    for(int i=0; i<nb_sampling_phase && !StopRun; i++)
    {
        // iteration = i; // 2, 5, 30, 50

        if( single_iteration )
            iteration = nb_iterations;
        else
            iteration = i;

        // interpolation for the number of sampling phase
        // int nb_samples = min_samples + double(iteration)*(max_samples-min_samples)/double(nb_sampling_phase-1);
        // int nb_samples = (iteration*100+1);
        int nb_samples = (16*(iteration+1));
        // int nb_samples = (160*(iteration+1));

        cout << "NB SAMPLES : " << nb_samples << endl;

        IocEvaluation eval( rob, nb_demos, nb_samples, nb_way_points, planners, move3d_demo_folder, move3d_traj_folder, move3d_tmp_data_folder );

        cout << "------------------------------" << endl;
        cout << " RUN : " << iteration << endl;
        cout << "------------------------------" << endl;

        switch( phase )
        {
        case generate:
            cout << "GENERATE" << endl;
            eval.generateDemonstrations();
            g3d_draw_allwin_active();
            break;

        case sample:
            cout << "SAMPLE" << endl;
            eval.loadDemonstrations();
            // eval.runLearning();

            if( sample_from_file )
                eval.runFromFileSampling( file_offset );
            else
                eval.runSampling();

            g3d_draw_allwin_active();
            break;

        case compare:
            cout << "COMPARE" << endl;
            results.push_back( eval.compareDemosAndPlanned() );
            g3d_draw_allwin_active();
            break;

        case run_planner:
            cout << "RUN MULTI-PLANNER" << endl;
            eval.loadDemonstrations();
            // eval.runPlannerMultipleFeature( 50 ); // 10
            eval.runPlannerWeightedFeature( 50 ); // 50 * 16 = 800
            StopRun = true;
            break;

        case monte_carlo:
            eval.loadDemonstrations();
            eval.monteCarloSampling( 10.0, 10 );
            break;

        default:
            cout << "DEFAULT : LOAD TRAJECTORIES" << endl;
            eval.loadPlannerTrajectories( 16, 16*i, 0 );
            // StopRun = true;
            break;
        }

        if( single_iteration )
            break;

        if ( PlanEnv->getBool(PlanParam::stopPlanner) ) {
            StopRun = true;
        }
    }

    if( !results.empty() )
    {
        Eigen::MatrixXd mat( results.size(), results[0].size() );
        for( int i=0;i<mat.rows();i++)
        {
            mat.row(i) = results[i];
        }

        move3d_save_matrix_to_file( mat, move3d_tmp_data_folder + "result.txt" );
    }

    return true;
}
