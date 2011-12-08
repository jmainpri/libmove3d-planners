
#include "OtpUtils.hpp"


#include "P3d-pkg.h"
//#include "move3d-headless.h"
#include "Planner-pkg.h"
#include "planEnvironment.hpp"

using namespace std;
using namespace tr1;
using namespace HRICS;

int ConfigHR::index = 0;

/**
 * show trajectory of both robot and human
 */
void g3d_show_tcur_both_rob(p3d_rob *robotPt, int (*fct)(p3d_rob* robot, p3d_localpath* curLp),
                            p3d_rob *hum_robotPt, int (*hum_fct)(p3d_rob* hum_robot, p3d_localpath* hum_curLp))
{

  if (robotPt->tcur == NULL)
  {
    PrintInfo(("g3d_show_tcur_both_rob : no current trajectory\n"));
    return;
  }

  if (hum_robotPt->tcur == NULL)
  {
      PrintInfo(("g3d_show_tcur_both_rob : no trajectoryfor human\n"));
  }

  G3D_Window *win;
//  configPt q;
  configPt q_rob;
  configPt q_hum;
  pp3d_localpath localpathPt;
  pp3d_localpath humLocalpathPt;

  int end = FALSE;
//  int njnt = robotPt->njoints;
  double u = 0, du, umax; /* parameters along the local path */
  double uHum = 0, duHum, umaxHum; /* parameters along the local path */

//  robotPt->draw_transparent = false;

  win = g3d_get_cur_win();
//  win->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;
  //g3d_draw_env_box();
//  g3d_draw_obstacles(win);
#ifdef P3D_PLANNER
  if(XYZ_GRAPH && ENV.getBool(Env::drawGraph)){g3d_draw_graph();}
#endif

  umax = p3d_compute_traj_rangeparam(robotPt->tcur);
  umaxHum = p3d_compute_traj_rangeparam(hum_robotPt->tcur);

  localpathPt = robotPt->tcur->courbePt;
  humLocalpathPt = hum_robotPt->tcur->courbePt;

  int loopOut = 0;
  double robotSpeed = PlanEnv->getDouble(PlanParam::env_robotSpeed);
  double humanSpeed = PlanEnv->getDouble(PlanParam::env_humanSpeed);
  clock_t start = clock();
  while (loopOut == 0)
  {
    /* position of the robot corresponding to parameter u */
    if (u < umax - EPS6)
    {
        q_rob = p3d_config_at_param_along_traj(robotPt->tcur,u);
        p3d_set_and_update_this_robot_conf_multisol(robotPt, q_rob, NULL, 0, localpathPt->ikSol);
//        p3d_set_and_update_robot_conf(q_rob);
    }
    if (uHum < umaxHum - EPS6)
    {
        q_hum = p3d_config_at_param_along_traj(hum_robotPt->tcur,uHum);
        p3d_set_and_update_this_robot_conf_multisol(hum_robotPt, q_hum, NULL, 0, humLocalpathPt->ikSol);
//        p3d_set_and_update_robot_conf(q_hum);
    }

    g3d_draw_allwin_active();
    if (fct) if (((*fct)(robotPt, localpathPt)) == FALSE) return;
    if (hum_fct) if (((*hum_fct)(hum_robotPt, humLocalpathPt)) == FALSE) return;

    //((double)gridInit - start) / CLOCKS_PER_SEC
    if (u < umax - EPS6)
    {
        du = ENV.getDouble(Env::showTrajFPS)*(robotSpeed * ((double)clock() - start) / CLOCKS_PER_SEC);/* localpathPt->stay_within_dist(robotPt, localpathPt,*/
    }

    if (uHum < umaxHum - EPS6)
    {
        duHum = ENV.getDouble(Env::showTrajFPS)*(humanSpeed * ((double)clock() - start) / CLOCKS_PER_SEC);/* localpathPt->stay_within_dist(robotPt, localpathPt,*/
    }

      u = du;
      if (u > umax - EPS6) {
        u = umax;
        end = TRUE;
      }

      uHum = duHum;
      if (uHum > umaxHum - EPS6) {
        uHum = umaxHum;
        end = TRUE;
      }
      if (uHum > umaxHum - EPS6 && u > umax - EPS6)
      {
          loopOut++;
      }
  }
}



void ConfigHR::setHumanConf(Robot* human, configPt q)
{
        q_hum = p3d_copy_config(human->getRobotStruct(),q);
}

void ConfigHR::setRobotConf(Robot* robot, configPt q)
{
        q_rob = p3d_copy_config(robot->getRobotStruct(),q);
}

void OutputConf::clearAll()
{
//    humanConf = NULL;
    humanTraj.clear();
    humanTrajExist = false;

//    robotConf = NULL;
    robotTraj.clear();
    robotTrajExist = false;

    cost = numeric_limits<double>::max( );
    configNumberInList = -1;
    id = -1;
    isStandingInThisConf = true;
}