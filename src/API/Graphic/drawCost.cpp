/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001).
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014
 */

#ifdef HRI_COSTSPACE
#include "hri_costspace/HRICS_costspace.hpp"
#include "hri_costspace/HRICS_otpmotionpl.hpp"
#include "hri_costspace/HRICS_config_space.hpp"
#include "hri_costspace/HRICS_navigation.hpp"
#include "hri_costspace/gestures/HRICS_workspace_occupancy.hpp"
#include "hri_costspace/gestures/HRICS_human_prediction_cost_space.hpp"
#include "hri_costspace/gestures/HRICS_gest_parameters.hpp"
#endif

#include "planner/cost_space.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "planner/TrajectoryOptim/Stomp/run_parallel_stomp.hpp"

#include "API/ConfigSpace/configuration.hpp"
#include "API/Graphic/drawModule.hpp"
#include "API/project.hpp"
#include "API/Grids/gridsAPI.hpp"

#include "collision_space/collision_space.hpp"

#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "GroundHeight-pkg.h"

#ifdef P3D_COLLISION_CHECKING
#include "Collision-pkg.h"
#endif

////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <iostream>

using namespace std;
using namespace Move3D;
MOVE3D_USING_SHARED_PTR_NAMESPACE

Eigen::Vector3d global_DrawnSphere;
vector<Eigen::Vector3d> CXX_drawBox;
vector<double> vect_jim;
double cost_max = 30.0;

extern Eigen::Vector3d current_WSPoint;
extern pair<double, Eigen::Vector3d> current_cost;
extern std::string hri_text_to_display;
extern std::vector<Eigen::Vector3d> OTPList;
extern std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
    path_to_draw;
extern double ZminEnv;
extern double ZmaxEnv;
extern void* GroundCostObj;

void drawGauge(int number, double cost);

#ifdef HRI_COSTSPACE
void g3d_draw_hrics(int opengl_context);
#endif

//! this functions draws registred
//! functions
void g3d_draw_registred_functions() {}

//! @ingroup graphic
//! Function drawing a box the V7 is bellow V5
//
//     V5 -- V6
//    /      / |
//   V1 -- V2 V8
//   |      | /
//   V3 -- V4
//
void g3d_draw_eigen_box(const Eigen::Vector3d& v1,
                        const Eigen::Vector3d& v2,
                        const Eigen::Vector3d& v3,
                        const Eigen::Vector3d& v4,
                        const Eigen::Vector3d& v5,
                        const Eigen::Vector3d& v6,
                        const Eigen::Vector3d& v7,
                        const Eigen::Vector3d& v8,
                        int color,
                        int fill,
                        double width) {
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  if (fill == 0) {
    /* filaire */
    g3d_set_color(color, NULL);
  }

  glPushAttrib(GL_LINE_BIT);
  glLineWidth(width);
  glBegin(GL_LINES);
  glVertex3d(v1[0], v1[1], v1[2]);
  glVertex3d(v3[0], v3[1], v3[2]);

  glVertex3d(v2[0], v2[1], v2[2]);
  glVertex3d(v4[0], v4[1], v4[2]);

  glVertex3d(v5[0], v5[1], v5[2]);
  glVertex3d(v7[0], v7[1], v7[2]);

  glVertex3d(v6[0], v6[1], v6[2]);
  glVertex3d(v8[0], v8[1], v8[2]);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex3d(v1[0], v1[1], v1[2]);
  glVertex3d(v2[0], v2[1], v2[2]);
  glVertex3d(v6[0], v6[1], v6[2]);
  glVertex3d(v5[0], v5[1], v5[2]);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex3d(v3[0], v3[1], v3[2]);
  glVertex3d(v4[0], v4[1], v4[2]);
  glVertex3d(v8[0], v8[1], v8[2]);
  glVertex3d(v7[0], v7[1], v7[2]);
  glEnd();

  glPopAttrib();
}
//#endif

std::vector<Move3D::confPtr_t> global_configToDraw;

void g3d_draw_configurations() {
  G3D_Window* win = g3d_get_cur_win();
  p3d_numcoll = false;
  for (int i = 0; i < int(global_configToDraw.size()); i++) {
    Move3D::Robot* robot = global_configToDraw[i]->getRobot();
    robot->setAndUpdate(*global_configToDraw[i]);
    win->vs.transparency_mode = G3D_TRANSPARENT_AND_OPAQUE;
    g3d_draw_robot(robot->getP3dRobotStruct()->num, win, 0);
    win->vs.transparency_mode = G3D_TRANSPARENT_AND_OPAQUE;
  }
}

std::vector<std::pair<Eigen::Vector3d, Eigen::MatrixXd> > global_linesToDraw;

void g3d_draw_3d_lines() {
  double color[4];
  color[0] = 1.0;
  color[1] = 0.8;
  color[2] = 0.2;
  color[3] = 1.0;

  const double size = 0.005;  // 0.005

  for (int l = 0; l < int(global_linesToDraw.size()); l++)
    for (int i = 0; i < int(global_linesToDraw[l].second.cols() - 1); i++) {
      color[0] = global_linesToDraw[l].first[0];
      color[1] = global_linesToDraw[l].first[1];
      color[2] = global_linesToDraw[l].first[2];

      g3d_set_color(Any, color);

      g3d_draw_solid_sphere(global_linesToDraw[l].second(0, i + 0),
                            global_linesToDraw[l].second(1, i + 0),
                            global_linesToDraw[l].second(2, i + 0),
                            size,
                            10);

      g3d_drawOneLine(global_linesToDraw[l].second(0, i + 0),
                      global_linesToDraw[l].second(1, i + 0),
                      global_linesToDraw[l].second(2, i + 0),
                      global_linesToDraw[l].second(0, i + 1),
                      global_linesToDraw[l].second(1, i + 1),
                      global_linesToDraw[l].second(2, i + 1),
                      Any,
                      color);

      if (i == int(global_linesToDraw[l].second.cols() - 2)) {
        g3d_draw_solid_sphere(global_linesToDraw[l].second(0, i + 1),
                              global_linesToDraw[l].second(1, i + 1),
                              global_linesToDraw[l].second(2, i + 1),
                              size,
                              10);
      }
    }
}

void g3d_draw_multistomp_lines() {
  std::map<Robot*, std::vector<Eigen::Vector3d> >::const_iterator itr;
  std::map<Robot*, std::vector<Eigen::Vector3d> >::const_iterator itr_next;
  std::map<Robot*, std::vector<Eigen::Vector3d> >::const_iterator itr_end;
  //    cout << "draw multiple stomp lines" << endl;

  for (itr = global_MultiStomplinesToDraw.begin();
       itr != global_MultiStomplinesToDraw.end();
       ++itr) {
    double color[4];

    if (!global_MultiStomplinesColors.empty()) {
      color[0] = global_MultiStomplinesColors[itr->first][0];
      color[1] = global_MultiStomplinesColors[itr->first][1];
      color[2] = global_MultiStomplinesColors[itr->first][2];
      color[3] = global_MultiStomplinesColors[itr->first][3];
    } else {
      color[0] = 0.8;
      color[1] = 1.0;
      color[2] = 0.2;
      color[3] = 1.0;
    }

    itr_next = itr;
    itr_next++;

    if (itr_next != global_MultiStomplinesToDraw.end()) {
      itr_end = itr_next;
      itr_end++;

      g3d_set_color(Any, color);

      for (int i = 0; i < int(itr->second.size()); i++) {
        Eigen::Vector3d pi = itr->second[i];
        Eigen::Vector3d pf = itr_next->second[i];

        if ((!ENV.getBool(Env::isCostSpace)) || (GroundCostObj == NULL)) {
          g3d_draw_solid_sphere(pi[0], pi[1], pi[2], 0.02, 10);
          g3d_drawOneLine(pi[0], pi[1], pi[2], pf[0], pf[1], pf[2], Any, color);
        } else {
          double Cost1;
          GHintersectionVerticalLineWithGround(
              GroundCostObj, pi[0], pi[1], &Cost1);
          double Cost2;
          GHintersectionVerticalLineWithGround(
              GroundCostObj, pf[0], pf[1], &Cost2);
          glLineWidth(3.);
          g3d_drawOneLine(pi[0],
                          pi[1],
                          Cost1 + (ZmaxEnv - ZminEnv) * 0.02,
                          pf[0],
                          pf[1],
                          Cost2 + (ZmaxEnv - ZminEnv) * 0.02,
                          Red,
                          NULL);
          glLineWidth(3.);

          if (itr_end == global_MultiStomplinesToDraw.end())
            g3d_drawSphere(
                pf[0], pf[1], Cost2 + (ZmaxEnv - ZminEnv) * 0.02, 1.);
        }
      }
    }
  }
}

/*
 * Draws the things related to cost spaces
 */
void g3d_draw_costspace() {
  if (ENV.getBool(Env::isCostSpace)) {
    for (int num = 0; num < 2; num++) {
      for (int it = 0; it < 3; it++) {
#ifdef P3D_COLLISION_CHECKING
        if (vectMinDist[num][it] != 0) {
          g3d_drawOneLine(vectMinDist[0][0],
                          vectMinDist[0][1],
                          vectMinDist[0][2],
                          vectMinDist[1][0],
                          vectMinDist[1][1],
                          vectMinDist[1][2],
                          Red,
                          NULL);
          break;
        }
#endif
      }
    }
  }
}

void g3d_draw_bounding_box(Robot* robot) {
  double box[8][3];
  Eigen::Vector3d p1;

  Joint* jnt = robot->getJoint(1);
  p3d_obj* object = static_cast<p3d_jnt*>(jnt->getP3dJointStruct())->o;

  if (object) {
    if (pqp_get_OBB_first_level(object, box)) {
      for (int j = 0; j < 8; j++) {
        p1[0] = box[j][0];
        p1[1] = box[j][1];
        p1[2] = box[j][2];

        p1 = jnt->getMatrixPos() * p1;

        box[j][0] = p1[0];
        box[j][1] = p1[1];
        box[j][2] = p1[2];
      }

      //            g3d_draw_complex_black_box( box );
    }
  }
}

void g3d_draw_squeleton() {
  double boxA[8][3];
  double boxB[8][3];

  CXX_drawBox.resize(8);

  Robot* robot =
      global_Project->getActiveScene()->getRobotByNameContaining("PR2");

  for (unsigned int i = 0; i < robot->getNumberOfJoints(); i++) {
    Joint* jnt = robot->getJoint(i);
    p3d_obj* obj = static_cast<p3d_jnt*>(jnt->getP3dJointStruct())->o;

    if (obj) {
      Eigen::Vector3d vect1, vect2, vect5, vect6;
      // pqp_draw_OBB_first_level(obj);

      if (pqp_get_OBB_first_level(obj, boxA)) {
        for (int j = 0; j < 8; j++) {
          CXX_drawBox[j][0] = boxA[j][0];
          CXX_drawBox[j][1] = boxA[j][1];
          CXX_drawBox[j][2] = boxA[j][2];

          boxA[j][0] = CXX_drawBox[j][0];
          boxA[j][1] = CXX_drawBox[j][1];
          boxA[j][2] = CXX_drawBox[j][2];

          //          if (j == 1)
          //          { vect1 = CXX_drawBox[j]; }
          //          if (j == 2)
          //          { vect2 = CXX_drawBox[j]; }
          //          if (j == 5)
          //          { vect5 = CXX_drawBox[j]; }
          //          if (j == 6)
          //          { vect6 = CXX_drawBox[j]; }

          CXX_drawBox[j] = jnt->getMatrixPos() * CXX_drawBox[j];

          boxB[j][0] = CXX_drawBox[j][0];
          boxB[j][1] = CXX_drawBox[j][1];
          boxB[j][2] = CXX_drawBox[j][2];

          //          if (j == ENV.getDouble(Env::extensionStep)) {
          //            continue;
          //          }

          double alpha = (double)j / 8;
          double colorvector[4];
          GroundColorMixGreenToRed(colorvector, alpha);

          //          glColor4dv(colorvector);
          //          g3d_draw_simple_box(boxA[j][0],boxA[j][0]+0.02,
          //                              boxA[j][1],boxA[j][1]+0.02,
          //                              boxA[j][2],boxA[j][2]+0.02, FALSE,
          //                              TRUE, 0.01);

          glColor4dv(colorvector);
          g3d_draw_simple_box(boxB[j][0],
                              boxB[j][0] + 0.02,
                              boxB[j][1],
                              boxB[j][1] + 0.02,
                              boxB[j][2],
                              boxB[j][2] + 0.02,
                              FALSE,
                              TRUE,
                              0.01);
        }

        // g3d_draw_complex_black_box(boxA);
        g3d_draw_complex_black_box(boxB);

        vect1 = CXX_drawBox[1];
        vect2 = CXX_drawBox[2];
        vect5 = CXX_drawBox[5];
        vect6 = CXX_drawBox[6];

        Eigen::Vector3d p1Eigen = 0.5 * (vect1 + vect2);
        Eigen::Vector3d p2Eigen = 0.5 * (vect5 + vect6);

        g3d_draw_simple_box(p1Eigen[0],
                            p1Eigen[0] + 0.02,
                            p1Eigen[1],
                            p1Eigen[1] + 0.02,
                            p1Eigen[2],
                            p1Eigen[2] + 0.02,
                            FALSE,
                            FALSE,
                            0.01);

        g3d_draw_simple_box(p2Eigen[0],
                            p2Eigen[0] + 0.02,
                            p2Eigen[1],
                            p2Eigen[1] + 0.02,
                            p2Eigen[2],
                            p2Eigen[2] + 0.02,
                            FALSE,
                            FALSE,
                            0.01);

        double p1[3], p2[3];
        double radius = 0.5 * ((vect1 - vect2).norm());

        p1[0] = p1Eigen[0];
        p1[1] = p1Eigen[1];
        p1[2] = p1Eigen[2];

        p2[0] = p2Eigen[0];
        p2[1] = p2Eigen[1];
        p2[2] = p2Eigen[2];

        double colorvector[4];
        colorvector[0] = 1.0;
        colorvector[1] = 1.0;
        colorvector[2] = 0.0;
        colorvector[3] = 0.5;

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glColor4dv(colorvector);
        g3d_draw_cylinder(p1, p2, radius, 20);

        glDisable(GL_BLEND);
      }
    }
  }
}

void g3d_draw_grids() {
#ifdef HRI_COSTSPACE

  if (HRICS_humanCostMaps && ENV.getBool(Env::drawDistance)) {
    HRICS_humanCostMaps->drawDistances();
  }

  if (HRICS_humanCostMaps && PlanEnv->getBool(PlanParam::drawReachableGrid)) {
    HRICS_humanCostMaps->drawReachableGrid();
    ENV.setBool(Env::drawGrid, false);
  }

  if (ENV.getBool(Env::drawEntireGrid) && API_activeGrid) {
    ENV.setBool(Env::drawGrid, true);
  }
  //-------------------------------------------------------------
  if (ENV.getBool(Env::drawGrid) && API_activeGrid) {
    API_activeGrid->draw();

    if (ENV.getBool(Env::drawBox)) {
      CXX_drawBox = API_activeGrid->getBox();

      if (!CXX_drawBox.empty()) {
        g3d_draw_eigen_box(CXX_drawBox[0],
                           CXX_drawBox[1],
                           CXX_drawBox[2],
                           CXX_drawBox[3],
                           CXX_drawBox[4],
                           CXX_drawBox[5],
                           CXX_drawBox[6],
                           CXX_drawBox[7],
                           Red,
                           0,
                           3);
      }
    }
  }
  //-------------------------------------------------------------
  if (ENV.getBool(Env::drawBox)) {
    Robot* robot = global_Project->getActiveScene()->getActiveRobot();
    CXX_drawBox = robot->getObjectBox();

    if (!CXX_drawBox.empty()) {
      g3d_draw_eigen_box(CXX_drawBox[0],
                         CXX_drawBox[1],
                         CXX_drawBox[2],
                         CXX_drawBox[3],
                         CXX_drawBox[4],
                         CXX_drawBox[5],
                         CXX_drawBox[6],
                         CXX_drawBox[7],
                         Red,
                         0,
                         3);

      Joint* jnt = robot->getJoint("rPalm");
      if (/*jnt != NULL*/ false) {
        Eigen::Vector3d pos = jnt->getVectorPos();

        g3d_draw_solid_sphere(pos[0], pos[1], pos[2], .05, 20);

        if (robot->isInObjectBox(pos)) {
          g3d_set_custom_color_draw(robot->getP3dRobotStruct(), true);

          GLdouble color_vect[4];
          color_vect[0] = 0;
          color_vect[1] = 1;
          color_vect[2] = 0;
          color_vect[3] = 0;
          g3d_set_custom_color_vect(color_vect);
        } else {
          g3d_set_custom_color_draw(robot->getP3dRobotStruct(), false);
        }
      }
    }
  }
  //-------------------------------------------------------------
  if (HRICS_activeNatu) {
    HRICS_activeNatu->printBodyPos();
  }

#endif

  if (global_collisionSpace && ENV.getBool(Env::drawVectorField)) {
    global_collisionSpace->drawGradient();
  }
  if (global_collisionSpace && PlanEnv->getBool(PlanParam::drawOccupVoxels)) {
    global_collisionSpace->draw();
    // global_collisionSpace->drawGradient();
  }

  if (global_collisionSpace && PlanEnv->getBool(PlanParam::drawStaticVoxels)) {
    global_collisionSpace->drawStaticVoxels();
  }

  if (global_collisionSpace && PlanEnv->getBool(PlanParam::drawSampledPoints)) {
    global_collisionSpace->getBodySampler()->draw();
  }

  if (global_collisionSpace && PlanEnv->getBool(PlanParam::drawStaticVoxels)) {
    global_collisionSpace->drawSquaredDist();
  }

  if (global_collisionSpace &&
      PlanEnv->getBool(PlanParam::drawBoundingVolumes)) {
    // traj_optim_draw_collision_points();
    global_collisionSpace->drawCollisionPoints();
  }

  if (global_optimizer && ENV.getBool(Env::drawTraj)) {
    global_optimizer->draw();
  }

  if (ENV.getBool(Env::drawPoints)) {
    if (PointsToDraw != NULL) {
      // cout << "PointsToDraw = " << PointsToDraw << endl;
      PointsToDraw->drawAllPoints();
    }
  }

  // Draws a sphere of 10 cm of radius
  g3d_drawSphere(global_DrawnSphere(0),
                 global_DrawnSphere(1),
                 global_DrawnSphere(2),
                 0.02);
}

void drawGauge(int number, double cost) {
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  double cylinderColorvector[4];

  cylinderColorvector[1] = 1.0;  // green
  cylinderColorvector[2] = 0.0;  // blue
  cylinderColorvector[0] = 0.0;  // red
  cylinderColorvector[3] = 0.7;  // transparency

  GroundColorMixGreenToRed(cylinderColorvector, cost);

  g3d_set_color(Any, cylinderColorvector);

  g3d_draw_solid_cylinder(1.0, -1.0 - (number * 0.5), 0.09, cost * 2, 30);

  cylinderColorvector[1] = 0.5;  // green
  cylinderColorvector[2] = 0.5;  // blue
  cylinderColorvector[0] = 0.5;  // red
  cylinderColorvector[3] = 0.4;  // transparency

  g3d_set_color(Any, cylinderColorvector);
  g3d_draw_solid_cylinder(1.0, -1.0 - (number * 0.5), 0.10, 2, 30);

  glDisable(GL_BLEND);
}

void drawSlice(int opengl_context);

/**
 * @ingroup graphics
 * Draws the thing related to HRI_COSTSPACE
 */
//#ifdef HRI_COSTSPACE
void g3d_draw_hrics(int opengl_context) {
  if (PlanEnv->getBool(PlanParam::drawParallelTraj))
    g3d_draw_multistomp_lines();

  if (GestEnv->getBool(GestParam::draw_human_sampled_points) &&
      global_workspaceOccupancy)
    global_workspaceOccupancy->drawSampledPoints();

  if (GestEnv->getBool(GestParam::draw_ws_occupancy) &&
      global_workspaceOccupancy)
    global_workspaceOccupancy->draw();

  if (GestEnv->getBool(GestParam::draw_robot_sampled_points) &&
      global_humanPredictionCostSpace)
    global_humanPredictionCostSpace->draw();

  for (int i = 0; i < int(global_motionRecorders.size()); i++) {
    global_motionRecorders[i]->draw();
  }

  int OTPListSize = OTPList.size();
  if (ENV.getBool(Env::enableHri)) {
    if (ENV.getBool(Env::HRIPlannerCS) && ENV.getBool(Env::drawTraj)) {
      //          printf("Draw 2d path\n");
      // dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->draw2dPath();
    }

    if (ENV.getBool(Env::drawOTPTraj) &&
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)) {
      dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->draw2dPath();
    }

    if (ENV.getBool(Env::isCostSpace)) {
      if (ENV.getBool(Env::enableHri)) {
        if (ENV.getBool(Env::HRIPlannerWS) && ENV.getBool(Env::drawTraj) &&
            HRICS_MotionPL != NULL) {
          //              printf("Draw 3d path\n");
          dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->draw3dPath();
        }
      }
    }
  }

  if (ENV.getBool(Env::drawTraj)) {
    if (!path_to_draw.empty()) {
      for (unsigned int i = 0; i < path_to_draw.size() - 1; i++) {
        glLineWidth(3.);
        g3d_drawOneLine(path_to_draw[i][0],
                        path_to_draw[i][1],
                        0.4,
                        path_to_draw[i + 1][0],
                        path_to_draw[i + 1][1],
                        0.4,
                        Blue,
                        NULL);
        glLineWidth(1.);

        double colorvector[4];
        colorvector[0] = 0.0;  // red
        colorvector[1] = 0.0;  // green
        colorvector[2] = 1.0;  // blue
        colorvector[3] = 1;  // transparency

        //        glEnable(GL_BLEND);
        g3d_set_color(Any, colorvector);

        g3d_draw_solid_sphere(
            path_to_draw[i + 1][0], path_to_draw[i + 1][1], 0.4, 0.05, 20);
        //            cout << "\n\n cell: \n" << m_2DPath[i] << endl;
      }
    }
  }

  //  if( HRICS_activeDist )
  //	{
  //		HRICS_activeDist->drawInteractionZone();
  //  }
  //
  //  if (HRICS_MotionPL) {
  //    dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->drawCurrentOTP();
  //  }

  if (ENV.getBool(Env::drawDistance) && HRICS_activeDist) {
    vect_jim = HRICS_activeDist->getVectorJim();

    glLineWidth(3.);

    for (unsigned int i = 0; i < vect_jim.size() / 6; i++) {
      g3d_drawOneLine(vect_jim[0 + 6 * i],
                      vect_jim[1 + 6 * i],
                      vect_jim[2 + 6 * i],
                      vect_jim[3 + 6 * i],
                      vect_jim[4 + 6 * i],
                      vect_jim[5 + 6 * i],
                      Red,
                      NULL);
    }

    glLineWidth(1.);
  }

  //    if( HRICS_activeNatu && PlanEnv->getBool(PlanParam::drawColorConfig) )
  //    {
  //      HRICS_activeNatu->setRobotColorFromConfiguration(true);
  //    }

  if (ENV.getBool(Env::drawGaze) &&
      (ENV.getBool(Env::HRIPlannerWS) || ENV.getBool(Env::HRIPlannerCS))) {
    vector<double> Gaze;
    Gaze.clear();

    // cout << "Draw Gaze" << endl;
    Gaze = HRICS_MotionPL->getVisibility()->getGaze();

    glLineWidth(3.);

    if ((Gaze.size() == 6)) {
      g3d_drawOneLine(
          Gaze[0], Gaze[1], Gaze[2], Gaze[3], Gaze[4], Gaze[5], Blue, NULL);
    }

    glLineWidth(1.);

    //    GLdouble GreenColor[4] =   { 0.0, 0.5, 0.0, 0.7 };
    //    GLdouble GreenColorT[4] =   { 0.0, 0.5, 0.0, 0.0 };
    //    //    GLdouble GreyColor[4] =   { 0.5, 0.5, 0.5, 0.5 };
    //    //    GLdouble GreyColorT[4] =   { 0.5, 0.5, 0.5, 0.0 };
    //
    //
    //    Robot* human =
    //    dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getHuman();
    //    int gazeIndex = 46;
    //    if ( human->getName() == "ACHILE_HUMAN1" )
    //    {
    //      gazeIndex = 42;
    //    }
    //
    //    //    cout << "HUMAN = " << human->getName() << endl;
    //    p3d_jnt* eyes = human->getJoint(gazeIndex)->getP3dJointStruct();
    //
    //    // 46 is for HERAKLES
    //    // 42 is for ACHILE
    //    g3d_draw_visibility_by_frame(eyes->abs_pos,DTOR(160),DTOR(160*0.75),1,
    //    GreenColor, GreenColorT);
  }

  if (HRICS_MotionPL != NULL && PlanEnv->getBool(PlanParam::drawRandomMap)) {
    double depth = 1.0;

    confPtr_t q_hum = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)
                          ->getHuman()
                          ->getCurrentPos();
    int indexFirstDof = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)
                            ->getHuman()
                            ->getJoint("Pelvis")
                            ->getIndexOfFirstDof();

    double xMin = (*q_hum)[indexFirstDof + 0] +
                  PlanEnv->getDouble(PlanParam::env_randomXMinLimit);
    double xMax = (*q_hum)[indexFirstDof + 0] +
                  PlanEnv->getDouble(PlanParam::env_randomXMaxLimit);
    double yMin = (*q_hum)[indexFirstDof + 1] +
                  PlanEnv->getDouble(PlanParam::env_randomYMinLimit);
    double yMax = (*q_hum)[indexFirstDof + 1] +
                  PlanEnv->getDouble(PlanParam::env_randomYMaxLimit);
    glLineWidth(3.);
    g3d_drawOneLine(xMin, yMin, depth, xMin, yMax, depth, Black, NULL);
    glLineWidth(1.);

    glLineWidth(3.);
    g3d_drawOneLine(xMin, yMax, depth, xMax, yMax, depth, Black, NULL);
    glLineWidth(1.);

    glLineWidth(3.);
    g3d_drawOneLine(xMax, yMax, depth, xMax, yMin, depth, Black, NULL);
    glLineWidth(1.);

    glLineWidth(3.);
    g3d_drawOneLine(xMax, yMin, depth, xMin, yMin, depth, Black, NULL);
    glLineWidth(1.);
  }

  if (HRICS_MotionPL) {
    //    bool rightHand = true;
    //    Eigen::Vector3d WSPoint =
    //    dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->computeOTPFromHandPose(
    //    rightHand );
    //
    //    double color_array[4];
    //
    //    glEnable(GL_BLEND);
    //    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //
    //    color_array[0]= 0.0;
    //    color_array[1]= 0.0;
    //    color_array[2]= 1.0;
    //    color_array[3]= 0.7;
    //    g3d_set_color(Any,color_array);
    //    g3d_draw_solid_sphere(WSPoint[0], WSPoint[1], WSPoint[2], 0.10, 10);
    //
    //    glDisable(GL_BLEND);
  }

  //"HRI cost = %2.2f"

  if (current_WSPoint(0) != 0 && current_WSPoint(1) != 0) {
    double colorvector[4];

    colorvector[1] = 1.0;  // green
    colorvector[2] = 0.0;  // blue

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //                Vector3d center = getWorkspacePoint();

    colorvector[0] = 0.0;  // red
    colorvector[3] = 0.9;  // transparency

    g3d_set_color(Any, colorvector);
    g3d_draw_solid_sphere(
        current_WSPoint[0], current_WSPoint[1], current_WSPoint[2], 0.02, 10);
    //                        GroundColorMixGreenToRed(colorvector,Cost);

    colorvector[0] = 1.0;  // red
    colorvector[3] = 0.5;  // transparency

    g3d_set_color(Any, colorvector);
    g3d_draw_solid_sphere(
        current_WSPoint[0], current_WSPoint[1], current_WSPoint[2], 0.10, 30);

    //    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    //    drawGauge(0, current_cost.first/cost_max);

    //    drawGauge(1, current_cost.second[0]);
    //    drawGauge(2, current_cost.second[1]);
    //    drawGauge(3, current_cost.second[2]);

    //            confPtr_t q_rob =
    //            dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getRobot()->getCurrentPos();
    //            confPtr_t q_hum =
    //            dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getHuman()->getCurrentPos();

    //    glLineWidth(3.);
    //    g3d_drawOneLine((*q_hum)[indexFirstDof + 0],   (*q_hum)[indexFirstDof
    //    + 1],    current_WSPoint[2],
    //                    (*q_rob)[indexFirstDof + 0],   (*q_rob)[indexFirstDof
    //                    + 1],    current_WSPoint[2],
    //                    Yellow, NULL);
    //    glLineWidth(1.);
  }
  OTPListSize = OTPList.size();
  if (OTPListSize > 0) {
    confPtr_t q_hum = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)
                          ->getHuman()
                          ->getCurrentPos();
    int indexFirstDof = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)
                            ->getHuman()
                            ->getJoint("Pelvis")
                            ->getIndexOfFirstDof();
    for (unsigned int i = 0; i < OTPList.size(); i++) {
      double colorvector[4];
      colorvector[0] = 0.0;  // red
      colorvector[1] = 1.0;  // green
      colorvector[2] = 0.0;  // blue
      colorvector[3] = 0.9;  // transparency

      glEnable(GL_BLEND);
      g3d_set_color(Any, colorvector);
      double pDist =
          std::sqrt(pow(OTPList.at(i)[0], 2) + pow(OTPList.at(i)[1], 2));
      double ang = atan2(OTPList.at(i)[1], OTPList.at(i)[0]);
      g3d_draw_solid_sphere(cos(ang + (*q_hum)[indexFirstDof + 5]) * pDist +
                                (*q_hum)[indexFirstDof + 0],
                            sin(ang + (*q_hum)[indexFirstDof + 5]) * pDist +
                                (*q_hum)[indexFirstDof + 1],
                            OTPList.at(i)[2] + (*q_hum)[indexFirstDof + 2],
                            0.02,
                            10);
      glDisable(GL_BLEND);
    }
  }

  if (PlanEnv->getBool(PlanParam::env_drawSlice)) {
    drawSlice(opengl_context);
  }

  if (PlanEnv->getBool(PlanParam::env_drawHumanModel)) {
    // std::vector<double> humanPos =
    // dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getHumanPos();

    double height = 2;

    double colorvector[4];
    colorvector[0] = 0.0;  // red
    colorvector[1] = 1.0;  // green
    colorvector[2] = 0.0;  // blue
    colorvector[3] = 0.9;  // transparency

    //        glEnable(GL_BLEND);
    g3d_set_color(Any, colorvector);
    double fx = PlanEnv->getDouble(PlanParam::env_futurX);
    double fy = PlanEnv->getDouble(PlanParam::env_futurY);
    double fRz = PlanEnv->getDouble(PlanParam::env_futurRZ);

    glLineWidth(3.);
    g3d_drawOneLine(fx, fy, 0, fx, fy, height, Black, NULL);
    glLineWidth(1.);

    g3d_draw_solid_sphere(fx, fy, height, 0.1, 20);

    p3d_vector3 origin, end;
    end[0] = fx + cos(fRz) * 0.2;
    end[1] = fy + sin(fRz) * 0.2;
    end[2] = height;

    origin[0] = fx;
    origin[1] = fy;
    origin[2] = height;

    g3d_draw_arrow(origin, end, 1, 0, 0);
  }

  //    bool draw = PlanEnv->getBool(PlanParam::env_drawFinalConf);
  //    bool isFinal = PlanEnv->getBool(PlanParam::env_isFinalConf);
  //    if ( draw  )
  //    {
  //        glEnable(GL_BLEND);
  //        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //
  //        G3D_Window *win;
  //        win = g3d_get_cur_win();
  //        Robot* rob = global_Project->getActiveScene()->getActiveRobot();
  //        confPtr_t q_cur = rob->getCurrentPos();
  //
  //// p3d_set_and_update_robot_conf(rob->getP3dRobotStruct()->ROBOT_POS);
  //        /* collision checking */
  //        win->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;
  //        //g3d_draw_robot(robotPt->num, win);
  //        p3d_set_and_update_robot_conf(rob->getP3dRobotStruct()->ROBOT_GOTO);
  //        /* collision checking */
  //        win->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;
  //        g3d_draw_robot(rob->getP3dRobotStruct()->num, win, opengl_context);
  //        win->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;
  //
  //        p3d_set_and_update_robot_conf(q_cur->getConfigStruct());
  ////        g3d_draw_trace_all_tcur();
  //    }
}
//#endif

void computeConfigCostOnTraj(p3d_rob* rob, configPt q) {
  if (rob == NULL) {
    return;
  }

  //    Robot* robot =  global_Project->getActiveScene()->getRobotByName(
  //                rob->name);

  if (ENV.getBool(Env::isCostSpace)) {
#ifdef HRI_COSTSPACE

    // configPt cost_q = q;

    if (ENV.getBool(Env::enableHri)) {
      p3d_rob* costRobot = rob;

      std::string robotName(costRobot->name);

      // Does not contain Robot
      if (robotName.find(global_ActiveRobotName) == std::string::npos) {
        costRobot =
            p3d_get_robot_by_name_containing(global_ActiveRobotName.c_str());
        // cost_q = p3d_get_robot_config(costRobot);
      }

      //            robot = global_Project->getActiveScene()->getRobotByName(
      //                        costRobot->name);
    }
#endif

    //        Configuration q( robot, cost_q );
    //        std::cout << "Cost for " << robot->getName() << " = " <<
    //        global_costSpace->cost(q) << std::endl;

    // std::cout << "Cost for " << r_Cost->getName() << " = " <<
    // HRICS_getPredictionOccupancyCost(q_Cost) << endl;
  }

  if (global_collisionSpace) {
    double dist = numeric_limits<double>::max();
    double potential = numeric_limits<double>::max();

    int ncol = global_collisionSpace->isRobotColliding(dist, potential);

    Robot* robCollSapce = global_collisionSpace->getRobot();

    if (ncol) {
      double colorvector[4];

      GroundColorMixGreenToRed(colorvector, 1.0);

      g3d_set_custom_color_draw(
          static_cast<p3d_rob*>(robCollSapce->getP3dRobotStruct()), true);
      g3d_set_custom_color_vect(colorvector);
    } else {
      g3d_set_custom_color_draw(
          static_cast<p3d_rob*>(robCollSapce->getP3dRobotStruct()), false);
    }
  }
}

#ifdef HRI_COSTSPACE
void drawSlice(int opengl_context) {
  Robot* rob =
      global_Project->getActiveScene()->getRobotByNameContaining("PR2");
  Robot* human = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getHuman();
  confPtr_t q_cur(human->getCurrentPos());
  //    getConfListSize() {return m_configList.size();}
  //            configPt getRobotConfigAt

  if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)) {
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getConfList();
  }

  G3D_Window* win;
  win = g3d_get_cur_win();
  win->vs.transparency_mode = G3D_TRANSPARENT_AND_OPAQUE;
  win->vs.transparency_mode = G3D_OPAQUE;

  for (int i = 0; i < dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)
                          ->getConfListSize();
       i++) {
    //        confPtr_t q( new
    //        Configuration(rob,dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getRobotConfigAt(i)
    //        ));
    //        rob->setAndUpdate(*q);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)
        ->setRobotsToConf(i, PlanEnv->getBool(PlanParam::env_isStanding));
    global_Project->getActiveScene()->setActiveRobot("PR2_ROBOT");
    // cout << "g3d_draw_robot" << endl;
    // q->print();
    g3d_draw_robot(rob->getP3dRobotStruct()->num, win, opengl_context);

    //        confPtr_t q( new
    //        Configuration(human,dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getRobotConfigAt(i)
    //        ));
    //        human->setAndUpdate(*q);
    //        global_Project->getActiveScene()->setActiveRobot("ACHILE_HUMAN1");
    //        //cout << "g3d_draw_robot" << endl;
    //        //q->print();
    //        g3d_draw_robot(human->getP3dRobotStruct()->num, win);
  }
  human->setAndUpdate(*q_cur);
}
#endif
