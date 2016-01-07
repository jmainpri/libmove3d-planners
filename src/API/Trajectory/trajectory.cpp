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

#include "API/Trajectory/trajectory.hpp"
#include "API/Graphic/drawModule.hpp"
#include "API/libmove3d_simple_api.hpp"

#include "planEnvironment.hpp"
#include "utils/misc_functions.hpp"

#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Localpath-pkg.h>
#include <libmove3d/include/GroundHeight-pkg.h>
#include <libmove3d/include/Planner-pkg.h>

#include "cost_space.hpp"

#if defined(HRI_COSTSPACE) && defined(HRI_PLANNER)
#include "hri_costspace/HRICS_hamp.hpp"
#endif

#include "move3d-headless.h"

#include <ctime>
#include <sys/time.h>

using namespace std;
using namespace Eigen;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

std::vector<Trajectory> global_trajToDraw;

Trajectory::Trajectory()
    : m_Robot(NULL),
      m_HighestCostId(0),
      m_isHighestCostIdSet(false),
      m_name(""),
      m_id(std::time(0)),
      m_file(""),
      m_use_continuous_color(false),
      m_Color(0),
      m_Source(new Configuration(m_Robot, NULL)),
      m_use_time_parameter(false),
      m_use_constant_dt(true),
      m_dt(0.),
      m_dts(std::vector<double>()) {
  m_Courbe.clear();
}

Trajectory::Trajectory(Robot* R)
    : m_Robot(R),
      m_HighestCostId(0),
      m_isHighestCostIdSet(false),
      m_name(""),
      m_id(std::time(0)),
      m_file(""),
      m_use_continuous_color(false),
      m_Color(0),
      m_Source(new Configuration(m_Robot, NULL)),
      m_use_time_parameter(false),
      m_use_constant_dt(true),
      m_dt(0.),
      m_dts(std::vector<double>()) {
  m_Courbe.clear();
}

Trajectory::Trajectory(const Trajectory& T)
    : m_Robot(T.m_Robot),
      m_HighestCostId(T.m_HighestCostId),
      m_isHighestCostIdSet(T.m_isHighestCostIdSet),
      m_name(T.m_name),
      m_id(std::time(0)),
      m_file(T.m_file),
      m_use_continuous_color(T.m_use_continuous_color),
      m_Color(T.m_Color),
      m_Source(T.m_Source),
      m_Target(T.m_Target),
      m_use_time_parameter(T.m_use_time_parameter),
      m_use_constant_dt(T.m_use_constant_dt),
      m_dt(T.m_dt),
      m_dts(T.m_dts) {
  for (size_t i = 0; i < T.m_Courbe.size(); i++) {
    m_Courbe.push_back(new LocalPath(*(T.m_Courbe[i])));
  }
  //	cout << "Copy Trajectory" << endl;
}

Trajectory& Trajectory::operator=(const Trajectory& T) {
  if (!m_Courbe.empty())  // Delete courbe if not empty
  {
    for (size_t i = 0; i < m_Courbe.size(); i++) {
      delete m_Courbe[i];
    }
    m_Courbe.clear();
  }

  m_Robot = T.m_Robot;

  for (size_t i = 0; i < T.m_Courbe.size(); i++) {
    m_Courbe.push_back(new LocalPath(*(T.m_Courbe[i])));
  }

  // TODO m_name of m_file and robot
  m_name = T.m_name;
  m_id = std::time(0);
  m_file = T.m_file;
  m_Source = T.m_Source;
  m_Target = T.m_Target;
  m_use_continuous_color = T.m_use_continuous_color;
  m_Color = T.m_Color;
  m_HighestCostId = T.m_HighestCostId;
  m_isHighestCostIdSet = T.m_isHighestCostIdSet;
  m_use_time_parameter = T.m_use_time_parameter;
  m_use_constant_dt = T.m_use_constant_dt;
  m_dt = T.m_dt;
  m_dts = T.m_dts;

  return *this;
}

Trajectory::Trajectory(std::vector<confPtr_t>& configs)
    : m_HighestCostId(0),
      m_isHighestCostIdSet(false),
      m_use_continuous_color(false),
      m_use_time_parameter(false),
      m_use_constant_dt(true),
      m_dt(0.),
      m_dts(std::vector<double>()) {
  if (!configs.empty()) {
    m_name = "";
    m_id = std::time(0);
    m_file = "";

    m_Robot = configs[0]->getRobot();

    m_Source = configs[0];
    m_Target = configs.back();

    m_Courbe.clear();

    for (unsigned int i = 0; i < configs.size() - 1; i++) {
      if (!configs[i]->equal(*configs[i + 1])) {
        LocalPath* path = new LocalPath(configs[i], configs[i + 1]);
        m_Courbe.push_back(path);
      } else {
        cout << "two configuration are the same in traj constructor" << endl;
      }
    }
  } else {
    cout << "Warning: Constructing a class out of empty vector of configuration"
         << endl;
  }
  m_Color = 1;
}

Trajectory::Trajectory(Robot* R, p3d_traj* t)
    : m_use_time_parameter(false),
      m_use_constant_dt(true),
      m_dt(0.),
      m_dts(std::vector<double>()) {
  m_Courbe.clear();

  if ((t == NULL) || (t->courbePt == NULL)) {
    return;
  }

  // TODO Name and m_file (string based)
  m_Robot = R;
  m_id = std::time(0);

  p3d_localpath* localpathPt = t->courbePt;

  while (localpathPt != NULL) {
    LocalPath* path = new LocalPath(m_Robot, localpathPt);
    // path->getBegin()->print();
    // path->getEnd()->print();
    m_Courbe.push_back(path);
    localpathPt = localpathPt->next_lp;
  }

  m_Source = confPtr_t(
      new Configuration(m_Robot, p3d_config_at_param_along_traj(t, 0)));
  m_Source->setConstraints();

  //	cout << "m_Source:" << endl;
  //	m_Source->print();

  m_Target = confPtr_t(new Configuration(
      m_Robot, p3d_config_at_param_along_traj(t, getParamMax())));
  m_Target->setConstraints();

  //	cout << "m_Target:" << endl;
  //	m_Target->print();

  //	cout << "range_param = " << range_param << endl;
  m_use_continuous_color = false;
  m_Color = 0;

  if (!getBegin()->equal(*configAtParam(0.0))) {
    cout << "Error in constructor : !getBegin()->equal(*configAtParam(0))"
         << endl;
    getBegin()->print();
    configAtParam(0.)->print();
  }

  if (!getEnd()->equal(*configAtParam(getParamMax()))) {
    cout << "------------------------------------------" << endl;
    cout << "Error in constructor : "
            "!getEnd()->equal(*configAtParam(getParamMax()))" << endl;
    getEnd()->print();
    configAtParam(getParamMax())->print();
  }
}

bool Trajectory::operator==(const Trajectory& t) const {
  if (m_Courbe.size() != t.m_Courbe.size()) {
    return false;
  }

  if (getParamMax() != t.getParamMax()) {
    return false;
  }

  for (int i = 0; i < int(m_Courbe.size()); i++) {
    if ((*m_Courbe[i]->getBegin()) != (*t.m_Courbe[i]->getBegin())) {
      return false;
    }
    if ((*m_Courbe[i]->getEnd()) != (*t.m_Courbe[i]->getEnd())) {
      return false;
    }
    if (m_Courbe[i]->getParamMax() != t.m_Courbe[i]->getParamMax()) {
      return false;
    }
  }

  return true;
}

bool Trajectory::operator!=(const Trajectory& t) const { return !(*this == t); }

Trajectory::~Trajectory() {
  for (int i = 0; i < int(m_Courbe.size()); i++) {
    delete m_Courbe.at(i);
  }
}

bool Trajectory::replaceP3dTraj() const {
  // cout << "Robot name : " << m_Robot->getP3dRobotStruct()->name << endl;
  // return replaceP3dTraj( p3d_get_robot_by_name( m_Robot->getName().c_str()
  // )->tcur );

  if (m_Robot->getUseLibmove3dStruct()) {
    return replaceP3dTraj(
        static_cast<p3d_rob*>(m_Robot->getP3dRobotStruct())->tcur);
  } else {
    return false;
  }
}

p3d_traj* Trajectory::replaceP3dTraj(p3d_traj* trajPt) const {
  if (!m_Robot->getUseLibmove3dStruct()) {
    return NULL;
  }

  //	print();

  p3d_rob* robotPt = (p3d_rob*)m_Robot->getP3dRobotStruct();

  if (trajPt != NULL) {
    if (strcmp(trajPt->rob->name, robotPt->name) != 0) {
      cout << " Warning : Robot not the same as the robot in traj " << endl;
      return NULL;
    }
    if (trajPt->courbePt != NULL) {
      destroy_list_localpath(robotPt, trajPt->courbePt);
    }
  } else {
    trajPt = p3d_create_empty_trajectory(robotPt);
  }

  // trajPt->name = strdup(name);
  // trajPt->file = NULL;  // Modification Fabien
  trajPt->num = 0;  // m_Robot->getP3dRobotStruct()->nt;
  // trajPt->rob = m_Robot->getP3dRobotStruct();

  p3d_localpath* localpathPt = NULL;
  p3d_localpath* localprevPt = NULL;

  bool first = true;

  for (int i = 0; i < int(m_Courbe.size()); i++) {
    localprevPt = localpathPt;
    localpathPt = m_Courbe[i]->getP3dLocalpathStruct()->copy(
        (p3d_rob*)m_Robot->getP3dRobotStruct(),
        m_Courbe[i]->getP3dLocalpathStruct());

    if (localprevPt) {
      localprevPt->next_lp = localpathPt;
    }

    localpathPt->prev_lp = localprevPt;

    if (first) {
      trajPt->courbePt = localpathPt;
      first = false;
    }
  }

  trajPt->range_param = p3d_compute_traj_rangeparam(trajPt);
  trajPt->nlp = m_Courbe.size();

  if (m_Courbe.size() != 0)
    localpathPt->next_lp = NULL;
  else {
    cout << "replaceP3dTraj with empty trajectory" << endl;
    trajPt->courbePt = NULL;
  }

  return trajPt;
  //	print()
}

void Trajectory::copyPaths(std::vector<LocalPath*>& vect) {
  vector<LocalPath*>::iterator it;

  for (it = vect.begin(); it != vect.end(); it++) {
    *it = new LocalPath(**it);
  }
}

p3d_traj* Trajectory::replaceHumanP3dTraj(Robot* rob, p3d_traj* trajPt) {
  //	print();

  //	Robot* rob =new Robot(p3d_get_robot_by_name(trajPt->rob->name));
  if (trajPt != NULL) {
    destroy_list_localpath((p3d_rob*)rob->getP3dRobotStruct(),
                           trajPt->courbePt);
  } else {
    trajPt = p3d_create_empty_trajectory((p3d_rob*)rob->getP3dRobotStruct());
  }

  //	trajPt->name       = strdup(name);
  //	trajPt->file       = NULL;  // Modification Fabien
  trajPt->num = 0;  // rob->getP3dRobotStruct()->nt;
  //    trajPt->rob = m_Robot->getP3dRobotStruct();

  //	cout << rob->getP3dRobotStruct() << endl;

  p3d_localpath* localpathPt = NULL;
  p3d_localpath* localprevPt = NULL;

  bool first = true;

  for (unsigned int i = 0; i < m_Courbe.size(); i++) {
    //    if ( *m_Courbe[i]->getBegin() ==  *m_Courbe[i]->getEnd() )
    //    {
    //      cout << "null LocalPath in replaceP3dTraj" << endl;
    //      continue;
    //    }

    localprevPt = localpathPt;
    localpathPt = m_Courbe[i]->getP3dLocalpathStruct()->copy(
        (p3d_rob*)rob->getP3dRobotStruct(),
        m_Courbe[i]->getP3dLocalpathStruct());

    if (localprevPt) {
      localprevPt->next_lp = localpathPt;
    }

    localpathPt->prev_lp = localprevPt;

    if (first) {
      trajPt->courbePt = localpathPt;
      first = false;
    }
  }

  if (m_Courbe.size() != 0) {
    localpathPt->next_lp = NULL;
  } else {
    cout << "replaceP3dTraj with empty trajectory" << endl;
  }

  trajPt->nlp = m_Courbe.size();

#ifdef P3D_PLANNER
  trajPt->range_param = p3d_compute_traj_rangeparam(trajPt);
#else
  printf("P3D_PLANNER not compiled in %s in %s", __PRETTY_FUNCTION__, __FILE__);
#endif

  return trajPt;
  //	print()
}

double Trajectory::getDuration() const {
  if (m_use_time_parameter) {
    if (m_use_constant_dt) {
      return m_dt * double(m_Courbe.size());
    } else {
      double t = 0.0;
      for (int i = 0; i < int(m_dts.size()); i++) t += m_dts[i];
      return t;
    }
  }
  return 0.0;
}

confPtr_t Trajectory::configAtTime(double time,
                                   unsigned int* id_localpath) const {
  if (m_Courbe.empty() || !m_use_time_parameter)
    return confPtr_t(new Configuration(m_Robot, NULL));

  if (time >= getDuration()) return m_Target;

  double alpha_local = 0.0;
  unsigned int path_id = 0;

  if (m_use_constant_dt) {
    double alpha_traj = time / m_dt;
    path_id = alpha_traj;
    alpha_local = alpha_traj - double(path_id);
  } else {
    // the dt refers to the time at which the confuration is along the traj
    // in the constant dt it is simply the length of the local path

    double t0 = 0.0;
    path_id = 0;
    for (; path_id < m_dts.size(); path_id++) {
      t0 += m_dts[path_id];
      if (time < t0) {
        t0 -= m_dts[path_id];  // time on traj, considers that the initial
                               // configuration is set at t=0.0 sec
        break;
      }
    }

    alpha_local = (time - t0) / m_dts[path_id];

    if (path_id == 0) {
      cout << " m_dts[0] : " << m_dts[path_id] << endl;
      exit(0);
    }
    path_id--;

    //        cout << "alpha_local : "  << alpha_local << endl;
    //        cout << "path_id : "  << path_id << endl;
    //        cout << "m_dts[path_id] : "  << m_dts[path_id] << endl;

    if (path_id >= m_Courbe.size()) cout << "return m_Target" << endl;
  }

  if (id_localpath != NULL) *id_localpath = path_id;
  if (alpha_local > 1.0) alpha_local = 1.0;
  if (path_id >= m_Courbe.size()) return m_Target;

  return m_Courbe[path_id]->configAtParam(alpha_local *
                                          m_Courbe[path_id]->getParamMax());
}

confPtr_t Trajectory::configAtParam(double param,
                                    unsigned int* id_localpath) const {
  if (m_Courbe.empty()) return confPtr_t(new Configuration(m_Robot, NULL));

  double soFar(0.0);
  double prevSoFar(0.0);

  for (size_t i = 0; i < m_Courbe.size(); i++) {
    soFar = soFar + m_Courbe[i]->getParamMax();

    // Parameter lies in the previous LP return configuration inside previous LP
    if (param < soFar) {
      if (param < prevSoFar)
        cout << "Error: getting Configuration at parameter in trajectory"
             << endl;

      if (id_localpath != NULL) *id_localpath = i;

      return m_Courbe[i]->configAtParam(param - prevSoFar);
    }
    prevSoFar = soFar;
  }

  if (id_localpath != NULL) *id_localpath = m_Courbe.size() - 1;

  return m_Courbe.back()->configAtParam(param);
}

vector<confPtr_t> Trajectory::getVectorOfConfiguration() const {
  vector<confPtr_t> vect;

  if (m_Courbe.empty()) {
    return vect;
  }

  vect.push_back(m_Courbe[0]->getBegin());

  for (size_t i = 0; i < m_Courbe.size(); i++)
    vect.push_back(m_Courbe[i]->getEnd());

  return vect;
}

vector<confPtr_t> Trajectory::getNConfAtParam(double delta) const {
  vector<confPtr_t> tmpVector(0);

  double param = 0;
  double soFar(0.0);
  double prevSoFar(0.0);

  for (size_t i = 0; i < m_Courbe.size(); i++) {
    soFar = soFar + m_Courbe[i]->getParamMax();

    // Parameter lies in the previous local path
    // return configuration inside previous LP
    while (param < soFar) {
      if (param < prevSoFar) {
        cout << "Error: getting Configuration at parameter in trajectory"
             << endl;
      }

      tmpVector.push_back(m_Courbe.at(i)->configAtParam(param - prevSoFar));
      param += delta;
    }
    // TODO watch out
    if (i < m_Courbe.size() - 1) {
      prevSoFar = soFar;
    }
  }

  tmpVector.push_back(
      m_Courbe.at(m_Courbe.size() - 1)->configAtParam(param - prevSoFar));

  return tmpVector;
}

bool Trajectory::isEmpty() { return m_Courbe.empty(); }

void Trajectory::clear() {
  m_Source = confPtr_t(new Configuration(m_Robot, NULL));

  for (int i = 0; i < int(m_Courbe.size()); i++) {
    delete m_Courbe.at(i);
  }

  m_Courbe.clear();
}

LocalPath* Trajectory::getLocalPath(unsigned int id) const {
  return m_Courbe[id];
}

int Trajectory::getNbOfViaPoints() const {
  if (m_Courbe.empty()) return 0;

  return (m_Courbe.size() + 1);
}

confPtr_t Trajectory::operator[](const int& i) const {
  if (i < 0 || m_Courbe.empty() || (i > int(m_Courbe.size()))) {
    return confPtr_t(new Configuration(m_Robot));
  }

  if (i == int(m_Courbe.size())) {
    return m_Courbe[i - 1]->getEnd();
  }

  return m_Courbe[i]->getBegin();
}

double Trajectory::computeSubPortionRange(
    const vector<LocalPath*>& portion) const {
  double range(0.0);

  for (int i = 0; i < int(portion.size()); i++) {
    // portion[i]->getBegin()->print();
    // portion[i]->getEnd()->print();

    range += portion[i]->getParamMax();
  }

  return range;
}

// void Trajectory::updateRange()
//{
//	nloc = m_Courbe.size();
//	range_param = computeSubPortionRange(m_Courbe);
//}

bool Trajectory::isValid() const {
  for (size_t i = 0; i < m_Courbe.size(); i++) {
    if (!m_Courbe[i]->isValid()) {
      //            cout <<"LocalPath["<<i<<"] = "<< m_Courbe[i]->getNbColTest()
      //            << ", size : " << m_Courbe.size() << endl;
      return false;
    }
  }

  return true;
}

void Trajectory::resetIsValid() {
  for (int i = 0; i < int(m_Courbe.size()); i++) {
    m_Courbe[i]->setLocalpathAsNotTested();
  }
}

vector<pair<double, double> > Trajectory::getCostProfile() {
  vector<pair<double, double> > vectOfCost;

  double previousPathParam = 0.0;

  for (unsigned int i = 0; i < m_Courbe.size(); i++) {
    vector<pair<double, double> > tmp = m_Courbe[i]->getCostProfile();

    for (unsigned int j = 0; j < tmp.size(); j++) {
      tmp[j].first += previousPathParam;
      vectOfCost.push_back(tmp[j]);
    }

    previousPathParam += m_Courbe[i]->getParamMax();

    //		vectOfCost.insert( vectOfCost.end() ,
    //											tmp.begin(),
    //											tmp.end());
  }

  return vectOfCost;
}

std::pair<double, double> Trajectory::computeSubPortionMinAndMaxCost(
    vector<LocalPath*>& portion) {
  double maxCost(0.0);
  double minCost(std::numeric_limits<double>::max());

  for (int i = 0; i < int(portion.size()); i++) {
    double cost = portion[i]->cost();

    if (cost > maxCost) maxCost = cost;
    if (cost < minCost) minCost = cost;
  }
  return std::make_pair(minCost, maxCost);
}

double Trajectory::computeSubPortionCost(
    const vector<LocalPath*>& portion) const {
  double sumCost(0.0);

  //    cout << " cost : " ;

  for (int i = 0; i < int(portion.size()); i++) {
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

double Trajectory::reComputeSubPortionCost(vector<LocalPath*>& portion,
                                           int& nb_cost_tests) {
  double sumCost(0.0);

  nb_cost_tests = 0;

  for (int i = 0; i < int(portion.size()); i++) {
    portion[i]->resetCostComputed();
    double cost = portion[i]->cost();

    nb_cost_tests += portion[i]->getNbCostTest();
    //    cout << "cost[" << i << "] = " << cost << endl;
    //    cout << "resolution[" << i << "] = " << portion[i]->getResolution() ;
    //    cout << " , length["  << i << "] = " << portion[i]->getParamMax() ;
    //		cout << ", cost[" << i << "] = " << cost << endl;
    sumCost += cost;
  }

  return sumCost;
}

double Trajectory::computeSubPortionIntergralCost(
    const vector<LocalPath*>& portion) {
  cout << __PRETTY_FUNCTION__ << endl;
  double cost(0.0);
  double step =
      ENV.getDouble(Env::dmax) * PlanEnv->getDouble(PlanParam::costResolution);
  double currentParam(0.0), currentCost, prevCost;
  double range = computeSubPortionRange(portion);
  int n_step = int(range / step);

  confPtr_t q = configAtParam(0.0);
  prevCost = q->cost();

  cout << "--------- Integral ----------------" << endl;
  cout << "Range = " << range << endl;
  cout << "step = " << step << endl;
  cout << "n_step = " << n_step << endl;

  global_costSpace->setDeltaStepMethod(cs_integral);

  for (int i = 0; i < n_step; i++) {
    currentParam += step;

    q = configAtParam(currentParam);
    currentCost = q->cost();

    double delta_cost =
        global_costSpace->deltaStepCost(prevCost, currentCost, step);

    cost += delta_cost;
    prevCost = currentCost;
  }

  cout << "cost (1) : " << cost << endl;

  cost = 0.0;

  if (!portion.empty())  // Approximation if points are dense enough
  {
    confPtr_t q_prev = portion[0]->getBegin();

    for (size_t i = 0; i < portion.size(); i++) {
      confPtr_t q = portion[i]->getEnd();
      cost += q->cost() * q->dist(*q_prev);
      q_prev = q;
    }

    cout << "cost (2) : " << cost << endl;
  }

  return cost;
}

double Trajectory::computeSubPortionCostVisib(vector<LocalPath*>& portion) {
  double epsilon = 0.002;
  double cost(0.0);
  double dmax = 0;

  bool inVisibilty(false);

  for (unsigned int i = 0; i < m_Courbe.size(); i++) {
    double resol = m_Courbe[i]->getResolution();
    dmax += resol;
  }

  dmax /= (double)m_Courbe.size();

  double currentParam(0.0);
  double prevCost;
  double currentCost = portion[0]->getBegin()->cost();
  double range = computeSubPortionRange(portion);

  int jnt_id = 0;

#if defined(HRI_COSTSPACE) && defined(HRI_PLANNER)
  // jnt_id = hriSpace->getTask();
  cout << "Be carefull not defined" << endl;
#else
  cout << "Error : HRI Planner not compiled nor linked" << endl;
  return 0;
#endif

  m_Robot->setAndUpdate(*m_Source);
  Vector3d prevPos;
  Vector3d currentPos = m_Robot->getJoint(jnt_id)->getVectorPos();

  while (currentParam <= range) {
    currentParam += dmax;

    confPtr_t currentConf = configAtParam(currentParam);

    prevCost = currentCost;
    prevPos = currentPos;

    m_Robot->setAndUpdate(*currentConf);
    currentPos = m_Robot->getJoint(jnt_id)->getVectorPos();
    double distStep = 0;
    for (int k = 0; k < currentPos.size(); k++) {
      distStep += pow((currentPos[k] - prevPos[k]), 2);
    }
    distStep = sqrt(distStep);

    double step_cost;

    if (!inVisibilty) {
      currentCost = currentConf->cost();
      // the cost associated to a small portion of curve

      step_cost =
          global_costSpace->deltaStepCost(prevCost, currentCost, distStep);
      //			p3d_ComputeDeltaStepCost(prevCost, currentCost,
      //distStep);

      cout << " Step Cost = " << step_cost << endl;

      if (currentCost < ENV.getDouble(Env::visThresh)) {
        inVisibilty = true;
      }
    } else {
      step_cost = epsilon * distStep;
    }

    cost += step_cost;
  }

  return cost;
}

double Trajectory::cost() const {
  if (!ENV.getBool(Env::isCostSpace)) return isValid() ? 0. : 1000.;

  double cost(0.0);
  cost = computeSubPortionCost(m_Courbe);

  //    cout << "cost  : " << cost << endl;

  // cost =  computeSubPortionIntergralCost(m_Courbe);
  // cost = reComputeSubPortionCost(m_Courbe,nb_cost_tests);
  return cost;
}

double Trajectory::costRecomputed() {
  if (!ENV.getBool(Env::isCostSpace)) return isValid() ? 0. : 1000.;

  int nb_test = 0;
  return reComputeSubPortionCost(m_Courbe, nb_test);
}

double Trajectory::costNoRecompute() {
  if (!ENV.getBool(Env::isCostSpace)) return isValid() ? 0. : 1000.;

  return computeSubPortionCost(m_Courbe);
}

double Trajectory::costStatistics(TrajectoryStatistics& stat) {
  int nb_cost_tests = 0;
  int total_cost_tests = 0;

  stat.length = getParamMax();

  if (global_costSpace != NULL) {
    stat.is_valid = isValid();

    CostSpaceDeltaStepMethod method = global_costSpace->getDeltaStepMethod();

    stat.sum = costNPoints(100);

    global_costSpace->setDeltaStepMethod(cs_max);
    reComputeSubPortionCost(m_Courbe, nb_cost_tests);
    total_cost_tests += nb_cost_tests;

    std::pair<double, double> min_max =
        computeSubPortionMinAndMaxCost(m_Courbe);
    stat.min = min_max.first;
    stat.max = min_max.second;

    global_costSpace->setDeltaStepMethod(cs_average);
    stat.average = reComputeSubPortionCost(m_Courbe, nb_cost_tests);

    global_costSpace->setDeltaStepMethod(cs_integral);
    stat.integral = reComputeSubPortionCost(m_Courbe, nb_cost_tests);

    global_costSpace->setDeltaStepMethod(cs_mechanical_work);
    stat.mecha_work = reComputeSubPortionCost(m_Courbe, nb_cost_tests);

    global_costSpace->setDeltaStepMethod(method);

    resetCostComputed();
  } else {
    stat.min = 0.0;
    stat.max = 0.0;
    stat.average = 0.0;
    stat.integral = 0.0;
    stat.mecha_work = 0.0;
  }

  return stat.integral;
}

double Trajectory::costDeltaAlongTraj() {
  if (!ENV.getBool(Env::isCostSpace)) return isValid() ? 0. : 1000.;

  cout << " m_Courbe.size() : " << m_Courbe.size() << endl;

  cout << "Sum of LP cost = " << computeSubPortionCost(m_Courbe) << endl;
  Trajectory tmp(*this);
  if (tmp != (*this)) {
    cout << "Trajectory not the same" << endl;
  }
  int nb_cost_tests = 0;
  cout << "Sum of LP cost (Recomputed) = "
       << reComputeSubPortionCost(tmp.m_Courbe, nb_cost_tests) << endl;
  double cost = computeSubPortionIntergralCost(m_Courbe);
  cout << "Intergral along traj = " << cost << endl;
  return cost;
}

double Trajectory::costNPoints(const int n_points) {
  double s = 0.0;
  double cost = 0.0;
  double delta = getParamMax() / double(n_points - 1);

  for (int i = 0; i < n_points; i++) {
    cost += configAtParam(s)->cost();
    // cout << "delta_cost["<<i<<"] = " << configAtParam(s)->cost() << endl;
    s += delta;
  }
  // cout << "cost : " << cost << endl;
  return cost;
}

double Trajectory::costSum() {
  double cost = 0.0;
  int i = 0;

  for (i = 0; i < int(m_Courbe.size()); i++) {
    cost += m_Courbe[i]->getBegin()->cost();
    // cout << "delta_cost["<<i<<"] = " << m_Courbe[i]->getBegin()->cost() <<
    // endl;
  }
  cost += m_Courbe[i - 1]->getEnd()->cost();
  // cout << "delta_cost["<<i-1<<"] = " << m_Courbe[i-1]->getEnd()->cost() <<
  // endl;
  return cost;
}

double Trajectory::getDeltaTime(int i) {
  double delta = 0;

  if (m_use_time_parameter) {
    if (m_use_constant_dt)

      delta *= m_dt;
    else
      delta *= m_dts[i];  // Warning not sure ...
  } else {
    if (i <= int(m_Courbe.size())) {
      if (i == 0 || i == int(m_Courbe.size())) {
        int id = (i == int(m_Courbe.size())) ? i - 1 : i;
        confPtr_t q_1 = m_Courbe[id]->getBegin();
        confPtr_t q_2 = m_Courbe[id]->getEnd();
        delta = q_1->dist(*q_2);
      } else {
        confPtr_t q_1 = m_Courbe[i - 1]->getBegin();
        confPtr_t q_2 = m_Courbe[i]->getBegin();
        delta = q_1->dist(*q_2);
      }
    }
  }

  return delta;
}

double Trajectory::costPerPoint() {
  double cost = 0.0;
  int i = 0;
  double delta;

  for (i = 0; i < int(m_Courbe.size()); i++) {
    delta = m_Courbe[i]->getBegin()->cost() * getDeltaTime(i);
    cost += delta;
  }

  delta = m_Courbe[i - 1]->getEnd()->cost() * getDeltaTime(i);
  cost += delta;

  return cost;
}

double Trajectory::costOfPortion(double param1, double param2) {
  uint first(0);
  uint last(0);
  vector<LocalPath*> path;

  pair<bool, vector<LocalPath*> > valid_portion =
      extractSubPortion(param1, param2, first, last);

  if (valid_portion.first) {
    path = valid_portion.second;
  } else {
    cout << "Error: inconsistant query in costOfPortion" << endl;
  }

  double Cost = computeSubPortionCost(path);

  for (unsigned int i = 0; i < path.size(); i++) {
    delete path.at(i);
  }
  return Cost;
}
// double Trajectory::costOfPortion(double param1,double param2){
//
//	double soFar(0.0);
//	double prevSoFar(0.0);
//	uint id_start;
//	uint id_end;
//	confPtr_t confPtrStart;
//	confPtr_t confPtrEnd;
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
//			confPtrStart =
//m_Courbe.at(i)->configAtParam(param1-prevSoFar);
//			id_start = i;
//			break;
//		}
//		if(i==nloc-1){
//			confPtrStart =
//m_Courbe.at(i)->configAtParam(param1-prevSoFar);
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
//			confPtrEnd =
//m_Courbe.at(i)->configAtParam(param2-prevSoFar);
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

void Trajectory::resetCostComputed() {
  m_Source->setCostAsNotTested();
  m_Target->setCostAsNotTested();

  for (unsigned int i = 0; i < m_Courbe.size(); i++) {
    m_Courbe[i]->resetCostComputed();
    m_Courbe[i]->getBegin()->setCostAsNotTested();
    m_Courbe[i]->getEnd()->setCostAsNotTested();
  }
}

vector<confPtr_t> Trajectory::getTowConfigurationAtParam(double param1,
                                                         double param2,
                                                         uint& lp1,
                                                         uint& lp2) const {
  vector<confPtr_t> conf;

  if (param1 < 0) {
    cout << "Error: the parameter is out of band" << endl;
  }
  if (param2 > getParamMax()) {
    cout << "Error: the parameter is out of band" << endl;
  }

  if (param1 > param2) {
    cout
        << "Error: not possible to replace trajectory because of the parameters"
        << endl;
    return conf;
  }

  // Looks for the local paths to be changed linear in NLOC
  //-------------------------------------------------------------------
  double soFar(0.0);
  double prevSoFar(0.0);

  confPtr_t q1;
  confPtr_t q2;

  for (uint i = 0; i < m_Courbe.size(); i++) {
    soFar = soFar + m_Courbe.at(i)->getParamMax();

    if (param1 < soFar) {
      q1 = m_Courbe.at(i)->configAtParam(param1 - prevSoFar);
      //			q1->print();
      //			param_start = param1-prevSoFar;
      lp1 = i;
      break;
    }
    prevSoFar = soFar;
    if (i == (m_Courbe.size() - 1)) {
      q1 = m_Courbe.at(i)->getEnd();
      lp1 = i;
      cout << "Error : conf ends local path" << endl;
    }
  }

  // id_LP_1 is the id of the path of which param1 lies in
  // we start searching for parm2 from here
  soFar = prevSoFar;

  for (uint i = lp1; i < m_Courbe.size(); i++) {
    soFar = soFar + m_Courbe.at(i)->getParamMax();

    if (param2 < soFar) {
      q2 = m_Courbe.at(i)->configAtParam(param2 - prevSoFar);
      //			q2->print();
      //			param_end = param2-soFar;
      lp2 = i;
      break;
    }
    prevSoFar = soFar;

    if (i == (m_Courbe.size() - 1)) {
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

double Trajectory::extractCostPortion(double param1, double param2) {
  double totalCost(0.0);

  vector<confPtr_t> conf;

  uint first;
  uint last;

  conf = getTowConfigurationAtParam(param1, param2, first, last);

  confPtr_t q1 = conf.at(0);
  confPtr_t q2 = conf.at(1);

  if (first > last) {
    cout << "Error: extractCostPortion: inconsistent query for subpath "
            "extraction" << endl;
    return 0;
  }

  // Case where they lie in the same path
  //----------------------------------------------------------------------------
  if (first == last) {
    if (q1->equal(*q2)) {
      cout << "Error: extractCostPortion: q1 and q2 are the same in subportion "
              "extraction" << endl;
      return 0;
    }
    LocalPath LP(q1, q2);
    return LP.cost();
  }

  confPtr_t start = m_Courbe.at(first)->getBegin();
  confPtr_t end = m_Courbe.at(last)->getEnd();

  // Adds the modified first local path to subpaths
  // Verifies that the configuration is not starting the local path
  if (!start->equal(*q1)) {
    if (!m_Courbe.at(first)->getEnd()->equal(*q1)) {
      LocalPath LP(q1, m_Courbe.at(first)->getEnd());

      if (LP.isValid()) {
        totalCost += LP.cost();
      } else {
        cout << "Error: extractCostPortion: portion of path not valid" << endl;
        return -1;
      }
    }
  } else {
    if (m_Courbe.at(first)->isValid()) {
      totalCost += m_Courbe.at(first)->cost();
    } else {
      cout << "Error: extractCostPortion: portion of path not valid" << endl;
      return -1;
    }
  }

  // Adds all the paths between First and Last
  for (uint i = first + 1; i < last; i++) {
    totalCost += m_Courbe.at(i)->cost();
  }

  // Verifies that the configuration is not ending the local path
  // Adds the modified Last local path to subpaths
  if (!end->equal(*q2)) {
    if (!m_Courbe.at(last)->getBegin()->equal(*q2)) {
      LocalPath LP(m_Courbe.at(last)->getBegin(), q2);

      if (LP.isValid()) {
        totalCost += LP.cost();
      } else {
        cout << "Error: extractCostPortion: portion of path not valid" << endl;
        return -1;
      }
    }
  } else {
    if (m_Courbe.at(last)->isValid()) {
      totalCost += m_Courbe.at(last)->cost();
    } else {
      cout << "Error: extractCostPortion: portion of path not valid" << endl;
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
   cout << "Error: extractCostPortion: in extract sub portion integrity" <<
   endl;
   }*/

  return totalCost;
}

pair<bool, vector<LocalPath*> > Trajectory::extractSubPortion(
    double param1,
    double param2,
    unsigned int& first,
    unsigned int& last,
    bool check_for_coll) const {
  vector<LocalPath*> paths;
  vector<confPtr_t> conf;

  conf = getTowConfigurationAtParam(param1, param2, first, last);

  confPtr_t q1 = conf.at(0);
  confPtr_t q2 = conf.at(1);

  if (first > last) {
    cout << "Error: inconsistent query for subpath extraction" << endl;
    return make_pair(false, paths);
  }

  // Case where they lie in the same path
  //----------------------------------------------------------------------------
  if (first == last) {
    if (q1->equal(*q2)) {
      paths.resize(0);
      cout << "Error: q1 and q2 are the same in subportion extraction" << endl;
      return make_pair(false, paths);
    }
    paths.push_back(new LocalPath(q1, q2));
    return make_pair(true, paths);
  }

  confPtr_t start = m_Courbe.at(first)->getBegin();
  confPtr_t end = m_Courbe.at(last)->getEnd();

  // Adds the modified first local path to subpaths
  // Verifies that the configuration is not starting the local path
  if (!start->equal(*q1)) {
    if (!m_Courbe.at(first)->getEnd()->equal(*q1)) {
      LocalPath* startNew = new LocalPath(q1, m_Courbe.at(first)->getEnd());

      if ((check_for_coll && startNew->isValid()) || !check_for_coll) {
        paths.push_back(startNew);
      } else {
        cout << "Error: portion of path not valid" << endl;
        return make_pair(false, paths);
      }
    }
  } else {
    LocalPath* startNew = new LocalPath(*m_Courbe.at(first));

    if ((check_for_coll && startNew->isValid()) || !check_for_coll) {
      paths.push_back(startNew);
    } else {
      cout << "Error: portion of path not valid" << endl;
      return make_pair(false, paths);
    }
  }

  // Adds all the paths between First and Last
  for (uint i = first + 1; i < last; i++) {
    paths.push_back(new LocalPath(*m_Courbe.at(i)));
  }

  // Verifies that the configuration is not ending the local path
  // Adds the modified Last local path to subpaths
  if (!end->equal(*q2)) {
    if (!m_Courbe.at(last)->getBegin()->equal(*q2)) {
      LocalPath* endNew = new LocalPath(m_Courbe.at(last)->getBegin(), q2);

      if ((check_for_coll && endNew->isValid()) || !check_for_coll) {
        paths.push_back(endNew);
      } else {
        cout << "Error: portion of path not valid" << endl;
        return make_pair(false, paths);
      }
    }
  } else {
    LocalPath* endNew = new LocalPath(*m_Courbe.at(last));

    if ((check_for_coll && endNew->isValid()) || !check_for_coll) {
      paths.push_back(endNew);
    } else {
      cout << "Error: portion of path not valid" << endl;
      return make_pair(false, paths);
    }
  }

  // Verifies the integrity of the sub paths
  if ((!q1->equal(*paths.at(0)->getBegin())) ||
      (!q2->equal(*paths.back()->getEnd()))) {
    paths.at(0)->getBegin()->print();
    q1->print();
    paths.back()->getEnd()->print();
    q2->print();
    cout << "Error: in extract sub portion integrity" << endl;
  }

  return make_pair(true, paths);
}

//! Extract sub trajectory
//! @param start is the id of the first localpath
//! @param end is the id of the last localpath
Trajectory Trajectory::extractSubTrajectoryOfLocalPaths(
    unsigned int id_start, unsigned int id_end) const {
  vector<LocalPath*> path;

  if (id_start > id_end) {
    cout << "Warning: inconsistant query in extractSubTrajectory" << endl;
  } else {
    for (unsigned int i = id_start; i <= id_end; i++) {
      path.push_back(new LocalPath(*m_Courbe[i]));
    }
  }

  Trajectory newTraj(m_Robot);

  newTraj.m_Courbe = path;

  if (path.size() == 0) {
    newTraj.m_Source = m_Courbe[id_start]->getBegin();
    newTraj.m_Target = m_Courbe[id_end]->getEnd();
  } else {
    newTraj.m_Source = path.at(0)->getBegin();
    newTraj.m_Target = path.back()->getEnd();
  }

  return newTraj;
}

Trajectory Trajectory::extractSubTrajectory(double param1,
                                            double param2,
                                            bool check_for_coll) const {
  unsigned int first(0);
  unsigned int last(0);

  Trajectory newTraj(m_Robot);

  if (param1 > param2) {
    cout << "Warning: inconsistant query in extractSubTrajectory" << endl;
  } else {
    pair<bool, vector<LocalPath*> > valid_portion =
        extractSubPortion(param1, param2, first, last, check_for_coll);

    if (check_for_coll) {
      if (valid_portion.first) {
        newTraj.m_Courbe = valid_portion.second;
      } else {
        cout << "Error: inconsistant query in extractSubTrajectory" << endl;
      }
    } else {
      newTraj.m_Courbe = valid_portion.second;
    }
  }

  if (newTraj.m_Courbe.size() == 0) {
    newTraj.m_Source = configAtParam(param1);
    newTraj.m_Target = newTraj.m_Source;
  } else {
    newTraj.m_Source = newTraj.m_Courbe[0]->getBegin();
    newTraj.m_Target = newTraj.m_Courbe.back()->getEnd();
  }

  return newTraj;
}

Trajectory Trajectory::extractReverseTrajectory() const {
  Trajectory newTraj(m_Robot);
  newTraj.m_Source = m_Target;
  newTraj.m_Target = m_Source;

  for (int i = int(m_Courbe.size() - 1); i >= 0; i--) {
    newTraj.push_back(m_Courbe[i]->getBegin()->copy());
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

void Trajectory::draw(int nbKeyFrame) {
  //    cout << __PRETTY_FUNCTION__ << endl;
  //    cout << this << endl;

  if (nbKeyFrame == 0) {
    nbKeyFrame = 100;
  }
  double du = getParamMax() / (nbKeyFrame - 1);
  if (du == 0.0) return;

  double u = du;

  double Cost1, Cost2;

  Joint* drawnjnt = NULL;

  int indexjnt = p3d_get_user_drawnjnt();
  if (indexjnt != -1 && indexjnt <= int(m_Robot->getNumberOfJoints())) {
    drawnjnt = m_Robot->getJoint(indexjnt);
  }
  if (drawnjnt == NULL) {
    return;
  }

  confPtr_t qSave = m_Robot->getCurrentPos();
  confPtr_t q = m_Source;
  m_Robot->setAndUpdate(*q);

  Eigen::Vector3d pi, pf;
  pi = drawnjnt->getVectorPos();

  int saveColor;
  bool red = false;

  double range_max = getParamMax();

  while (u <= range_max) {
    if (u > (range_max - du / 2)) u = range_max;

    /* position of the robot corresponding to parameter u */
    q = configAtParam(u);
    //        q = m_Robot->getCurrentPos();

    m_Robot->setAndUpdate(*q);

    pf = drawnjnt->getVectorPos();

    // cout << "u : " << u << " , pi : " << pi.transpose() << " , pf : " <<
    // pf.transpose() << endl;

    if (m_isHighestCostIdSet) {
      if (getLocalPathId(u) == m_HighestCostId && !red) {
        red = true;
        saveColor = m_Color;
        m_Color = 3;
      }
      if ((m_Color == 3) && (getLocalPathId(u) != m_HighestCostId) && red) {
        m_Color = saveColor;
        red = false;
      }
    }

    double height_i, height_f;

    if ((!ENV.getBool(Env::isCostSpace)) || (GroundCostObj == NULL)) {
      height_i = pi[2];
      height_f = pf[2];
    } else {
      /*val1 =*/GHintersectionVerticalLineWithGround(
          GroundCostObj, pi[0], pi[1], &Cost1);
      /*val2 =*/GHintersectionVerticalLineWithGround(
          GroundCostObj, pf[0], pf[1], &Cost2);
      height_i = Cost1 + (ZmaxEnv - ZminEnv) * 0.02;
      height_f = Cost2 + (ZmaxEnv - ZminEnv) * 0.02;
    }

    if (move3d_use_api_functions()) glLineWidth(3.);

    if (!m_use_continuous_color) {
      move3d_draw_one_line(pi[0],
                           pi[1],
                           height_i,
                           pf[0],
                           pf[1],
                           height_f,
                           int(m_Color) /*% 8*/,
                           NULL,
                           m_Robot);
    } else {
      double colorvector[4];
      GroundColorMixGreenToRed(colorvector, m_Color);
      move3d_draw_one_line(pi[0],
                           pi[1],
                           height_i,
                           pf[0],
                           pf[1],
                           height_f,
                           Any,
                           colorvector,
                           m_Robot);
    }

    if (move3d_use_api_functions()) glLineWidth(1.);

    pi = pf;
    u += du;
  }

  //    if ((ENV.getBool(Env::isCostSpace)) && (GroundCostObj != NULL))
  //    {
  //        for(size_t i=0; i<m_Courbe.size(); i++)
  //        {
  //            m_Robot->setAndUpdate(*m_Courbe[i]->getEnd());
  //            pf = drawnjnt->getVectorPos();
  //            double colorvector[4];
  //            /*val2 =*/ GHintersectionVerticalLineWithGround( GroundCostObj,
  //            pf[0], pf[1], &Cost2 );
  //            move3d_draw_sphere( pf[0], pf[1], Cost2 + (ZmaxEnv - ZminEnv) *
  //            0.02, 1. /*, 10*/, colorvector, m_Robot );
  //        }
  //    }

  //    cout << "Draw trajectory" << endl;
  //    cout << "Draw joint : " << drawnjnt->getName() << ", index : " <<
  //    p3d_get_user_drawnjnt() << endl;

  m_Robot->setAndUpdate(*qSave);
}

bool Trajectory::push_back(shared_ptr<LocalPath> path) {
  if (m_Courbe.empty()) {
    m_Source = path->getBegin();
    m_Target = path->getEnd();

    m_Courbe.push_back(new LocalPath(*path));
  } else {
    if (m_Target->equal(*path->getBegin())) {
      m_Courbe.push_back(new LocalPath(*path));
      m_Target = path->getEnd();
    } else {
      return false;
    }
  }

  return true;
}

bool Trajectory::push_back(confPtr_t q, double dt) {
  if (!push_back(q)) return false;

  if (m_use_time_parameter && !m_use_constant_dt) m_dts.push_back(dt);

  return true;
}

bool Trajectory::push_back(confPtr_t q) {
  bool add_config = false;

  if (m_Courbe.empty()) {
    if (m_Source->getConfigStruct() == NULL) {
      m_Source = q;
      add_config = true;
    } else {
      if (!m_Source->equal(*q)) {
        m_Courbe.push_back(new LocalPath(m_Source, q));
        m_Target = q;
        add_config = true;
      }
      //            else
      //                cout << "source equal q" << endl;
    }
  } else {
    if (!m_Target->equal(*q)) {
      m_Courbe.push_back(new LocalPath(m_Target, q));
      m_Target = q;
      add_config = true;
    }
    //        else
    //            cout << "target equal q" << endl;
  }

  return add_config;
}

bool Trajectory::cutTrajInSmallLPSimple(unsigned int nLP, bool use_time) {
  double range = use_time ? getDuration() : computeSubPortionRange(m_Courbe);
  double delta = range / double(nLP);

  vector<LocalPath*> portion;
  bool null_length_local_path = false;
  double length = 0.0;
  double t = 0.0;
  // cout << "use_time : " << use_time << " (" << m_dt << "), range : " << range
  // << " , delta : " << delta << endl;

  for (unsigned int i = 0; i < nLP; i++) {
    confPtr_t q_init = use_time ? configAtTime(t) : configAtParam(t);
    confPtr_t q_goal =
        use_time ? configAtTime(t + delta) : configAtParam(t + delta);
    portion.push_back(new LocalPath(q_init, q_goal));
    t += delta;

    double path_length = portion.back()->getParamMax();
    if (path_length == 0.0) null_length_local_path = true;

    length += portion.back()->getParamMax();
    //        cout << "length : " << length << endl;
  }

  if (use_time) {  // TODO make this work for all types of time
    m_dt = delta;
    // cout << "set dt to : " << std::scientific << m_dt << endl;
  }

  m_Courbe = portion;

  if (portion.size() != nLP) {
    throw string("Error: int cutTrajInSmallLPSimple");
  }

  if (length == 0.0) {
    cout << "Null length trajectory in cutTrajInSmallLPSimple" << endl;
    return false;
  }

  if (null_length_local_path) {
    cout << "Null length localpath in cutTrajInSmallLPSimple" << endl;
    return false;
  }

  return true;
}

unsigned int Trajectory::cutPortionInSmallLP(vector<LocalPath*>& portion,
                                             unsigned int nLP) {
  double range = computeSubPortionRange(portion);

  double soFar = 0.0;
  double prevSoFar = 0.0;
  double param = 0.0;

  if (nLP == 0) {
    cout << "Error: cutPortionInSmallLP" << endl;
  }

  if (nLP == 1) {
    return nLP;
  }

  if (portion.empty()) {
    cout << "Can not cut an empty portion" << endl;
    return 0;
  }

  if (range == 0.0) {
    cout << "Can not cut a zero range trajectory" << endl;
    return 0;
  }

  cout << "NB Of LP = " << portion.size() << endl;
  cout << "NB Of LP = " << nLP << endl;

  confPtr_t confPtr;
  confPtr_t confPtrTmp;

  prevSoFar = 0.0;
  soFar = portion[0]->getParamMax();
  confPtrTmp = portion[0]->getBegin();

  const double dMax = range / nLP;

  // Compute real number of small LP
  unsigned int nbOfSmallLP = 0;

  for (int i = 0; i < int(portion.size()); i++) {
    double resol;
    double length = portion[i]->getParamMax();

    if (length < dMax) {
      resol = length;
    } else if (floor(length / dMax) == length / dMax) {
      resol = dMax;
    } else {
      double n = floor(length / dMax);  // minimal number of segment
      resol = length / n;
    }

    unsigned int nbOfDMax = floor((portion[i]->getParamMax() / resol) + 0.5);
    nbOfSmallLP += nbOfDMax;
  }
  cout << "range = " << range << endl;
  cout << "nbOfSmallLP = " << nbOfSmallLP << endl;
  vector<LocalPath*> portionTmp;

  try {
    int j = 0;
    bool tooFar = false;

    // Loop for every small localpath
    for (int i = 0; i < int(nbOfSmallLP); i++) {
      double resol;
      double length = portion[j]->getParamMax();

      if (length < dMax) {
        resol = length;
      } else if (floor(length / dMax) == length / dMax) {
        resol = dMax;
      } else {
        double n = floor(length / dMax);  // minimal number of segment
        resol = length / n;
      }

      param += resol;

      // Check to go seek the conf on next big LP
      while (param > soFar) {
        j++;
        if (j >= int(portion.size())) {
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
      } else {
        break;
      }
    }
  } catch (string str) {
    cout << "Exeption in cutPortionInSmallLP" << endl;
    cout << str << endl;
    return 0;
  } catch (...) {
    cout << "Exeption in cutPortionInSmallLP" << endl;
    return 0;
  }

  // The last LP is made of the end configuration
  if (portionTmp.size() >= 2) {
    confPtrTmp = portionTmp[portionTmp.size() - 2]->getEnd();
  } else {
    // portionTmp.back
    confPtrTmp = portionTmp.back()->getBegin();
  }

  confPtr = portion.back()->getEnd();

  delete portionTmp.back();
  portionTmp.back() = new LocalPath(confPtrTmp, confPtr);

  cout << "old range = " << range << endl;
  cout << "new range = " << computeSubPortionRange(portionTmp) << endl;
  cout << "portionTmp.size() : " << portionTmp.size() << endl;

  // Delete and replace every thing
  for (int i = 0; i < int(portion.size()); i++) {
    delete portion.at(i);
  }

  portion.clear();
  portion = portionTmp;

  if (nLP != portion.size()) {
    cout << "Error: cutPortionInSmallLP ( nLP = " << nLP
         << " , portion.size() = " << portion.size() << " )" << endl;
  }

  return portion.size();
}

bool Trajectory::cutTrajInSmallLP(unsigned int nLP) {
  bool succeed = true;

  try {
    // cutPortionInSmallLP(m_Courbe, nLP);
    succeed = cutTrajInSmallLPSimple(nLP, m_use_time_parameter);
  } catch (string str) {
    cout << "Exeption in cutTrajInSmallLP" << endl;
    cout << str << endl;
    return false;
  } catch (...) {
    cout << "Exeption in cutTrajInSmallLP" << endl;
    return false;
  }

  // cout << "Cutting into " << nLP << " local paths" << endl;
  // cout << "Traj Size = " << m_Courbe.size() << endl;
  // cout << "Cost Of trajectory :" << this->cost() << endl;

  if (!succeed) return false;

  //    if (!m_Source->equal(*configAtParam(0)))
  //    {
  //        cout << "Error" << endl;
  //        return false;
  //    }

  //    if (!m_Target->equal(*configAtParam(getParamMax())))
  //    {
  //        m_Source->print();
  //        m_Target->print();
  //        configAtParam( getParamMax() )->print();
  //        cout << "Error" << endl;
  //        return false;
  //    }
  return true;
}

bool Trajectory::concat(const Trajectory& traj) {
  if (traj.m_Courbe.size() == 0) return true;

  if (!m_Courbe.back()->getEnd()->equal(*traj.m_Courbe[0]->getBegin())) {
    m_Courbe.back()->getEnd()->print();
    traj.m_Courbe[0]->getBegin()->print();
    m_Courbe.back()->getEnd()->equal(*traj.m_Courbe[0]->getBegin(), true);
    return false;
  }

  if (m_Courbe.size() == 0) {
    m_Source = traj.m_Courbe[0]->getBegin();
  }

  for (int i = 0; i < int(traj.m_Courbe.size()); i++) {
    m_Courbe.push_back(new LocalPath(*traj.m_Courbe[i]));
  }
  m_Target = traj.m_Courbe.back()->getEnd();
  return true;
}

bool Trajectory::replacePortionOfLocalPaths(unsigned int id1,
                                            unsigned int id2,
                                            vector<LocalPath*> paths,
                                            bool freeMemory) {
  // WARNING: the ids are ids of nodes and not LocalPaths
  if (id1 == id2) {
    cout << "Error: in replace local (id1 and id2 are the same)" << endl;
    return false;
  }
  if (id2 < id1) {
    cout << "Error: in replace local (id2 > id2)" << endl;
    return false;
  }
  if (freeMemory) {
    for (uint i = id1; i < id2; i++) {
      delete m_Courbe[i];
    }
  }

  m_Courbe.erase(m_Courbe.begin() + id1, m_Courbe.begin() + id2);
  m_Courbe.insert(m_Courbe.begin() + id1, paths.begin(), paths.end());
  return true;
}

bool Trajectory::replacePortion(double param1,
                                double param2,
                                vector<LocalPath*> paths,
                                bool freeMemory) {
  confPtr_t q11 = paths.at(0)->getBegin();
  confPtr_t q12 = paths.back()->getEnd();

  confPtr_t q21;
  confPtr_t q22;

  if (param1 > param2) {
    cout << "Warning: Error not possible to replace trajectory because of the "
            "parameters" << endl;
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

  for (unsigned int i = 0; i < m_Courbe.size(); i++) {
    soFar = soFar + m_Courbe[i]->getParamMax();

    // param1 is in local path i
    if (param1 < soFar) {
      // get configuration in local path i
      q21 = m_Courbe[i]->configAtParam(param1 - prevSoFar);
      // param_start = param1 - prevSoFar;
      id_LP_1 = i;
      break;
    }
    prevSoFar = soFar;
    if (i == (getParamMax() - 1)) {
      cout << "Warning: first parameter not found on trajectory" << endl;
      //			return;
    }
  }

  soFar = prevSoFar;

  for (unsigned int i = id_LP_1; i < m_Courbe.size(); i++) {
    soFar = soFar + m_Courbe[i]->getParamMax();

    // param2 is in local path i
    if (param2 < soFar) {
      // get configuration in local path i
      q22 = m_Courbe.at(i)->configAtParam(param2 - prevSoFar);
      //            param_end = param2 - soFar;
      id_LP_2 = i;
      break;
    }
    prevSoFar = soFar;

    if (i == (m_Courbe.size() - 1)) {
      q22 = m_Courbe.at(m_Courbe.size() - 1)->getEnd();
      id_LP_2 = i;
      //            param_end = soFar;
    }
  }

  // Condition has to be false if the trajectory is consistent
  if ((!q11->equal(*q21)) || (!q12->equal(*q22))) {
    if (!q11->equal(*q21)) {
      cout << "q11 and q21" << endl;
      q11->print();
      q21->print();
    }
    if (!q12->equal(*q22)) {
      cout << "q12 and q22" << endl;
      q12->print();
      q22->print();
    }
    cout << "Warning: Error not possible to replace trajectory because "
            "configuration are not the same" << endl;
    return false;
  }

  // Adds to paths the portion of local path (At begin and End of the portion)
  //----------------------------------------------------------------------------
  confPtr_t start = m_Courbe[id_LP_1]->getBegin();
  confPtr_t end = m_Courbe[id_LP_2]->getEnd();

  /*cout << "Start and End" << endl;
   start->print(); end->print();
   cout << "q21 and q22" << endl;
   q21->print(); q22->print();*/

  // Verifies that the configuration is not starting the local path
  if (!start->equal(*q21)) {
    LocalPath* startNew = new LocalPath(start, q21);

    if (startNew->isValid()) {
      paths.insert(paths.begin(), new LocalPath(start, q21));
    } else {
      // cout << "Error: portion of path not valid" << endl;
      return false;
    }
  }

  // Verifies that the configuration is not ending the local path
  if (!end->equal(*q22)) {
    LocalPath* endNew = new LocalPath(q22, end);

    if (endNew->isValid()) {
      paths.push_back(endNew);
    } else {
      // cout << "Error: portion of path not valid" << endl;
      return false;
    }
  }

  /*for(int i=0;i<paths.size();i++)
   {
   paths[i]->print();
   }*/
  //	cout << "PathSizeOf() = " << paths.size() << endl;
  // TODO optional
  // cutPortionInSmallLP(paths);

  // Free, Erases the old ones and the Inserts the new ones
  //---------------------------------------------------------------------------
  unsigned int id1_erase = id_LP_1;
  unsigned int id2_erase = id_LP_2 + 1;

  replacePortionOfLocalPaths(id1_erase, id2_erase, paths, freeMemory);
  return true;
}

bool Trajectory::replaceBegin(double param, const vector<LocalPath*>& paths) {
  uint first(0);
  uint last(0);
  vector<LocalPath*> new_courbe;
  cout << "Replace begining at " << param << " over (" << getParamMax() << ")"
       << endl;

  pair<bool, vector<LocalPath*> > valid_portion =
      extractSubPortion(param, getParamMax(), first, last, false);

  if (valid_portion.first) {
    new_courbe = paths;
  } else {
    cout << "Error: inconsistant query in replaceBegin" << endl;
    return false;
  }

  for (int i = 0; i < int(valid_portion.second.size()); i++) {
    new_courbe.push_back(valid_portion.second[i]);
  }

  for (int i = 0; i < int(m_Courbe.size()); i++) {
    delete m_Courbe[i];
  }

  m_Courbe = new_courbe;
  m_Source = new_courbe.at(0)->getBegin();
  m_Target = new_courbe.back()->getEnd();
  return true;
}

bool Trajectory::replaceEnd(double param, const vector<LocalPath*>& paths) {
  uint first(0);
  uint last(0);
  vector<LocalPath*> new_courbe;

  pair<bool, vector<LocalPath*> > valid_portion =
      extractSubPortion(0.0, param, first, last, false);

  if (valid_portion.first) {
    new_courbe = valid_portion.second;
  } else {
    cout << "Error: inconsistant query in replaceEnd" << endl;
    return false;
  }

  for (int i = 0; i < int(paths.size()); i++) {
    new_courbe.push_back(paths[i]);
  }

  for (int i = 0; i < int(m_Courbe.size()); i++) {
    delete m_Courbe[i];
  }

  m_Courbe = new_courbe;
  m_Source = new_courbe.at(0)->getBegin();
  m_Target = new_courbe.back()->getEnd();
  return true;
}

unsigned int Trajectory::getLocalPathId(double param) const {
  double soFar(0.0);

  for (unsigned int i = 0; i < m_Courbe.size(); i++) {
    soFar = soFar + m_Courbe[i]->getParamMax();

    if (param < soFar) {
      return i;
    }
  }
  return m_Courbe.size() - 1;
}

int Trajectory::meanCollTest() {
  int CollTest = 0.0;
  for (unsigned int i = 0; i < m_Courbe.size(); i++) {
    if (!(m_Courbe[i]->isValid())) {
      cout << "Trajectory::Warning => LocalPath is not valid in trajectory"
           << endl;
    }
    CollTest += m_Courbe[i]->getNbColTest();
  }
  return (int)(CollTest / m_Courbe.size());
}

vector<double> Trajectory::getCostAlongTrajectory(int nbSample) {
  double step = this->getParamMax() / (double)nbSample;

  vector<double> cost;

  for (double param = 0; param < this->getParamMax(); param = param + step) {
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
Eigen::MatrixXd Trajectory::getJointPoseTrajectory(
    const std::vector<Move3D::Joint*>& joints) const {
  if (m_Courbe.size() > 0) {
    int rows = int(joints.size()) * 3;
    int cols = m_Courbe.size() + 1;

    Eigen::MatrixXd mat(rows, cols);

    for (int j = 0; j < int(m_Courbe.size()); j++) {
      m_Robot->setAndUpdate(*m_Courbe[j]->getBegin());

      for (int i = 0; i < int(joints.size()); i++) {
        Eigen::Vector3d p(joints[i]->getVectorPos());
        mat.col(j)[0 + 3 * i] = p[0];
        mat.col(j)[1 + 3 * i] = p[1];
        mat.col(j)[2 + 3 * i] = p[2];
      }
    }

    if (m_Courbe.size() - 1 >= 0) {
      m_Robot->setAndUpdate(*m_Courbe.back()->getEnd());
      for (int i = 0; i < int(joints.size()); i++) {
        Eigen::Vector3d p(joints[i]->getVectorPos());
        mat.col(m_Courbe.size())[0 + 3 * i] = p[0];
        mat.col(m_Courbe.size())[1 + 3 * i] = p[1];
        mat.col(m_Courbe.size())[2 + 3 * i] = p[2];
      }
    }

    return mat;
  } else {
    return Eigen::MatrixXd(0, 0);
  }
}

// Returns a matrix with the waypoints of the trajectory
// The number of rows is the number of dofs
// The number of cols is the number of waypoints
Eigen::MatrixXd Trajectory::getJointPoseTrajectory(
    const Move3D::Joint* joint) const {
  if (m_Courbe.size() > 0) {
    int rows = 7;
    int cols = m_Courbe.size() + 1;

    Eigen::MatrixXd mat(rows, cols);

    for (int j = 0; j < int(m_Courbe.size()); j++) {
      m_Robot->setAndUpdate(*m_Courbe[j]->getBegin());
      Eigen::Transform3d T(joint->getMatrixPos());
      Eigen::Quaterniond q(T.rotation());
      mat.col(j)[0] = T.translation().x();
      mat.col(j)[1] = T.translation().y();
      mat.col(j)[2] = T.translation().z();
      mat.col(j)[3] = q.w();
      mat.col(j)[4] = q.x();
      mat.col(j)[5] = q.y();
      mat.col(j)[6] = q.z();
    }

    if (m_Courbe.size() - 1 >= 0) {
      m_Robot->setAndUpdate(*m_Courbe.back()->getEnd());
      Eigen::Transform3d T(joint->getMatrixPos());

      // cout << "translation : " << T.translation() << endl;
      // cout << "rotation : " << T.rotation() << endl;

      Eigen::Quaterniond q(T.rotation());

      mat.col(m_Courbe.size())[0] = T.translation().x();
      mat.col(m_Courbe.size())[1] = T.translation().y();
      mat.col(m_Courbe.size())[2] = T.translation().z();
      mat.col(m_Courbe.size())[3] = q.w();
      mat.col(m_Courbe.size())[4] = q.x();
      mat.col(m_Courbe.size())[5] = q.y();
      mat.col(m_Courbe.size())[6] = q.z();
    }

    return mat;
  } else {
    return Eigen::MatrixXd(0, 0);
  }
}

// Returns a matrix with the waypoints of the trajectory
// The number of rows is the number of dofs
// The number of cols is the number of waypoints
Eigen::MatrixXd Trajectory::getEigenMatrix(int startIndex, int endIndex) const {
  if (startIndex == 0 && endIndex == 0) {
    if (m_Courbe.size() > 0) {
      int rows = m_Courbe[0]->getBegin()->getEigenVector().size();
      int cols = m_Courbe.size() + 1;

      Eigen::MatrixXd mat(rows, cols);

      for (int j = 0; j < int(m_Courbe.size()); j++) {
        mat.col(j) = m_Courbe[j]->getBegin()->getEigenVector();
      }

      if (m_Courbe.size() - 1 >= 0) {
        mat.col(m_Courbe.size()) = m_Courbe.back()->getEnd()->getEigenVector();
      }

      return mat;
    } else {
      return Eigen::MatrixXd(0, 0);
    }
  } else {
    // Get indicies in order
    std::vector<int> incides;
    for (int i = startIndex; i <= endIndex; i++) incides.push_back(i);

    return getEigenMatrix(incides);
  }
}

// Returns a matrix with the waypoints of the trajectory
// The number of rows is the number of dofs
// The number of cols is the number of waypoints
Eigen::MatrixXd Trajectory::getEigenMatrix(
    const std::vector<int>& dof_indices) const {
  if (m_Courbe.size() > 0) {
    int rows = dof_indices.size();
    int cols = m_Courbe.size() + 1;

    Eigen::MatrixXd mat(rows, cols);

    for (int j = 0; j < int(m_Courbe.size()); j++) {
      mat.col(j) = m_Courbe[j]->getBegin()->getEigenVector(dof_indices);
    }

    if (m_Courbe.size() - 1 >= 0) {
      mat.col(m_Courbe.size()) =
          m_Courbe.back()->getEnd()->getEigenVector(dof_indices);
    }

    return mat;
  } else {
    return Eigen::MatrixXd(0, 0);
  }
}

bool Trajectory::setFromEigenMatrix(const Eigen::MatrixXd& mat,
                                    const std::vector<int>& dof_indices) {
  m_Courbe.clear();

  confPtr_t q_cur = m_Robot->getCurrentPos();

  for (int j = 0; j < mat.cols(); j++) {
    confPtr_t q = q_cur->copy();

    for (int i = 0; i < int(dof_indices.size()); i++) {
      (*q)[dof_indices[i]] = mat(i, j);
    }

    push_back(q);
  }

  return true;
}

bool Trajectory::saveToFile(std::string filename) const {
  std::vector<int> r_dof_indices = m_Robot->getAllDofIds();
  Eigen::MatrixXd mat = getEigenMatrix(r_dof_indices);

  Eigen::MatrixXd mat_time(mat.rows(), mat.cols() + 1);
  mat_time.block(0, 1, mat.rows(), mat.cols()) = mat;

  for (int i = 0; i < mat.rows(); i++) {
    if (m_use_time_parameter) {
      if (m_use_constant_dt) mat_time(i, 0) = m_dt;
      //            else
      //                mat_time(i,0) = m_dts[i];
    } else
      mat_time(i, 0) = -1.;
  }

  // cout << mat << endl;
  move3d_save_matrix_to_csv_file(mat_time, filename);
  return true;
}

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove) {
  unsigned int numRows = matrix.rows();
  unsigned int numCols = matrix.cols() - 1;

  if (colToRemove < numCols)
    matrix.block(0, colToRemove, numRows, numCols - colToRemove) =
        matrix.block(0, colToRemove + 1, numRows, numCols - colToRemove);

  matrix.conservativeResize(numRows, numCols);
}

bool Trajectory::loadFromFile(std::string filename) {
  if (m_Robot == NULL) return false;

  std::vector<int> r_dof_indices = m_Robot->getAllDofIds();

  Eigen::MatrixXd mat = move3d_load_matrix_from_csv_file(filename);
  if (mat.rows() == 0 && mat.cols() == 0) return false;

  if (mat(0, 0) != -1.) {
    m_use_time_parameter = true;
    m_use_constant_dt = true;
    m_dt = mat(0, 0);
  }
  else {
    m_use_time_parameter = false;
  }

  removeColumn(mat, 0);

  // cout << mat << endl;
  setFromEigenMatrix(mat, r_dof_indices);
  return true;
}

void Trajectory::show() const {
  cout << __PRETTY_FUNCTION__ << endl;

  if (size() == 0) {
    cout << "warning : motion is empty" << endl;
    return;
  }

  cout << "m_use_time_parameter : " << m_use_time_parameter
       << " , size : " << m_Courbe.size() << endl;
  cout << "time length : " << getDuration() << endl;
  cout << "param max : " << getParamMax()
       << " , dmax : " << ENV.getDouble(Env::dmax) << endl;

  bool StopRun = false;
  double tu_init = 0.0, tu = 0.0, t = 0.0;

  while (!StopRun) {
    if (m_use_time_parameter) {
      t = tu - tu_init;
      m_Robot->setAndUpdate(*configAtTime(t));
    } else {
      m_Robot->setAndUpdate(*configAtParam(t));
    }

    bool ncol = m_Robot->isInCollision();

    if (ncol) cout << "Robot in collision " << endl;

    g3d_set_draw_coll(ncol);
    g3d_draw_allwin_active();

    if (m_use_time_parameter) {
      timeval tim;

      gettimeofday(&tim, NULL);
      tu = tim.tv_sec + (tim.tv_usec / 1000000.0);

      if (tu_init == 0.0) tu_init = tu;

      if (t >= getDuration()) StopRun = true;
    } else {
      usleep(20000);  // sleep 20ms (50Hz)

      t += ENV.getDouble(Env::dmax);

      if (t >= getParamMax()) StopRun = true;
    }

    if (PlanEnv->getBool(PlanParam::stopPlanner)) StopRun = true;
  }

  cout << "end " << __PRETTY_FUNCTION__ << endl;
}

void Trajectory::printAllLocalpathCost() {
  cout << "( ";
  for (int i = 0; i < int(m_Courbe.size()); i++) {
    cout << m_Courbe[i]->cost();

    if (i != int(m_Courbe.size() - 1)) {
      cout << " , ";
    } else {
      cout << " )" << endl;
    }
  }
}

void Trajectory::print() const {
  cout << "-------------- Trajectory --------------" << endl;
  cout << " Number of LP " << m_Courbe.size() << endl;
  cout << " Range Parameter " << this->getParamMax() << endl;

  Eigen::MatrixXd mat = getEigenMatrix();
  cout << mat << endl;
  cout << "-----------------------------------" << endl;
}

void draw_traj_debug() {
  if (ENV.getBool(Env::debugCostOptim) || ENV.getBool(Env::drawTrajVector)) {
    for (size_t i = 0; i < global_trajToDraw.size(); i++) {
      global_trajToDraw[i].draw(500);
    }
  }
}
