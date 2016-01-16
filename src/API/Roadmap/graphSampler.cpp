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
#include "graphSampler.hpp"

#include "API/project.hpp"
#include "planner/planEnvironment.hpp"

#include <iomanip>

extern void* GroundCostObj;

using namespace Move3D;

using std::cout;
using std::cerr;
using std::endl;

graphSampler::graphSampler() {
  num_points_per_dim_ = 2;
  num_joints_ = 2;
  vect_length_ =
      num_joints_ * pow(double(num_points_per_dim_), double(num_joints_));
  noise_generator_ = NULL;
}

graphSampler::graphSampler(int num_points_per_dim, int num_joints)
    : num_points_per_dim_(num_points_per_dim), num_joints_(num_joints) {
  vect_length_ =
      num_joints_ * pow(double(num_points_per_dim_), double(num_joints_));
  noise_generator_ = NULL;
}

graphSampler::~graphSampler() { delete noise_generator_; }

void graphSampler::initialize() {
  tmp_noise_ = Eigen::VectorXd::Zero(vect_length_);
  precision_ = Eigen::MatrixXd::Zero(vect_length_, vect_length_);

  //    precision_ <<   1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 1  -> 1,1
  //                    0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 2  -> 1,2
  //                    0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 3  -> 1,3
  //                    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 4  -> 1,4
  //                    0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 5  -> 2,1
  //                    0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 6  -> 2,2
  //                    0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 7  -> 2,3
  //                    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 8  -> 2,4
  //                    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 9  -> 3,1
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10 -> 3,2
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 11 -> 3,3
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 12 -> 3,4
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 13 -> 4,1
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 14 -> 4,2
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 15 -> 4,3
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 16 -> 4,4
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
  //                    1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  //                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

  // double a = 1.0;
  // double b = 0.5;
  // double c = 0.01;

  double a = PlanEnv->getDouble(PlanParam::samplegraphVarianceA);
  double c = PlanEnv->getDouble(PlanParam::samplegraphVarianceB);

  // x  x  x  x      y  y  y  y
  // 1, 2, 3, 4      1, 2, 3, 4
  precision_ << 1, a, c, a, 0, a, c, c,  // 1  -> 1,1 x
      a, 1, a, c, c, 0, a, a,            // 2  -> 1,2 x
      c, a, 1, a, a, a, 0, c,            // 3  -> 2,1 x
      a, c, a, 1, c, a, c, 0,            // 4  -> 2,2 x

      0, c, a, c, 1, c, a, a,  // 1  -> 1,1 y
      a, 0, a, a, c, 1, a, a,  // 2  -> 1,2 y
      c, a, 0, c, a, a, 1, c,  // 3  -> 2,1 y
      c, a, c, 0, a, a, c, 1;  // 4  -> 2,1 y

  preAllocateMultivariateGaussianSampler();
}

bool graphSampler::preAllocateMultivariateGaussianSampler() {
  // invert the control costs, initialize noise generators:
  inv_precision_ = precision_.inverse();

  cout.precision(3);
  cout << inv_precision_ << endl;  // << std::scientific << endl;

  // TODO see of the noise generator needs to be
  // var free or var all
  MultivariateGaussian* mvg = new MultivariateGaussian(
      Eigen::VectorXd::Zero(vect_length_), inv_precision_);
  noise_generator_ = mvg;

  return true;
}

Eigen::VectorXd graphSampler::sample_noisy(double std_dev) {
  // Eigen::VectorXd tmp;
  cout << "tmp_noise_ : " << tmp_noise_.transpose() << endl;
  noise_generator_->sample(tmp_noise_);
  tmp_noise_ *= std_dev;
  cout << "tmp_noise_ : " << tmp_noise_.transpose() << endl;
  return tmp_noise_;
}

void graphSampler::setNodesInGraph(Graph* g) {
  int ith = 1;
  for (int i = 0; i < num_points_per_dim_; i++) {
    for (int j = 0; j < num_points_per_dim_; j++) {
      confPtr_t q = g->getRobot()->getCurrentPos();
      int x_id = num_points_per_dim_ * i + j;
      int y_id = num_points_per_dim_ * i + j + num_points_per_dim_ * 2;
      (*q)[6] = tmp_noise_[x_id];
      (*q)[7] = tmp_noise_[y_id];
      cout << "node( " << x_id << " , " << y_id << " ) : " << (*q)[6] << " , "
           << (*q)[7] << endl;
      Node* N = new Node(g, q);
      N->color_ = ith++;
      g->addNode(N);
    }
  }
}

Graph* graphSampler::sample() {
  Robot* robot = global_Project->getActiveScene()->getActiveRobot();
  Graph* graph = new Graph(robot);

  int iterations = 1;
  if (PlanEnv->getBool(PlanParam::samplegraphMultiLoop)) iterations = 100;

  for (int i = 0; i < iterations; i++) {
    sample_noisy(20);
    setNodesInGraph(graph);
  }
  cout << "Nb of nodes : " << graph->getNumberOfNodes() << endl;

  delete API_activeGraph;
  API_activeGraph = graph;
  return graph;
}

Graph* graphSampler::makeGrid(int DichotomicFactor) {
  Robot* robot = global_Project->getActiveScene()->getActiveRobot();
  cout << "Make grid for robot : " << robot->getName() << endl;

  Graph* graph = new Graph(robot);
  Node* newNodePt;
  double vMinDof1, vMaxDof1, vMinDof2, vMaxDof2;
  int nbPart = std::pow(2, DichotomicFactor), i, j, count = 0;
  confPtr_t q;
  int indPrev;
  bool compute_edge_cost = false;
  //    std::vector<Node*> nodes;

  Joint* joint = robot->getJoint(1);
  joint->getDofRandBounds(0, vMinDof1, vMaxDof1);
  joint->getDofRandBounds(1, vMinDof2, vMaxDof2);

  for (i = 0; i < nbPart; i++) {
    Node* prevJNode = NULL;

    for (j = 0; j < nbPart; j++) {
      q = robot->getCurrentPos();
      (*q)[6] = vMinDof1 + i * (vMaxDof1 - vMinDof1) / (nbPart - 1);
      (*q)[7] = vMinDof2 + j * (vMaxDof2 - vMinDof2) / (nbPart - 1);

      newNodePt = new Node(graph, q);
      if (newNodePt == NULL) {
        cerr << "Error: Failed to create a new node for an optimal cost search "
                "\n";
        delete graph;
        return NULL;
      }

      graph->addNode(newNodePt);

      if (prevJNode == NULL) {
        prevJNode = newNodePt;
      } else {
        // edge linking previous node in the j direction
        Node* node = prevJNode;
        LocalPath path(node->getConfiguration(), newNodePt->getConfiguration());
        graph->mergeComp(
            node, newNodePt, path.getParamMax(), compute_edge_cost);
        prevJNode = newNodePt;
      }
      // count is the current counter of the node and
      // indPrev is the indice of the previous node in the i direction
      indPrev = count - nbPart;

      // edge linking previous node - 1 (diagonal) in the i direction
      if (((indPrev - 1) >= 0) && ((indPrev - 1) % nbPart != (nbPart - 1))) {
        Node* node = graph->getNodes()[indPrev - 1];
        LocalPath path(node->getConfiguration(), newNodePt->getConfiguration());
        graph->mergeComp(
            node, newNodePt, path.getParamMax(), compute_edge_cost);
      }
      // edge linking previous node in the i direction
      if (indPrev >= 0) {
        Node* node = graph->getNodes()[indPrev];
        LocalPath path(node->getConfiguration(), newNodePt->getConfiguration());
        graph->mergeComp(
            node, newNodePt, path.getParamMax(), compute_edge_cost);
      }
      // edge linking previous node +1 in the i direction
      if (((indPrev + 1) >= 0) && (((indPrev + 1) % nbPart != 0))) {
        Node* node = graph->getNodes()[indPrev + 1];
        LocalPath path(node->getConfiguration(), newNodePt->getConfiguration());
        graph->mergeComp(
            node, newNodePt, path.getParamMax(), compute_edge_cost);
      }
      count++;
    }
  }

  cout << "End make grid" << endl;

  delete API_activeGraph;
  API_activeGraph = graph;
  return graph;
}
