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
#include "misc_functions.hpp"
#include "NumsAndStrings.hpp"
#include "API/project.hpp"
#include "API/libmove3d_api.hpp"
#include "planEnvironment.hpp"
#include "hri_costspace/gestures/HRICS_gest_parameters.hpp"
#include "hri_costspace/HRICS_parameters.hpp"

#include <iostream>
#include <fstream>
#include <dirent.h>

// For random number generator (seed can be passed as argument)
#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Util-pkg.h>
#include <libmove3d/include/Collision-pkg.h>

using std::cout;
using std::endl;

bool no_save_to_file = true;

void save_strings_to_file(std::vector<std::string> strings,
                          std::string filename) {
  if (!strings.empty()) {
    std::ofstream file(filename.c_str());

    if (file.is_open()) {
      for (size_t i = 0; i < strings.size(); i++) {
        file << std::scientific << strings[i] << '\n';
      }
      file.close();
    }
  }
}

std::vector<std::string> load_strings_from_file(std::string filename) {
  std::vector<std::string> result;
  std::ifstream file(filename.c_str(), std::ifstream::in);

  if (file.is_open()) {
    std::string line;
    std::string cell;

    while (file.good()) {
      std::getline(file, line);
      std::stringstream line_stream(line);

      while (std::getline(line_stream, cell, ',')) {
        result.push_back(cell);
      }
    }
  }

  return result;
}

void move3d_save_matrix_to_file(const Eigen::MatrixXd& matrix,
                                std::string filename) {
  cout << "save matrix to : " << filename << endl;
  std::ofstream file(filename.c_str());
  if (file.is_open()) file << std::scientific << matrix << '\n';
  //        for( int i=0;i<mat.rows();i++){
  //            for( int j=0;j<mat.cols();j++)
  //                file << mat(i,j) << " ";
  //            file << endl;
  //        }
  file.close();
}

void move3d_save_matrix_to_csv_file(const Eigen::MatrixXd& matrix,
                                    std::string filename) {
  std::ofstream s;
  s.open(filename.c_str());

  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      s << matrix(i, j);

      // Warning introduction of potential bug
      // TODO make sure it works in a test file
      if (j < matrix.cols() - 1) s << ",";
    }
    s << endl;
  }

  s.close();
}

// general case, stream interface
inline size_t word_count(std::stringstream& is)
// can pass an open std::ifstream() to this if required
{
  //    cout << is.str() << endl;
  size_t c = 0;
  for (std::string w; std::getline(is, w, ','); ++c)
    ;
  //        cout << "found word : " << w << endl;
  return c;
}

// simple string interface
inline size_t word_count(const std::string& str) {
  //    cout << "line is : " << str << endl;
  std::stringstream ss(str);
  return word_count(ss);
}

Eigen::MatrixXd move3d_load_matrix_from_csv_file(std::string filename) {
  Eigen::MatrixXd matrix;
  //    cout << "load matrix from : " << filename << endl;

  std::ifstream file(filename.c_str(), std::ifstream::in);

  if (file.good() && file.is_open()) {
    std::string line;
    std::string cell;

    int n_rows = std::count(std::istreambuf_iterator<char>(file),
                            std::istreambuf_iterator<char>(),
                            '\n');

    int i = 0, j = 0;

    file.clear();
    file.seekg(0, std::ios::beg);

    while (file.good()) {
      std::getline(file, line);
      std::stringstream lineStream(line);

      if (i == 0) {
        int n_cols = word_count(line);
        matrix = Eigen::MatrixXd(n_rows, n_cols);
        //                cout << "size : ( "
        // << n_rows << " , " << n_cols << " )" << endl;
      }

      j = 0;

      while (std::getline(lineStream, cell, ',')) {
        convert_text_to_num<double>(matrix(i, j), cell, std::dec);
        j++;
      }
      i++;
    }

    file.close();
  } else {
    cout << "could not open file : " << filename << endl;
  }

  return matrix;
}

std::vector<std::string> move3d_get_folders_in_folder(std::string foldername) {
  DIR* dir = opendir(foldername.c_str());

  std::cout << "Process directory: " << foldername.c_str() << std::endl;

  if (NULL == dir) {
    std::cout << "could not open directory: " << foldername.c_str()
              << std::endl;
    return std::vector<std::string>();
  }

  std::vector<std::string> folders;
  struct dirent* entity = readdir(dir);
  while (entity != NULL) {
    // find entity type
    if (entity->d_type == DT_DIR) {
      // it's a direcotry
      // don't process the  '..' and the '.' directories
      if (std::string(entity->d_name) != "." &&
          std::string(entity->d_name) != "..") {
        // it's an directory so process it
        folders.push_back(std::string(entity->d_name));
      }
    }

    entity = readdir(dir);
  }

  closedir(dir);
  return folders;
}

std::vector<std::string> move3d_get_files_in_folder(std::string foldername,
                                                    std::string extension,
                                                    int nb_max_files) {
  bool quiet = true;
  std::vector<std::string> files;

  std::string command = "ls " + foldername;
  FILE* fp = popen(command.c_str(), "r");
  if (fp == NULL) {
    cout << "ERROR in system call" << endl;
    return files;
  }

  char str[PATH_MAX];
  while (fgets(str, PATH_MAX, fp) != NULL) {
    std::string filename(str);
    filename = filename.substr(0, filename.size() - 1);
    std::string file_extension(filename.substr(filename.find_last_of(".") + 1));
    if (extension == file_extension) {
      if (!quiet) {
        cout << "add : " << filename << endl;
      }
      files.push_back(filename);
    }

    if ((nb_max_files > 0) && (int(files.size()) >= nb_max_files)) {
      break;
    }
  }
  pclose(fp);
  return files;
}

std::vector<int> move3d_change_basis(int number, int basis) {
  std::vector<int> result(basis, 0);

  size_t i = 1;
  while (number > 0)  // number in base 10
  {
    int place = basis - (i++);
    if (place < 0) {
      result.insert(result.begin(), number % basis);
    } else {
      result[place] = number % basis;
    }

    number = number / basis;
  }
  return result;
}

Eigen::VectorXd move3d_lerp(const Eigen::VectorXd& v0,
                            const Eigen::VectorXd& v1,
                            double t)
// t \in [0 1]
{
  return v0 + (v1 - v0) * t;  // v0*(1-t)+v1*t
}

void move3d_smooth_circular_parameters(Eigen::VectorXd& params) {
  double previous = params[0];

  for (int i = 1; i < params.size(); i++) {
    double diff = previous - params[i];
    double angle_diff = diff_angle(params[i], previous);

    while (std::fabs(angle_diff - diff) > 1e-12) {
      if (diff > 0)
        params[i] += 2 * M_PI;
      else
        params[i] -= 2 * M_PI;

      diff = previous - params[i];
      angle_diff = diff_angle(params[i], previous);

      //            cout << "i : " << i << endl;
      //            cout << "diff : "  << diff << endl;
      //            cout << "angle_diff : "  << angle_diff << endl;
    }

    previous = params[i];
  }
}

int move3d_random_integer(int min, int max) {
  int random_number = p3d_random_integer(min, max);
  // cout << "random number : " << random_number << endl;
  return random_number;
}

//! This function prints the joint mapping of a robot
void print_joint_mapping(Move3D::Robot* robot) {
  if (robot == NULL) return;

  cout << "print joint mapping" << endl;

  for (size_t i = 0; i < robot->getNumberOfJoints(); i++) {
    // cout << i
    // << " , joint name : " << rob->getJoint(i)->getName() << endl;

    for (size_t j = 0; j < robot->getJoint(i)->getNumberOfDof(); j++) {
      // cout << "jnt->getName() : "
      // << robot->getJoint(i)->getName() << "(" <<i<< ") , ";
      // cout << "index_dof : "
      // << robot->getJoint(i)->getIndexOfFirstDof()+j << endl;
      cout << "move3d_map[\"" << robot->getJoint(i)->getName()
           << "\"]=" << robot->getJoint(i)->getIndexOfFirstDof() + j << ";"
           << endl;
    }
  }
}

void print_joint_anchors(Move3D::Robot* robot) {
  if (robot == NULL) return;

  cout << "print joint mapping" << endl;

  for (size_t i = 0; i < robot->getNumberOfJoints(); i++) {
    Move3D::Joint* j = robot->getJoint(i);
    Move3D::Joint* j_prev = j->getPreviousJoint();
    if (j_prev == NULL) continue;

    //        Eigen::Transform3d T( j_prev->getMatrixPos().inverse() );
    Eigen::Vector3d v(j_prev->getVectorPos());

    cout << "j : " << j->getName() << " , j_prev : " << j_prev->getName()
         << " \t" << (j->getVectorPos() - v).transpose() << endl;
  }

  for (size_t j = 0; j < robot->getNumberOfJoints(); j++) {
    p3d_obj* o = robot->getJoint(j)->getP3dJointStruct()->o;

    if (o == NULL) continue;

    cout << "j : " << robot->getJoint(j)->getName() << endl;

    for (int i = 0; i < o->np; i++) {
      cout << "name : " << o->name << endl;
      // cout << i  << ": "
      // << o->jnt->abs_pos[0][3] << " "
      // << o->jnt->abs_pos[1][3] << " " << o->jnt->abs_pos[2][3] << endl;
      cout << i << ": " << o->pol[i]->pos_rel_jnt[0][3] << " "
           << o->pol[i]->pos_rel_jnt[1][3] << " "
           << o->pol[i]->pos_rel_jnt[2][3] << endl;

      cout << i << ": " << o->pol[i]->pos0[0][3] << " " << o->pol[i]->pos0[1][3]
           << " " << o->pol[i]->pos0[2][3] << endl;

      p3d_matrix4 t;
      p3d_matMultXform(o->jnt->abs_pos, o->pol[i]->pos_rel_jnt, t);
      cout << i << ": " << t[0][3] << " " << t[1][3] << " " << t[2][3] << endl;
    }
  }
}

std::vector<Move3D::confPtr_t> move3d_load_context_from_csv_file(
    std::string filename) {
  Move3D::Scene* sce = Move3D::global_Project->getActiveScene();
  std::vector<Move3D::confPtr_t> context;
  std::ifstream file(filename.c_str(), std::ifstream::in);

  cout << "load context from : " << filename << endl;

  if (file.is_open()) {
    std::string line;
    std::string cell;
    std::string robot_name;

    int n_rows = std::count(std::istreambuf_iterator<char>(file),
                            std::istreambuf_iterator<char>(),
                            '\n');

    context.resize(n_rows);

    cout << "nb of lines : " << n_rows << endl;

    int i = 0, j = 0;

    file.clear();
    file.seekg(0, std::ios::beg);

    while (file.good()) {
      std::getline(file, line);
      std::stringstream lineStream(line);

      int nb_of_cell_on_line = word_count(line);
      if (nb_of_cell_on_line < 2) {
        break;
      }

      Eigen::VectorXd q(Eigen::VectorXd::Zero(nb_of_cell_on_line - 1));

      std::getline(lineStream, robot_name, ',');
      Move3D::Robot* robot = sce->getRobotByName(robot_name);

      //            cout << "robot name : " << robot_name << endl;
      //            cout << "q size : " << q.size() << endl;

      j = 0;

      while (std::getline(lineStream, cell, ',')) {
        convert_text_to_num<double>(q(j++), cell, std::dec);
      }

      context[i] = Move3D::confPtr_t(new Move3D::Configuration(robot));
      context[i]->setFromEigenVector(q);
      //            cout << q.transpose() << endl;
      //            context[i]->print();

      i++;
    }

    file.close();
  } else {
    cout << "could not open file : " << filename << endl;
  }

  return context;
}

void move3d_save_context_to_csv_file(
    const std::vector<Move3D::confPtr_t>& context, std::string filename) {
  std::ofstream s;
  s.open(filename.c_str());

  for (size_t k = 0; k < context.size(); k++) {
    s << context[k]->getRobot()->getName() << ",";

    Eigen::VectorXd q = context[k]->getEigenVector();

    for (int i = 0; i < q.size(); i++) s << q(i) << ",";

    s << endl;
  }

  s.close();
}

Eigen::VectorXd moved_get_vector(const std::vector<std::string>& config) {
  Eigen::VectorXd tmp(config.size());

  for (int i = 0; i < int(config.size()); i++)
    convert_text_to_num<double>(tmp[i], config[i], std::dec);

  return tmp;
}

// Derived from code by Yohann Solaro
// (
// http://listengine.tuxfamily.org/lists.tuxfamily.org/eigen/2010/01/msg00187.html
// )
// see :
// http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method
Eigen::MatrixXd move3d_pinv(const Eigen::MatrixXd& b, double rcond) {
  bool flip = false;
  Eigen::MatrixXd a = b;

//  if (b.rows() < b.cols()) {
//    a = b.transpose();
//    flip = true;
//  } else {
//    a = b;
//  }

  // SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> svdA;
  svdA.compute(a, Eigen::ComputeFullU | Eigen::ComputeThinV);

  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType vSingular =
      svdA.singularValues();

  // Build a diagonal matrix with the Inverted Singular values
  // The pseudo inverted singular matrix is easy to compute :
  // is formed by replacing every nonzero entry by its reciprocal (inversing).
  Eigen::VectorXd vPseudoInvertedSingular(svdA.matrixV().cols());

  for (int iRow = 0; iRow < vSingular.rows(); iRow++) {
    if (fabs(vSingular(iRow)) <= rcond)  // Todo : Put epsilon in parameter
    {
      vPseudoInvertedSingular(iRow) = 0.;
    } else
      vPseudoInvertedSingular(iRow) = 1. / vSingular(iRow);
  }

  // A little optimization here
  Eigen::MatrixXd mAdjointU = svdA.matrixU().adjoint().block(
      0, 0, vSingular.rows(), svdA.matrixU().adjoint().cols());

  // Pseudo-Inversion : V * S * U'
  Eigen::MatrixXd a_pinv =
      (svdA.matrixV() * vPseudoInvertedSingular.asDiagonal()) * mAdjointU;

  if (flip) {
    // a = a.transpose();
    a_pinv = a_pinv.transpose();
  }

  return a_pinv;
}

bool move3d_start_and_load_p3d_file(std::string filename) {
  std::string home;
  home = getenv("HOME_MOVE3D");
  if ("" == home) {
    std::cout << "Error : HOME_MOVE3D not define" << std::endl;
    return false;
  }
  cout << "home : " << home << endl;

  // Load environment from file
  p3d_col_set_mode(p3d_col_mode_none);
  p3d_read_desc((home + filename).c_str());

  // Set collision checker
  p3d_col_set_mode(p3d_col_mode_pqp);
  p3d_col_start(p3d_col_mode_pqp);
  set_collision_by_object(TRUE);

  // Set c++ api function
  move3d_set_classic_libmove3d_api();

  return true;
}

bool move3d_start_and_load_manipulator() {
  return move3d_start_and_load_p3d_file(
      "/../assets/Manipulator/manipulateur.p3d");
}

Move3D::Robot* move3d_start_all_p3d_environment(std::string filename) {
  if (!move3d_start_and_load_p3d_file(filename)) {
    return NULL;
  }
  std::string robot_name;
  robot_name = XYZ_ENV->active_robot->name;  // XYZ_ENV->robot[0]->name;
  cout << "-- Robot name is : " << robot_name << endl;

  // Make c++ project interface
  Move3D::global_Project = new Move3D::Project(new Move3D::Scene(XYZ_ENV));
  Move3D::global_Project->getActiveScene()->setActiveRobot(robot_name);
  Move3D::Robot* robot =
      Move3D::global_Project->getActiveScene()->getActiveRobot();

  // GET QT PARAMETERS (used to be in project)
  initPlannerParameters();
  initGestureParameters();
  initHricsParameters();

  return robot;
}
