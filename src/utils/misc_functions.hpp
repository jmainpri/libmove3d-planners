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
#ifndef MISC_FUNCTIONS_HPP
#define MISC_FUNCTIONS_HPP

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "API/Device/robot.hpp"

//! Save vector of string to file
void save_strings_to_file(std::vector<std::string> strings,
                          std::string filename);

//! Load string from file
std::vector<std::string> load_strings_from_file(std::string filename);

//! Save Eigen matrix to file (can be loaded with matlab)
void move3d_save_matrix_to_file(const Eigen::MatrixXd& mat,
                                std::string filename);

void move3d_save_matrix_to_csv_file(const Eigen::MatrixXd& mat,
                                    std::string filename);

//! Save scenario to file
std::vector<Move3D::confPtr_t> move3d_load_context_from_csv_file(
    std::string filename);

void move3d_save_context_to_csv_file(
    const std::vector<Move3D::confPtr_t>& context, std::string filename);

//! Get eigen vector from vector of strings
//! representing float numbers
Eigen::VectorXd moved_get_vector(const std::vector<std::string>& config);

//! Returns the files in the folder with argument extension
std::vector<std::string> move3d_get_files_in_folder(std::string foldername,
                                                    std::string extention,
                                                    int nb_max_files = -1);

//! Retunrs the folders in the folder given as argument
std::vector<std::string> move3d_get_folders_in_folder(std::string foldername);

//! Load matrix from file
Eigen::MatrixXd move3d_load_matrix_from_csv_file(std::string filename);

template <class Matrix>
void matrix_write_binary(std::string filename, const Matrix& matrix) {
  std::ofstream out(filename.c_str(),
                    std::ios::out | std::ios::binary | std::ios::trunc);
  typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
  out.write((char*)(&rows), sizeof(typename Matrix::Index));
  out.write((char*)(&cols), sizeof(typename Matrix::Index));
  out.write((char*)matrix.data(),
            rows * cols * sizeof(typename Matrix::Scalar));
  out.close();
}

template <class Matrix>
void matrix_read_binary(std::string filename, Matrix& matrix) {
  std::ifstream in(filename.c_str(), std::ios::in | std::ios::binary);
  typename Matrix::Index rows = 0, cols = 0;
  in.read((char*)(&rows), sizeof(typename Matrix::Index));
  in.read((char*)(&cols), sizeof(typename Matrix::Index));
  matrix.resize(rows, cols);
  in.read((char*)matrix.data(), rows * cols * sizeof(typename Matrix::Scalar));
  in.close();
}

//! Test wether a matrix has nan
template <typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x) {
  return ((x - x).array() == (x - x).array()).all();
}

template <typename Derived>
inline bool is_nan(const Eigen::MatrixBase<Derived>& x) {
  return ((x.array() == x.array())).all();
}
//! Get a random interger
int move3d_random_integer(int min, int max);

//! Change number basis
std::vector<int> move3d_change_basis(int number, int basis);

//! Linear interpolation between two vectors
Eigen::VectorXd move3d_lerp(const Eigen::VectorXd& v0,
                            const Eigen::VectorXd& v1,
                            double t);

//! Matrix pseudo inverse
Eigen::MatrixXd move3d_pinv(const Eigen::MatrixXd& b, double rcond);

//! this function makes sure the curve in param
//! is continus for compting sum of acceleration
//! on circular joints
void move3d_smooth_circular_parameters(Eigen::VectorXd& params);

//! Print the joint mapping of a given robot
void print_joint_mapping(Move3D::Robot* robot);

//! Print joint achors
void print_joint_anchors(Move3D::Robot* robot);

//! Start move3d and load file as referenced from HOME_MOVE3D
bool move3d_start_and_load_p3d_file(std::string filename);

//! Start move3d and load manipulator
bool move3d_start_and_load_manipulator();

//! Start environment with C++ interface
Move3D::Robot* move3d_start_all_p3d_environment(std::string filename);

#endif  // MISC_FUNCTIONS_HPP
