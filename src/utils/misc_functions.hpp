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

#include "API/Device/robot.hpp"

//! Save Eigen matrix to file (can be loaded with matlab)
void move3d_save_matrix_to_file( const Eigen::MatrixXd& mat, std::string filename );
void move3d_save_matrix_to_csv_file( const Eigen::MatrixXd& mat, std::string filename );

//! Returns the files in the folder with argumen extension
std::vector<std::string>  move3d_get_files_in_folder( std::string foldername, std::string extention, int nb_max_files=-1 );

//! Load matrix from file
Eigen::MatrixXd move3d_load_matrix_from_csv_file( std::string filename );

//! Get a random interger
double move3d_random_integer( int min, int max );

//! Change number basis
std::vector<int> move3d_change_basis( int number , int basis );

//! Linear interpolation between two vectors
Eigen::VectorXd move3d_lerp( const Eigen::VectorXd& v0, const Eigen::VectorXd& v1, double t );

//! Print the joint mapping of a given robot
void print_joint_mapping( Move3D::Robot* robot );

//! Print joint achors
void print_joint_anchors( Move3D::Robot* robot );

#endif // MISC_FUNCTIONS_HPP
