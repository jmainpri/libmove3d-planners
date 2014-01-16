#ifndef MISC_FUNCTIONS_HPP
#define MISC_FUNCTIONS_HPP

#include <string>
#include <Eigen/Core>

void move3d_save_matrix_to_file( const Eigen::MatrixXd& mat, std::string filename );

#endif // MISC_FUNCTIONS_HPP
