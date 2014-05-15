#ifndef MISC_FUNCTIONS_HPP
#define MISC_FUNCTIONS_HPP

#include <string>
#include <vector>
#include <Eigen/Core>

//! Save Eigen matrix to file (can be loaded with matlab)
void move3d_save_matrix_to_file( const Eigen::MatrixXd& mat, std::string filename );

//! Returns the files in the folder with argumen extension
std::vector<std::string>  move3d_get_files_in_folder( std::string foldername, std::string extention, int nb_max_files=-1 );

double move3d_random_integer( int min, int max );

std::vector<int> move3d_change_basis( int number , int basis );

Eigen::VectorXd move3d_lerp( const Eigen::VectorXd& v0, const Eigen::VectorXd& v1, double t );

#endif // MISC_FUNCTIONS_HPP
