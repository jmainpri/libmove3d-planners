#ifndef MISC_FUNCTIONS_HPP
#define MISC_FUNCTIONS_HPP

#include <string>
#include <vector>
#include <Eigen/Core>

//! Save Eigen matrix to file (can be loaded with matlab)
void move3d_save_matrix_to_file( const Eigen::MatrixXd& mat, std::string filename );

//! Returns the files in the folder with argumen extension
std::vector<std::string>  move3d_get_files_in_folder( std::string foldername, std::string extention );

#endif // MISC_FUNCTIONS_HPP
