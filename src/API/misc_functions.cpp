#include "misc_functions.hpp"

#include <iostream>
#include <fstream>

using std::cout;
using std::endl;

void move3d_save_matrix_to_file( const Eigen::MatrixXd& mat, std::string filename )
{
    cout << "save matrix to : " << filename << endl;
    std::ofstream file( filename.c_str() );
    if (file.is_open())
        file << mat << '\n';
    file.close();
}
