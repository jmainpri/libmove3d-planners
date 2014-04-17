#include "misc_functions.hpp"

#include <iostream>
#include <fstream>

// For random number generator (seed can be passed as argument)
#include <libmove3d/include/P3d-pkg.h>

using std::cout;
using std::endl;

void move3d_save_matrix_to_file( const Eigen::MatrixXd& mat, std::string filename )
{
     cout << "save matrix to : " << filename << endl;
    std::ofstream file( filename.c_str() );
    if (file.is_open())
        file << mat << '\n';
//        for( int i=0;i<mat.rows();i++){
//            for( int j=0;j<mat.cols();j++)
//                file << mat(i,j) << " ";
//            file << endl;
//        }
    file.close();
}

std::vector<std::string>  move3d_get_files_in_folder( std::string foldername, std::string extension, int nb_max_files )
{
    bool quiet = true;
    std::vector<std::string> files;

    std::string command = "ls " + foldername;
    FILE* fp = popen( command.c_str(), "r");
    if (fp == NULL) {
        cout << "ERROR in system call" << endl;
        return files;
    }

    char str[PATH_MAX];
    while ( fgets( str, PATH_MAX, fp) != NULL )
    {
        std::string filename( str );
        filename = filename.substr(0, filename.size()-1);
        std::string file_extension( filename.substr( filename.find_last_of(".") + 1 ) );
        if( extension == file_extension )
        {
            if( !quiet ) {
                cout << "add : " << filename << endl;
            }
            files.push_back( filename );
        }

        if( (nb_max_files > 0) && (int(files.size()) >= nb_max_files) ) {
            break;
        }
    }
    pclose(fp);
    return files;
}

double move3d_random_integer( int min, int max )
{
    return p3d_random_integer( min, max );
}
