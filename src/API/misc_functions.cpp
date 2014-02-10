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

std::vector<std::string>  move3d_get_files_in_folder( std::string foldername, std::string extension )
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
    }
    pclose(fp);
    return files;
}
