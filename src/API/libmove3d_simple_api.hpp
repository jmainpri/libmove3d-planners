#ifndef LIBMOVE3D_SIMPLE_API_HPP
#define LIBMOVE3D_SIMPLE_API_HPP

#include "API/Device/robot.hpp"

double* move3d_configuration_simple_constructor(Move3D::Robot* robot);
void move3d_set_api_functions_configuration_simple();
void move3d_set_api_functions_localpath_simple();
void move3d_set_api_functions( bool use_move3d_fct );
bool move3d_use_api_functions();

#endif // LIBMOVE3D_SIMPLE_API_HPP