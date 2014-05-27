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
#ifndef CSPACE_INCLUDED_HPP
#define CSPACE_INCLUDED_HPP

#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"

namespace Move3D
{

class CSpace
{
public:
    // IMPORTANT : to properly initialize a CSpace object, the following must be done :
    // create the CSpace object,
    // then in no particular order :
    // call the set_step function,
    // call the set_robot function.
    CSpace() : m_cost_step(0)
    {};

    virtual ~CSpace() {};

    void set_cost_step(const double& cost_step) { m_cost_step = cost_step; }

    void set_step(const double& step) { m_step = step; }

    double get_step() { return(m_step); }

    void set_robot(Robot* robot) { m_robot = robot; /* m_c_robot = robot->getP3dRobotStruct(); */ }

    Robot* get_robot() { return(m_robot); }

    virtual double q_cost(confPtr_t q) = 0;

    virtual double lp_cost(confPtr_t q1, confPtr_t q2) = 0;

    // virtual double traj_cost(p3d_traj* traj);
    
    bool get_connection_radius_flag() { return(m_connection_radius_flag); }

    void set_connection_radius_flag(bool flag) { m_connection_radius_flag = flag; }

    virtual double volume() = 0;

    virtual double unit_sphere() = 0;

    virtual unsigned dimension() = 0;

protected:
    double m_step;
    double m_cost_step;
    Robot* m_robot;
    /* p3d_rob* m_c_robot; */
    bool m_connection_radius_flag;
};

class CSpaceCostMap2D : public CSpace
{
public:
    CSpaceCostMap2D();
  
  virtual ~CSpaceCostMap2D();
  virtual double q_cost(confPtr_t q);
  virtual double lp_cost(confPtr_t q1, confPtr_t q2);
  
  // Compute the volume of the cspace :
  // i.e. the area of the 2d costmap.
  virtual double volume();
  virtual double unit_sphere();
  virtual unsigned dimension();
};

class ArmCSpace : public CSpace
{
public:
  ArmCSpace( const std::vector<Joint*>& joints );
  ~ArmCSpace();
  double q_cost(confPtr_t q);
  double lp_cost(confPtr_t q1, confPtr_t q2);
  
  // Compute the volume of the cspace :
  // i.e. the area of the 2d costmap.
  double volume();
  double unit_sphere();
  unsigned dimension();

private:
  std::vector<Joint*> joints_;
};

class GenericCSpace : public CSpace
{
public:
    enum path_cost_mode { integral, work };

    GenericCSpace(path_cost_mode mode);

    virtual ~GenericCSpace();
    virtual double q_cost(confPtr_t q);
    virtual double lp_cost(confPtr_t q1, confPtr_t q2);

    // Compute the volume of the cspace :
    // the product of all dofs.
    virtual double volume() { return(0.0); };
    virtual double unit_sphere() { return(0.0); };
    virtual unsigned dimension() { return(0); };

private:
    path_cost_mode m_mode;
};

}

#endif // CSPACE_INCLUDED_HPP
