#ifndef CSPACE_INCLUDED_HPP
#define CSPACE_INCLUDED_HPP

#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"

#include <libmove3d/include/P3d-pkg.h>

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

    void set_robot(Robot* robot) { m_robot = robot; m_c_robot = robot->getRobotStruct(); }

    Robot* get_robot() { return(m_robot); }

    virtual double q_cost(confPtr_t q) = 0;

    virtual double lp_cost(confPtr_t q1, confPtr_t q2) = 0;

    virtual double traj_cost(p3d_traj* traj);
    
    bool get_connection_radius_flag() { return(m_connection_radius_flag); }

    void set_connection_radius_flag(bool flag) { m_connection_radius_flag = flag; }

    virtual double volume() = 0;

    virtual double unit_sphere() = 0;

    virtual unsigned dimension() = 0;

protected:
    double m_step;
    double m_cost_step;
    Robot* m_robot;
    p3d_rob* m_c_robot;
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

class Pr2CSpace : public CSpace
{
public:
  Pr2CSpace();
  ~Pr2CSpace();
  double q_cost(confPtr_t q);
  double lp_cost(confPtr_t q1, confPtr_t q2);
  
  // Compute the volume of the cspace :
  // i.e. the area of the 2d costmap.
  double volume();
  double unit_sphere();
  unsigned dimension();
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
