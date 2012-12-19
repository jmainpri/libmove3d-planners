#include "cspace.hpp"
#include "localpath.hpp"
//#include "riehl_localpaths.hpp" // PathSegments
//#include "mechanical_cost_functions.hpp" // 2d costmap cost function
//#include "mltrrt_cost_based_selection.hpp" // clearance cost
#include "cost_space.hpp"
#include <cmath>

#include <boost/math/special_functions/factorials.hpp>

double CSpace::traj_cost(p3d_traj* traj)
{
    double cost(0.);
    p3d_localpath *lp;
    if (traj)
    {
        lp = traj->courbePt;
        while (lp)
        {
            LocalPath path(m_robot, lp);
            cost += this->lp_cost(path.getBegin(), path.getEnd());
            lp = lp->next_lp;
        }
    }
    return(cost);
}

CSpaceCostMap2D::CSpaceCostMap2D()
{
    m_connection_radius_flag = true;
}

CSpaceCostMap2D::~CSpaceCostMap2D()
{
}

double CSpaceCostMap2D::q_cost(confPtr_t q)
{
    return(computeIntersectionWithGround(*q));
}

double CSpaceCostMap2D::lp_cost(confPtr_t q1, confPtr_t q2)
{
    LocalPath p(q1, q2);
//    PathSegments segments(p.getParamMax(), m_cost_step);
//    double cost(0.);
//    double cost_q_prev(this->q_cost(q1));
//    //printf("path length, step : %f, %f\n", p.getParamMax(), m_cost_step);
//    for(unsigned i(1); i < segments.nbPoints(); ++i)
//    {
//        double cost_q_next(this->q_cost(p.configAtParam(segments[i])));
//        //cost += std::max(cost_q_prev, cost_q_next) * segments.step();
//        cost += std::max(cost_q_next - cost_q_prev, 0.0) + segments.step() * 0.01;
//        //printf("cost: %f\n", cost);
//        cost_q_prev = cost_q_next;
//    }
    int nb_test=0;
    return global_costSpace->cost(p,nb_test);
}

double CSpaceCostMap2D::volume()
{
    assert(m_c_robot);
    // this means that there is 1 joint + the unused(legacy) joint 0
    assert(m_c_robot->njoints == 1);
    assert(m_c_robot->joints[1]->type == P3D_PLAN);
    return(fabs((m_c_robot->joints[1]->dof_data[0].vmax - m_c_robot->joints[1]->dof_data[0].vmin) * 
                (m_c_robot->joints[1]->dof_data[1].vmax - m_c_robot->joints[1]->dof_data[1].vmin)));
}

double CSpaceCostMap2D::unit_sphere()
{
    return(M_PI);
}

unsigned CSpaceCostMap2D::dimension()
{
    return(2);
}

//-----------------------------------------------------------
//-----------------------------------------------------------
Pr2CSpace::Pr2CSpace()
{
  m_connection_radius_flag = true;
}

Pr2CSpace::~Pr2CSpace()
{
}

double Pr2CSpace::q_cost(confPtr_t q)
{
  return(global_costSpace->cost(*q));
}

double Pr2CSpace::lp_cost(confPtr_t q1, confPtr_t q2)
{
  LocalPath p(q1, q2);
  int nb_test;
  return global_costSpace->cost(p,nb_test);
}

double Pr2CSpace::volume()
{
  assert(m_c_robot);
//  jnt->getName() : right-Arm1(6) , index_dof : 16
//  jnt->getName() : right-Arm2(7) , index_dof : 17
//  jnt->getName() : right-Arm3(8) , index_dof : 18
//  jnt->getName() : right-Arm4(9) , index_dof : 19
//  jnt->getName() : right-Arm5(10) , index_dof : 20
//  jnt->getName() : right-Arm6(11) , index_dof : 21
//  jnt->getName() : right-Arm7(12) , index_dof : 22

  double volume=1.0;
  
  for( int i=6; i<=12; i++) 
  {
    p3d_jnt* jntPt = m_c_robot->joints[i];
    volume *= jntPt->dist*fabs(jntPt->dof_data[0].vmax - jntPt->dof_data[0].vmin);
  }
  
  return volume;
}

double Pr2CSpace::unit_sphere()
{
  double n = 7;
  return pow(2,(n+1)/2)*pow(M_PI,(n-1)/2)/boost::math::double_factorial<double>(7);
}

unsigned Pr2CSpace::dimension()
{
  return(7);
}

//-------------------------------------------------------------
//-------------------------------------------------------------
GenericCSpace::GenericCSpace(path_cost_mode mode) :
    m_mode(mode)
{
    m_connection_radius_flag = false;
}

GenericCSpace::~GenericCSpace()
{
}

double GenericCSpace::q_cost(confPtr_t q)
{
    return(global_costSpace->cost(*q));
}

double GenericCSpace::lp_cost(confPtr_t q1, confPtr_t q2)
{
    LocalPath p(q1, q2);
//    PathSegments segments(p.getParamMax(), m_cost_step);
//    double cost(0.);
//    double cost_q_prev(this->q_cost(q1));
//    switch(m_mode)
//    {
//    case integral:
//    {
//        for(unsigned i(1); i < segments.nbPoints(); ++i)
//        {
//            double cost_q_next(this->q_cost(p.configAtParam(segments[i])));
//            cost += (cost_q_next + cost_q_prev) * 0.5 * segments.step();
//            cost_q_prev = cost_q_next;
//        }
//    }
//    case work:
//    {
//        for(unsigned i(1); i < segments.nbPoints(); ++i)
//        {
//            double cost_q_next(this->q_cost(p.configAtParam(segments[i])));
//            cost += std::max(cost_q_next - cost_q_prev, 0.0) + 0.01 * segments.step();
//            cost_q_prev = cost_q_next;
//        }
//    }
//    default: {}
//    }
    int nb_test;
    return global_costSpace->cost(p,nb_test);
}
