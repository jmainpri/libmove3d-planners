#include "PRMStar.hpp"

// C++ Implementation: sPRM

#include "PRMStar.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/Device/robot.hpp"

#include "planner/planEnvironment.hpp"

#include <iostream>
#include <libmove3d/p3d/env.hpp>
#include <libmove3d/include/P3d-pkg.h>

using namespace Move3D;
using std::cout;
using std::endl;

MOVE3D_USING_SHARED_PTR_NAMESPACE

static bool print_lower_connect = false;

PRMStar::PRMStar( Robot* R, Graph* G) : sPRM(R,G)
{
    cout << __PRETTY_FUNCTION__ << endl;

    cspace_ = NULL;

    if( _Robot->getNumberOfJoints() == 2 && _Robot->getP3dRobotStruct()->joints[1]->type == P3D_PLAN )
    {
        cspace_ = new CSpaceCostMap2D();
        initCSpace();
        cout << "plannar CS space" << endl;
    }

    if( _Robot->getName() == "PR2_ROBOT" )
    {
        std::vector<Joint*> joints;
        for( int i=6;i<=12;i++) joints.push_back( _Robot->getJoint(i) );
        cspace_ = new ArmCSpace( joints );
        initCSpace();
    }
}

PRMStar::~PRMStar()
{

}

void PRMStar::initCSpace()
{
    cspace_->set_step( radius_ );
    cspace_->set_cost_step( radius_ );
    cspace_->set_robot( _Robot );
}

double PRMStar::computeRadius()
{
    if( cspace_ == NULL )
    {
        return radius_;
    }

    return rrgBallRadius();
}

double PRMStar::rrgBallRadius()
{
    if( cspace_->get_connection_radius_flag() )
    {
        double inv_d = 1.0 / cspace_->dimension();
        double gamma_rrg = 2 * pow(1.0 + inv_d, inv_d) * pow( cspace_->volume() / cspace_->unit_sphere(), inv_d);
        double nb_nodes = _Graph->getNumberOfNodes();

        double radius = gamma_rrg * pow((log(nb_nodes)/nb_nodes), inv_d);
        //double radius = std::min(cspace_->get_step(), radius );

        if( print_lower_connect )
            cout << "radius : " << radius << endl;

        return radius;
    }
    else
    {
        return cspace_->get_step();
    }
}
