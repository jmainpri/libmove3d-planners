/*
 * HRITaskSpaceCost.cpp
 *
 *  Created on: Sep 8, 2009
 *      Author: jmainpri
 */


#include "HRICS_HAMP.hpp"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

HriSpaceCost::HriSpaceCost(p3d_rob* rob,int jnt) :
        _JntId(jnt),
        _test(0)
{
    _Robot = new Robot(rob);
    _Bitmap = initialize();
}

HriSpaceCost::~HriSpaceCost()
{
    delete _Robot;
    // WARNING Implement
}

hri_bitmapset* HriSpaceCost::initialize()
{
    if(INTERPOINT != NULL)
    {
        hri_bt_destroy_bitmapset(INTERPOINT);
    }

    INTERPOINT = hri_exp_init();
    HRI_GIK = hri_gik_create_gik();

    return INTERPOINT;
}

vector<int> HriSpaceCost::getTaskPosition()
{

    vector<int> pos1(3);

    Vector3d pos2 = _Robot->getJointPos(_JntId);


    pos1[0] = (int)((pos2[0]-_Bitmap->realx)/_Bitmap->pace);
    pos1[1] = (int)((pos2[1]-_Bitmap->realy)/_Bitmap->pace);
    pos1[2] = (int)((pos2[2]-_Bitmap->realz)/_Bitmap->pace);

    //	cout << "Task is at :"
    //				<< " ( " << pos2[0]
    //				<< " , " << pos2[1]
    //				<< " , " << pos2[2] << " ) " << endl;
    //
    //	cout << "Task is at :"
    //			<< " ( " << pos1[0]
    //			<< " , " << pos1[1]
    //			<< " , " << pos1[2] << " ) " << endl;

    return pos1;
}

void HriSpaceCost::changeTask(int idJnt)
{
    _JntId = idJnt;
}

int HriSpaceCost::getTask()
{
    return _JntId;
}

int HriSpaceCost::test()
{
    return _test;
}

void HriSpaceCost::changeTest(int i)
{
    _test = i;
}

double HriSpaceCost::distanceCost()
{
    pos = getTaskPosition();

    double cost = hri_exp_distance_val(_Bitmap,
                                       pos.at(0),
                                       pos.at(1),
                                       pos.at(2));

    cout << "Cost is : " << cost << endl;

    return ENV.getDouble(Env::Kdistance)*cost;
}

double HriSpaceCost::visibilityCost()
{
    pos = getTaskPosition();

    double cost = hri_exp_vision_val(_Bitmap,
                                     pos.at(0),
                                     pos.at(1),
                                     pos.at(2));

    return ENV.getDouble(Env::Kvisibility)*cost;

    //	return 0;
    //	hri_exp_combined_val(hri_bitmapset* btset, int x, int y, int z)
}

double HriSpaceCost::combinedCost()
{

    pos = getTaskPosition();

    HRI_WEIGHTS[0] =  ENV.getDouble(Env::Kdistance);
    HRI_WEIGHTS[1] =  ENV.getDouble(Env::Kvisibility);

    return hri_exp_path_val(_Bitmap,
                            pos.at(0),
                            pos.at(1),
                            pos.at(2));

    /*    double visib = visibilityCost();
    double dista = distanceCost();

    if(ENV.getBool(Env::debugCostOptim))
    {
        cout << "Visibility Cost = " << visib << endl;
        cout << "Distance Cost = " << dista << endl;
    }
    return dista + visib;*/
}

double HriSpaceCost::comfortCost()
{
    pos = getTaskPosition();

    return hri_exp_hcomfort_val(_Bitmap,
                                pos.at(0),
                                pos.at(1),
                                pos.at(2));

}

double HriSpaceCost::switchCost()
{
    switch(_test)
    {
                case 0 :	return distanceCost(); break;
                case 1 :	return comfortCost() ; break;
                case 2 : 	return visibilityCost(); break;
                case 3 : 	return combinedCost(); break;
                default : 	cout << "No Cost" << endl;
            }
    return 0;
}

void HriSpaceCost::computeWorkspacePath()
{
    double qs[3], qf[3];
    hri_bitmap * bitmap;

    cout << "Computing Workspace Path for joint "<< _JntId << endl;

    shared_ptr<Configuration> q = _Robot->getCurrentPos();

    _Robot->setAndUpdate( *_Robot->getInitialPosition() );

    qs[0] = _Bitmap->robot->joints[_JntId]->abs_pos[0][3];
    qs[1] = _Bitmap->robot->joints[_JntId]->abs_pos[1][3];
    qs[2] = _Bitmap->robot->joints[_JntId]->abs_pos[2][3];

    _Robot->setAndUpdate( *_Robot->getGoTo() );

    qf[0] = _Bitmap->robot->joints[_JntId]->abs_pos[0][3];
    qf[1] = _Bitmap->robot->joints[_JntId]->abs_pos[1][3];
    qf[2] = _Bitmap->robot->joints[_JntId]->abs_pos[2][3];

    _Robot->setAndUpdate( *q );


    if(_Bitmap==NULL || _Bitmap->bitmap[BT_PATH]==NULL)
    {
        cout << "Trying to find a path in a non existing bitmap or bitmapset" << endl;
        return ;
    }

    bitmap = _Bitmap->bitmap[BT_PATH];

    bitmap->search_start = hri_bt_get_cell(bitmap,
                                           (int)((qs[0]-_Bitmap->realx)/_Bitmap->pace),
                                           (int)((qs[1]-_Bitmap->realy)/_Bitmap->pace),
                                           (int)((qs[2]-_Bitmap->realz)/_Bitmap->pace));

    bitmap->search_goal  = hri_bt_get_cell(bitmap,
                                           (int)((qf[0]-_Bitmap->realx)/_Bitmap->pace),
                                           (int)((qf[1]-_Bitmap->realy)/_Bitmap->pace),
                                           (int)((qf[2]-_Bitmap->realz)/_Bitmap->pace));

    if(bitmap->search_start == NULL)
    {
        cout << "Search start cell does not exist" << endl;
        PrintWarning(("Search start cell does not exist\n"));
        _Bitmap->pathexist = FALSE;
        return ;
    }

    if(bitmap->search_goal == NULL )
    {
        cout << "Search goal cell does not exist\n" << endl;
        _Bitmap->pathexist = FALSE;
        return ;
    }

    if( _Bitmap->pathexist )
    {
        hri_bt_reset_path(_Bitmap);
    }

    if( hri_bt_astar_bh(_Bitmap,bitmap) > -1 )
    {
        _Bitmap->pathexist = TRUE;
        bitmap->active = TRUE;
        g3d_draw_allwin_active();
        cout << "Success" << endl;
        return ;
    }
    else
    {
        _Bitmap->pathexist = FALSE;
        cout << "Fail" << endl;
        return ;
    }
}

void HriSpaceCost::computeHoleManipulationPath()
{
    cout << "Computing GIK Path for joint "<< _JntId << endl;

    if(_Bitmap!=NULL)
    {
        hri_bt_reset_path(_Bitmap);
    }
    if(BTGRAPH!=NULL)
    {
        p3d_del_graph(BTGRAPH);
        BTGRAPH = NULL;
    }

    if(_Bitmap->robot != _Robot->getRobotStruct() )
    {
        cout << "Error : _Bitmap->robot != _Robot->getRobotStruct()" << endl;
        return;
    }

    if( hri_exp_get_robot_joint_object() != _JntId )
    {
        cout << "Error : hri_exp_get_robot_object() = "<< hri_exp_get_robot_joint_object() << endl;
        cout << "Error : hri_exp_get_robot_object() != _JntId" << endl;
        return;
    }

    cout << "Looking for a 3D path for joint "<< hri_exp_get_robot_joint_object() << endl;

    _Robot->setAndUpdate( *_Robot->getInitialPosition() );

    hri_exp_set_exp_from_config( _Bitmap, _Robot->getGoTo()->getConfigStruct() );

    hri_gik_set_visstep(300);

    if ( hri_exp_find_manip_path(_Bitmap) )
    {
        p3d_rob* robotPt = (p3d_rob * )p3d_get_desc_curid(P3D_ROBOT);
        p3d_sel_desc_name(P3D_ROBOT, _Bitmap->robot->name );
        ENV.setBool(Env::drawGraph,true);
        p3d_graph_to_traj(_Bitmap->robot);
        g3d_add_traj((char*)"Globalsearch",p3d_get_desc_number(P3D_TRAJ));
        p3d_sel_desc_name(P3D_ROBOT,robotPt->name);
        ENV.setBool(Env::drawTraj,true);
        cout << "Traj added" << endl;
    }
    else
    {
        cout << "Error : computeHoleManipulationPath " << endl;
    }
}

void HriSpaceCost::computingAStarOnGraph()
{

}
