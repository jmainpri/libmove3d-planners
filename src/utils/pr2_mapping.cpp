#include "pr2_mapping.hpp"
#include "misc_functions.hpp"

//    torso_lift_joint		Torso
//    head_pan_joint			pan_cam
//    head_tilt_joint			tilt_cam
//    laser_tilt_mount_joint		laser-jnt
//    r_shoulder_pan_joint		right-Arm1
//    r_shoulder_lift_joint		right-Arm2
//    r_upper_arm_roll_joint		right-Arm3
//    r_elbow_flex_joint		right-Arm4
//    r_forearm_roll_joint		right-Arm5
//    r_wrist_flex_joint		right-Arm6
//    r_wrist_roll_joint		right-Arm7
//    r_gripper_joint			fingerJointGripper_0
//    l_shoulder_pan_joint		left-Arm1
//    l_shoulder_lift_joint		left-Arm2
//    l_upper_arm_roll_joint		left-Arm3
//    l_elbow_flex_joint		left-Arm4
//    l_forearm_roll_joint		left-Arm5
//    l_wrist_flex_joint		left-Arm6
//    l_wrist_roll_joint		left-Arm7
//    l_gripper_joint			fingerJointGripper_1

pr2_mapping::pr2_mapping(Move3D::Robot* robot)
{
    name_map_["torso_lift_joint"]         = "Torso";
    name_map_["head_pan_joint"]           = "pan_cam";
    name_map_["head_tilt_joint"]		  = "tilt_cam";
    name_map_["laser_tilt_mount_joint"]   = "laser-jnt";
    name_map_["r_shoulder_pan_joint"]	  = "right-Arm1";
    name_map_["r_shoulder_lift_joint"]	  = "right-Arm2";
    name_map_["r_upper_arm_roll_joint"]   = "right-Arm3";
    name_map_["r_elbow_flex_joint"]       = "right-Arm4";
    name_map_["r_forearm_roll_joint"]	  = "right-Arm5";
    name_map_["r_wrist_flex_joint"]       = "right-Arm6";
    name_map_["r_wrist_roll_joint"]       = "right-Arm7";
    name_map_["r_gripper_joint"]		  = "fingerJointGripper_0";
    name_map_["l_shoulder_pan_joint"]	  = "left-Arm1";
    name_map_["l_shoulder_lift_joint"]	  = "left-Arm2";
    name_map_["l_upper_arm_roll_joint"]   = "left-Arm3";
    name_map_["l_elbow_flex_joint"]       = "left-Arm4";
    name_map_["l_forearm_roll_joint"]	  = "left-Arm5";
    name_map_["l_wrist_flex_joint"]       = "left-Arm6";
    name_map_["l_wrist_roll_joint"]       = "left-Arm7";
    name_map_["l_gripper_joint"]		  = "fingerJointGripper_1";

    pr2_map_["fl_caster_rotation_joint"] = 0;
    pr2_map_["fl_caster_l_wheel_joint"] = 1;
    pr2_map_["fl_caster_r_wheel_joint"] = 2;
    pr2_map_["fr_caster_rotation_joint"] = 3;
    pr2_map_["fr_caster_l_wheel_joint"] = 4;
    pr2_map_["fr_caster_r_wheel_joint"] = 5;
    pr2_map_["bl_caster_rotation_joint"] = 6;
    pr2_map_["bl_caster_l_wheel_joint"] = 7;
    pr2_map_["bl_caster_r_wheel_joint"] = 8;
    pr2_map_["br_caster_rotation_joint"] = 9;
    pr2_map_["br_caster_l_wheel_joint"] = 10;
    pr2_map_["br_caster_r_wheel_joint"] = 11;
    pr2_map_["torso_lift_joint"] = 12;
    pr2_map_["torso_lift_motor_screw_joint"] = 13;
    pr2_map_["head_pan_joint"] = 14;
    pr2_map_["head_tilt_joint"] = 15;
    pr2_map_["laser_tilt_mount_joint"] = 16;
    pr2_map_["r_upper_arm_roll_joint"] = 17;
    pr2_map_["r_shoulder_pan_joint"] = 18;
    pr2_map_["r_shoulder_lift_joint"] = 19;
    pr2_map_["r_forearm_roll_joint"] = 20;
    pr2_map_["r_elbow_flex_joint"] = 21;
    pr2_map_["r_wrist_flex_joint"] = 22;
    pr2_map_["r_wrist_roll_joint"] = 23;
    pr2_map_["r_gripper_joint"] = 24;
    pr2_map_["r_gripper_l_finger_joint"] = 25;
    pr2_map_["r_gripper_r_finger_joint"] = 26;
    pr2_map_["r_gripper_r_finger_tip_joint"] = 27;
    pr2_map_["r_gripper_l_finger_tip_joint"] = 28;
    pr2_map_["r_gripper_motor_screw_joint"] = 29;
    pr2_map_["r_gripper_motor_slider_joint"] = 30;
    pr2_map_["l_upper_arm_roll_joint"] = 31;
    pr2_map_["l_shoulder_pan_joint"] = 32;
    pr2_map_["l_shoulder_lift_joint"] = 33;
    pr2_map_["l_forearm_roll_joint"] = 34;
    pr2_map_["l_elbow_flex_joint"] = 35;
    pr2_map_["l_wrist_flex_joint"] = 36;
    pr2_map_["l_wrist_roll_joint"] = 37;
    pr2_map_["l_gripper_joint"] = 38;
    pr2_map_["l_gripper_l_finger_joint"] = 39;
    pr2_map_["l_gripper_r_finger_joint"] = 40;
    pr2_map_["l_gripper_r_finger_tip_joint"] = 41;
    pr2_map_["l_gripper_l_finger_tip_joint"] = 42;
    pr2_map_["l_gripper_motor_screw_joint"] = 43;
    pr2_map_["l_gripper_motor_slider_joint"] = 44;
}

Move3D::Trajectory pr2_mapping::load_trajectory(std::string filename)
{
    Eigen::MatrixXd traj = move3d_load_matrix_from_csv_file( filename );

    std::vector<int> indices_matrix;
    std::vector<int> indices_move3d;

    // Get joint indicies for the matrix and the move3d robot
    for( std::map<std::string,std::string>::iterator it_map=name_map_.begin();
         it_map!=name_map_.end(); it_map++ )
    {
        indices_matrix.push_back( pr2_map_[it_map->first] + 1 ); // Add one to account for dts (at index 0)
        indices_move3d.push_back( robot_->getJoint( it_map->first )->getIndexOfFirstDof() );
    }

    std::vector<double> dts( traj.rows() );
    Eigen::MatrixXd traj_active( traj.rows(), indices_matrix.size() );

    for( int i=0; i<traj.rows(); i++ )
    {
        dts[i] = traj(i,0); // Get dts

        for( int j=0; j<int(indices_matrix.size()); j++ )
        {
            traj_active(i,j) = traj(i,indices_matrix[j]); // Get active dofs
        }
    }

    Move3D::Trajectory pr2_traj( robot_ );
    pr2_traj.setFromEigenMatrix( traj_active, indices_move3d );
    pr2_traj.setUseTimeParameter( true );
    pr2_traj.setDeltaTimes( dts );

    return pr2_traj;
}
