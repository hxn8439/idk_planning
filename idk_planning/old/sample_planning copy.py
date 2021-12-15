#!/usr/bin/env python

import sys
import copy
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
import moveit_commander
from math import radians, degrees

# forward kinematics 
known_fk = {
    'default_home': [0.0, radians(-90.0), 0.0, radians(-90.0), 0.0, 0.0],
    'camera_home': [0.0, radians(-90.0), radians(45.0), radians(-45.0), radians(-90.0), 0.0],
    # 'near_pick': [radians(-15.51), radians(-78.91), radians(121.48), radians(-131.26), radians(-88.28), radians(-5.79)],
    # 'near_place': [radians(-12.11), radians(-63.03), radians(95.05), radians(-121.38), radians(-88.27), radians(-5.85)]
    'near_pick': [radians(7.74), radians(-76.00), radians(116.96), radians(-131.28), radians(-89.94), radians(-0.02)],
    'near_place': [radians(5.67), radians(-60.45), radians(91.93), radians(-121.86), radians(-89.94), radians(-0.02)]
}

camera_tube_offset = 4 # measured in cm

def go_to_known_locations(group, name):
    joint_goal = move_group.get_current_joint_values()
    joint_goal[:] = known_fk[name]
    
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()
    
def offset_xyz(group, x, y, z):
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x += x
    wpose.position.y += y
    wpose.position.z += z
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, _) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    move_group.execute(plan, wait=True)
    
    # return plan, fraction
    
# def clear_planning_scene(scene):
#     scene.robot_state.attached_collision_objects.clear()
#     scene.world.collision_objects.clear()

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    
    robot = moveit_commander.RobotCommander()
    
    scene = moveit_commander.PlanningSceneInterface()
    
    group_name = "manipulator"
    
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                                 moveit_msgs.msg.DisplayTrajectory,
    #                                                 queue_size=20)
    
    # rospy.sleep(2)
    
    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = robot.get_planning_frame()
    # box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.x = 0.37
    # box_pose.pose.position.y = 0 
    # box_pose.pose.position.z = -0.0165
    # box_name = "box"
    # scene.add_box(box_name, box_pose, size=(1.43, 0.56, 0.01))
    
    # go_to_known_locations(move_group, 'camera_home')
    # plan, _ = offset_xyz(move_group, 0.1, -0.1, -0.1)
    
    offset = [0.0385, 0.0351, 0.0388, 0.0384, 0.0375, 0.0370, 0.0391, 0.0378, 0.0375, 0.0]
    
    go_to_known_locations(move_group, 'camera_home')
    rospy.sleep(1)
    go_to_known_locations(move_group, 'near_pick')
    rospy.sleep(1)
    for ofs in offset:
        # down
        offset_xyz(move_group, 0, 0, -0.01)
        rospy.sleep(2)
        # up
        offset_xyz(move_group, 0, 0, 0.02)
        rospy.sleep(1)
        # to the board
        offset_xyz(move_group, 0.15, 0, 0)
        rospy.sleep(1)
        # down
        offset_xyz(move_group, 0, 0, -0.01)
        rospy.sleep(2)
        # up 
        offset_xyz(move_group, 0, 0, 0.01)
        rospy.sleep(1)
        # diagonal
        offset_xyz(move_group, -0.15, -ofs, 0)
        rospy.sleep(1)
        # down
        offset_xyz(move_group, 0, 0, -0.01)
        rospy.sleep(1)
    go_to_known_locations(move_group, 'camera_home')
    
    # move_group.execute(plan, wait=True)

    # offset_xyz(move_group, -0.1, 0.1, 0.1)
    # raw_input("Press Enter to continue...")
    
    # while not rospy.is_shutdown():
    #     go_to_known_locations(move_group, 'default_home')
    #     raw_input("Press Enter to continue...")
    #     go_to_known_locations(move_group, 'camera_home')
    #     raw_input("Press Enter to continue...")