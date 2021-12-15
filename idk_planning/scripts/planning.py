#!/usr/bin/env python

from math import radians
from copy import deepcopy

# UR Service
from ur_msgs.srv import SetIORequest, SetIO

# moveit stuff
import moveit_commander

# ros stuff
import rospy
from geometry_msgs.msg import PoseStamped

class URPlanner:
    def __init__(self, autohome = False, speed = 1, accel = 1, group_name='manipulator'):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_max_velocity_scaling_factor(speed)
        self.move_group.set_max_acceleration_scaling_factor(accel)
        
        # define location
        self.locations = self.init_known_locations()
        
        # vacuum powering service
        rospy.wait_for_service('/ur_hardware_interface/set_io')
        self.io_srv = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        
        # autohome the robot
        if autohome:
            self.move_group.set_named_target('up')
            self.move_group.go(wait=True)
            self.go_to_known_location('zone2')

        rospy.sleep(2)
        self.init_planning_scene()
        rospy.sleep(2)
        
    def nozzle_to_camera(self, x_offset = 3.05, y_offset = 4.39):
        x_offset /= 100
        y_offset /= 100
        self.move_ik_xyz(-x_offset, y_offset, 0.0)
        
    def vacuum_on(self):
        req = SetIORequest()
        req.fun = 1
        req.pin = 0
        req.state = 1
        response = self.io_srv(req)
        return response
    
    def vacuum_off(self):
        req = SetIORequest()
        req.fun = 1
        req.pin = 0
        req.state = 0
        response = self.io_srv(req)
        return response
        
    def go_to_known_location(self, location_name):
        if location_name not in self.locations:
            return
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[:] = self.locations[location_name]
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        
    def move_cartesian_xyz(self, x, y, z, percent_capable = 0.95):
        self.move_group.set_start_state_to_current_state()
        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        wpose.position.x += x
        wpose.position.y += y
        wpose.position.z += z
        waypoints.append(deepcopy(wpose))
        plan, fraction = self.move_group.compute_cartesian_path(waypoints,  # waypoints to follow
                                                                0.001,      # eef_step
                                                                0.0)        # jump_threshold         
        if fraction > percent_capable:
            self.move_group.execute(plan, wait=True)
            
    def move_cartesian_xyz_interp(self, x, y, z, percent_capable = 0.9, interp=100):
        self.move_group.set_start_state_to_current_state()
        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        dx = self.delta(wpose.position.x, wpose.position.x + x, interp)
        dy = self.delta(wpose.position.y, wpose.position.y + y, interp)
        dz = self.delta(wpose.position.z, wpose.position.z + z, interp)
        for _ in range(interp):
            wpose.position.x += dx
            wpose.position.y += dy
            wpose.position.z += dz
            waypoints.append(deepcopy(wpose))
        plan, fraction = self.move_group.compute_cartesian_path(waypoints,  # waypoints to follow
                                                                0.001,      # eef_step
                                                                0.0)        # jump_threshold
                                                                 
        if fraction > percent_capable:
            self.move_group.execute(plan, wait=True)
    
    def delta(self, start, end, interp):
        return (end - start) / interp
    
    def move_ik_xyz(self, x, y, z):
        self.move_group.set_start_state_to_current_state()
        pose_goal = self.move_group.get_current_pose("wrist_3_link").pose
        pose_goal.position.x += x
        pose_goal.position.y += y
        pose_goal.position.z += z
        self.move_group.set_pose_target(deepcopy(pose_goal))
        _ = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
    def init_known_locations(self):
        locations = {}
        
        name = 'zone1'
        joint_angles = [radians(-1.86), radians(-90.39), radians(89.14), radians(-89.93), radians(-90.37), radians(-91.60)]
        locations[name] = joint_angles
        
        name = 'zone2'
        joint_angles = [radians(-33.95), radians(-86.36), radians(84.99), radians(-89.82), radians(-89.67), radians(-123.70)]
        locations[name] = joint_angles
        
        return locations
    
    def init_planning_scene(self):
        # clearing the scene
        self.scene.clear()
        
        # table platform
        table_thickness = 0.01
        table = PoseStamped()
        table.header.frame_id = self.robot.get_planning_frame()
        table.pose.orientation.w = 1.0
        table.pose.position.x = 1.4224 / 4
        table.pose.position.y = 0.0
        table.pose.position.z = -table_thickness / 2.0
        self.scene.add_box("table", table, size=(1.4224, 0.56, table_thickness))
        
        # side fences
        lfence = PoseStamped()
        lfence.header.frame_id = self.robot.get_planning_frame()
        lfence.pose.orientation.w = 1.0
        lfence.pose.position.x = 1.4224 / 4
        lfence.pose.position.y = 0.56 / 2 + 0.01 / 2
        lfence.pose.position.z = 0.25 / 4
        self.scene.add_box("lfence", lfence, size=(1.4224, 0.01, 0.25))
        
        rfence = PoseStamped()
        rfence.header.frame_id = self.robot.get_planning_frame()
        rfence.pose.orientation.w = 1.0
        rfence.pose.position.x = 1.4224 / 4
        rfence.pose.position.y = -(0.56 / 2 + 0.01 / 2)
        rfence.pose.position.z = 0.25 / 4
        self.scene.add_box("rfence", rfence, size=(1.4224, 0.01, 0.25))

        # workspace limit
        zcap = PoseStamped()
        zcap.header.frame_id = self.robot.get_planning_frame()
        zcap.pose.orientation.w = 1.0
        zcap.pose.position.x = 1.4224 / 4
        zcap.pose.position.y = 0.0
        zcap.pose.position.z = 0.8
        self.scene.add_box("z_fence", zcap, size=(1.4224, 0.56, 0.01))