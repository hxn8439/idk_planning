#!/usr/bin/env python

import sys
import copy
from copy import deepcopy
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
import cv2, cv_bridge
import moveit_commander
import numpy as np
from math import radians, degrees

from sensor_msgs.msg import Image

class ur5_vision:
    def __init__(self):
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.known_contours = []
        self.known_centers = []
        self.crosshair = (368, 319)
        cv2.namedWindow('Display')
        
    def increase_brightness(self, img, value=30):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        lim = 255 - value
        v[v > lim] = 255
        v[v <= lim] += value

        final_hsv = cv2.merge((h, s, v))
        img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return img

    def pixel_to_displacement(self, pixel):
        py, px = pixel
        y_offset = -1 * (py -  self.crosshair[0]) * (0.32 / 480)
        x_offset = -1 * (px -  self.crosshair[1]) * (0.41 / 640)
        z_offset = -0.29
        return x_offset, y_offset, z_offset

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        image_bright = self.increase_brightness(image, 255)
        
        # # image = cv2.medianBlur(image, 5)
        blur_image = cv2.GaussianBlur(image_bright,(5,5),0)
        blur_image = cv2.GaussianBlur(blur_image,(5,5),0)
        
        img_gray = cv2.cvtColor(blur_image, cv2.COLOR_BGR2GRAY)
        
        # _, thresh = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY)
        thresh = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        
        # edges = cv2.Canny(thresh, 100, 255)
        
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        area_limit = 250
        large_contours = [c for c in contours if cv2.contourArea(c) >= area_limit]
        self.known_contours = deepcopy(large_contours)
        self.known_centers = []
        
        for contour in self.known_contours:
            M = cv2.moments(contour)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 3, (0, 255, 0), -1)
            text = "y:" + str(cx) + ", x:" + str(cy)
            dist_text = "y:" + str(-1 * (cx -  self.crosshair[0])) + ", x:" + str(-1*(cy - self.crosshair[1]))
            cv2.putText(image, text, (cx + 10, cy - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.3333, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(image, dist_text, (cx + 10, cy - 30),cv2.FONT_HERSHEY_SIMPLEX, 0.3333, (0, 0, 255), 1, cv2.LINE_AA)
            self.known_centers.append((cx, cy))
            
        # cv2.line(self.crosshair[0]-20, self.crosshair[1], self.crosshair[0]+20, self.crosshair[1])
        # cv2.line(self.crosshair[0]/2, self.crosshair[1]-20, self.crosshair[0]/2, self.crosshair[1]+20)
        cv2.putText(image, "368, 319", (368 - 20, 319 + 15),cv2.FONT_HERSHEY_SIMPLEX, 0.3333, (0, 100, 255), 1, cv2.LINE_AA)
        cv2.circle(image, (368, 319) , 5, (0, 100, 255), thickness=1, lineType=8, shift=0)
        cv2.drawContours(image, large_contours, -1, (255, 255, 0), 1)
        cv2.imshow('Display', image)
        cv2.waitKey(1)

# forward kinematics 
known_fk = {
    'default_home': [0.0, radians(-90.0), 0.0, radians(-90.0), 0.0, 0.0],
    'camera_home': [radians(-17.72), radians(-95.26), radians(93.72), radians(-88.65), radians(-89.95), radians(-17.77)],
    # 'near_pick': [radians(-15.51), radians(-78.91), radians(121.48), radians(-131.26), radians(-88.28), radians(-5.79)],
    # 'near_place': [radians(-12.11), radians(-63.03), radians(95.05), radians(-121.38), radians(-88.27), radians(-5.85)]
    'zone1': [radians(-27.55), radians(-115.21), radians(108.02), radians(-83.11), radians(-89.91), radians(-27.68)],
    'zone2': [radians(-12.92), radians(-69.93), radians(64.87), radians(-85.08), radians(-89.98), radians(-13.00)],
    'near_pick': [radians(7.74), radians(-76.00), radians(116.96), radians(-131.28), radians(-89.94), radians(-0.02)],
    'near_place': [radians(5.67), radians(-60.45), radians(91.93), radians(-121.86), radians(-89.94), radians(-0.02)]
}

known_xyz = {
    'z1q1': [-0.0975, 0.24, -0.29],
    'z1q2': [-0.0975 - 0.0053, 0.24 - 0.4126, -0.29],
    'z1q3': [-0.0975 - 0.0053 + 0.32, 0.24 - 0.4126, -0.29]
}

def rotate_wrist(group, degrees):
    joint_goal = group.get_current_joint_values()
    joint_goal[-1] += radians(degrees)
    group.go(joint_goal, wait=True)
    group.stop()

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
    
    follower=ur5_vision()
    
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
    
    # go_to_known_locations(move_group, 'default_home')
    #go_to_known_locations(move_group, 'camera_home')
    
    # rospy.sleep(2)
    
    #go to zone 1, use camera to store all contours as contours_start
    # go_to_known_locations(move_group, 'zone1')
    # rospy.sleep(2)
    
    # for center in follower.known_centers:
    #     x, y, _ = follower.pixel_to_displacement(center)
    #     offset_xyz(move_group, x, y, 0)
    #     rospy.sleep(1)
    #     offset_xyz(move_group, 0, 0, -0.29)
    #     rospy.sleep(1)
    #     go_to_known_locations(move_group, 'zone1')
    #     rospy.sleep(2)
    
    rospy.spin()
    
    # print(f'Currently at pixel {(368, 319)}')
    # for center in follower.known_centers:
    #     print(f'Center at {}')
    # zone1_contours = deepcopy(follower.known_contours)
    # offset_xyz(move_group, *known_xyz['z1q3'])
    # offset_xyz(move_group, 0, -0.15, 0)
    # rotate_wrist(move_group, 90)


    # go_to_known_locations(move_group, 'zone2')
    #zone2_contours = deepcopy(follower.known_contours)
    
    # print(len(zone1_contours))
    # print(len(zone2_contours))
    
    #go to zone 2, use camera to store all contours as contours_finish
    # go_to_known_locations(move_group, 'zone2')
    
    # go_to_known_locations(move_group, 'zone1')
    # go_to_known_locations(move_group, 'zone2')
    
    # go_to_known_locations(move_group, 'zone1')
    # go_to_known_locations(move_group, 'zone2')
    
    # go_to_known_locations(move_group, 'zone1')
    # go_to_known_locations(move_group, 'zone2')    
    
    # go_to_known_locations(move_group, 'zone1')
    # go_to_known_locations(move_group, 'zone2')
    
    # go_to_known_locations(move_group, 'camera_home')
    

    # rospy.sleep(1)
    # go_to_known_locations(move_group, 'near_pick')
    # rospy.sleep(1)
    # for ofs in offset:
    #     # down
    #     offset_xyz(move_group, 0, 0, -0.01)
    #     rospy.sleep(2)
    #     # up
    #     offset_xyz(move_group, 0, 0, 0.02)
    #     rospy.sleep(1)
    #     # to the board
    #     offset_xyz(move_group, 0.15, 0, 0)
    #     rospy.sleep(1)
    #     # down
    #     offset_xyz(move_group, 0, 0, -0.01)
    #     rospy.sleep(2)
    #     # up 
    #     offset_xyz(move_group, 0, 0, 0.01)
    #     rospy.sleep(1)
    #     # diagonal
    #     offset_xyz(move_group, -0.15, -ofs, 0)
    #     rospy.sleep(1)
    #     # down
    #     offset_xyz(move_group, 0, 0, -0.01)
    #     rospy.sleep(1)
    # go_to_known_locations(move_group, 'camera_home')
    
    # move_group.execute(plan, wait=True)

    # offset_xyz(move_group, -0.1, 0.1, 0.1)
    # raw_input("Press Enter to continue...")
    
    # while not rospy.is_shutdown():
    #     go_to_known_locations(move_group, 'default_home')
    #     raw_input("Press Enter to continue...")
    #     go_to_known_locations(move_group, 'camera_home')
    #     raw_input("Press Enter to continue...")