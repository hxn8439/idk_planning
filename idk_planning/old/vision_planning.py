#!/usr/bin/env python

from logging import debug
import pyrealsense2 as rs
import numpy as np
import cv2, cv_bridge
import rospy
import moveit_commander

import sys
from copy import deepcopy
from math import sqrt, radians

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridgeError

block_thickness = 8.8 / 1000

class realsense_camera:
    def __init__(self, debug=False, roi=0.85):
        # get intrinsics
        self.intrinsics = self.generate_intrinsics()
        self.nozzle_y_offset = 0
        self.nozzle_z_offset = 0.0976
        
        # debug flag
        self.debug = debug
        
        # define canvas
        self.frame_width  = self.intrinsics.width
        self.frame_height = self.intrinsics.height
        
        self.x_min = int((1 - roi) * self.frame_width)
        self.y_min = int((1 - roi) * self.frame_height)
        self.x_max = int(roi * self.frame_width)
        self.y_max = int(roi * self.frame_height)
        
        self.pixel_scale = 0.298 / (self.y_max - self.y_min)
        self.point_top_left = (self.x_min, self.y_min)
        self.point_bot_right = (self.x_max, self.y_max)
        
        # starting streams
        self.bridge = cv_bridge.CvBridge()
        self.depth_subscriber = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.handle_depth_image)
        self.color_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.handle_color_image)

        # camera center pixel
        self.center = (self.intrinsics.width/2, self.intrinsics.height/2)
        self.nozzle = (702, 472)
        
        # live updating frames        
        self.depth_frame = None
        self.color_frame = None
        
        # contours and centers
        self.contours = None
        self.object_centers = None
        
        # colors
        self.crosshair_color = (0, 255, 0)
        
        # wait for updates
        rospy.sleep(3)
    
    def place_text(self, image, text, location, color, scale = 0.3333):
        x, y = location
        x -= len(text) * 3
        y += 15
        font = cv2.FONT_HERSHEY_SIMPLEX
        return cv2.putText(image, text, (x, y), font, scale, color, 1, cv2.LINE_AA)
    
    def place_crosshair(self, image, location, color):
        return cv2.drawMarker(image, location, color, markerSize=10, thickness=1, line_type=cv2.LINE_AA)
    
    def generate_contours(self, brightup = 100, contour_area=250, display=False):
        image_bright = self.increase_image_brightness(deepcopy(self.color_frame), brightup)
        blur_image = cv2.GaussianBlur(image_bright,(5,5),0)
        blur_image = cv2.GaussianBlur(blur_image,(5,5),0)
        blur_image = cv2.GaussianBlur(blur_image,(5,5),0)
        blur_image = cv2.medianBlur(blur_image,5)
        img_gray = cv2.cvtColor(blur_image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
        large_contours = [c for c in contours if cv2.contourArea(c) >= contour_area]
        if display:
            # cnt = cv2.drawContours(deepcopy(self.color_frame), large_contours, -1, (255, 255, 0), 1)
            roi = cv2.rectangle(deepcopy(self.color_frame), self.point_top_left, self.point_bot_right, (255, 0, 255), 1, cv2.LINE_AA)
            # cv2.imshow('brighten-ed image', image_bright)
            # cv2.imshow('blurred image', blur_image)
            # cv2.imshow('grayscaled image', img_gray)
            # cv2.imshow('threshold image', thresh)
            cv2.imshow('region of interest (zone1)', roi)
            # cv2.imshow('visible contours', cnt)
        return deepcopy(large_contours)
    
    def generate_intrinsics(self):
        info = rospy.wait_for_message('/camera/aligned_depth_to_color/camera_info', CameraInfo, None)
        intrinsics = rs.intrinsics()
        intrinsics.width = info.width
        intrinsics.height = info.height
        intrinsics.ppx = info.K[2]
        intrinsics.ppy = info.K[5]
        intrinsics.fx = info.K[0]
        intrinsics.fy = info.K[4]
        if info.distortion_model == 'plumb_bob':
            intrinsics.model = rs.distortion.brown_conrady
        elif info.distortion_model == 'equidistant':
            intrinsics.model = rs.distortion.kannala_brandt4
        intrinsics.coeffs = [i for i in info.D]
        return intrinsics
        
    def generate_centers(self):
        centers = []
        for contour in self.contours:
            M = cv2.moments(contour)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            point = (cx, cy)
            if self.point_in_roi(point): 
                if self.point_in_list(point, centers) == False:
                    centers.append(point)
        return deepcopy(centers)
    
    def generate_trajectories(self):
        return [self.distance_to_pixel(center) for center in self.object_centers]
    
    # https://github.com/IntelRealSense/realsense-ros/issues/1342#issuecomment-681172015
    def distance_to_pixel(self, pixel):
        if self.depth_frame is not None:
            x, y = pixel
            depth = self.depth_frame[y, x]
            dx, dy, _ = rs.rs2_deproject_pixel_to_point(self.intrinsics, [x, y], depth)
            dx /= 1000
            dy /= 1000
            return round(dx, 3), -1 * round(dy, 3)
        return None, None
        
    def handle_depth_image(self, image_msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(image_msg, image_msg.encoding)
    
    def handle_color_image(self, color_msg):
        # store original image
        self.color_frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        
        # generate contours
        self.contours = self.generate_contours(display=self.debug)

        # generate centers of objects
        self.object_centers = self.generate_centers()
        
        # make copy of original to draw on
        output = deepcopy(self.color_frame)
        
        # placing crosshairs
        for center in self.object_centers:
            # dist = self.relative_dist_to_nozzle(center)
            # dx, dy = self.distance_to_pixel(center)
            self.place_crosshair(output, center, self.crosshair_color)
            self.place_text(output, str(center), center, self.crosshair_color)
            # self.place_text(output, str((dx, dy)), (center[0], center[1] + 12), (0, 0, 255))
            
        if self.debug:
            self.place_crosshair(output, self.nozzle, (255, 0, 255))
            self.place_crosshair(output, self.center, (255, 0, 255))
            self.place_text(output, str(self.nozzle), (self.nozzle[0], self.nozzle[1] - 25), (255, 0, 255))
            self.place_text(output, str(self.center), (self.center[0], self.center[1] - 25), (255, 0, 255))

        # output image
        cv2.imshow('processed object crosshair', output)
        cv2.waitKey(1)
        
    def generate_trajectories(self):
        return [self.distance_to_pixel(center) for center in deepcopy(self.object_centers)]
        
    def increase_image_brightness(self, image, value):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        lim = 255 - value
        v[v > lim] = 255
        v[v <= lim] += value

        final_hsv = cv2.merge((h, s, v))
        image = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return image
    
    def point_in_roi(self, point):
        px, py = point
        within_x = (px >= self.x_min and px <= self.x_max)
        within_y = (py >= self.y_min and py <= self.y_max)
        return (within_x and within_y)
    
    def point_in_list(self, point, point_list, limit=10):
        actual = np.array(point)
        for testing in point_list:
            testing = np.array(testing)
            distance = np.linalg.norm(actual - testing, ord=2) # euclidean distance
            if distance < limit:
                return True
        return False
            
class ur_moveit_planner:
    def __init__(self, group_name='manipulator', max_speed = 0.1, max_accel = 0.5):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        
        # settings
        self.move_group.set_max_velocity_scaling_factor(max_speed)
        self.move_group.set_max_acceleration_scaling_factor(max_accel)
        
        # generate known locations
        self.known_fk = self.generate_known_locations()
        
    def go_to_known_location(self, location_name):
        if location_name not in self.known_fk.keys():
            print('Location specified not known')
        else:
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[:] = self.known_fk[location_name]
            self.move_group.go(joint_goal, wait=True)
            self.move_group.stop()
            
    def move_xyz(self, x, y, z, percent_capable = 0.95):
        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        wpose.position.x += x
        wpose.position.y += y
        wpose.position.z += z
        waypoints.append(deepcopy(wpose))
        plan, fraction = self.move_group.compute_cartesian_path(waypoints,   # waypoints to follow
                                                                0.001,        # eef_step
                                                                0.0)         # jump_threshold
        if fraction > percent_capable:
            self.move_group.execute(plan, wait=True)

    def generate_known_locations(self):
        locations = {}
        
        name = 'default_home'
        joint_angles = [0.0, radians(-90.0), 0.0, radians(-90.0), 0.0, 0.0]
        locations[name] = joint_angles

        name = 'camera_home'
        joint_angles = [radians(-17.72), radians(-95.26), radians(93.72), radians(-88.65), radians(-89.95), radians(-17.77)]
        locations[name] = joint_angles
        
        name = 'lined_home'
        joint_angles = [0.0, radians(-90.00), radians(90.00), radians(-90.00), radians(-90.00), radians(-90.00)]
        locations[name] = joint_angles
        
        name = 'double_zone'
        joint_angles = [radians(-18.89), radians(-91.25), radians(90.00), radians(-90.00), radians(-90.00), radians(-108.64)]
        locations[name] = joint_angles
        
        name = 'zone1'
        joint_angles = [radians(-1.86), radians(-90.39), radians(89.14), radians(-89.93), radians(-90.37), radians(-91.60)]
        locations[name] = joint_angles
        
        name = 'zone2'
        joint_angles = [radians(-33.95), radians(-86.36), radians(84.99), radians(-89.82), radians(-89.67), radians(-123.70)]
        locations[name] = joint_angles
        
        return locations
    
if __name__ == '__main__':
    rospy.init_node('idk_vision_planner', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    
    # RealSense camera
    camera = realsense_camera(debug=True, roi=0.90)

    # UR5 Controller
    # robot = ur_moveit_planner()
    
    # x_offset = -0.015 - (10.5 / 1000)
    # y_offset = 0.042 + (0.055)
    
    # x_offset = -0.015
    # y_offset = 0.042

    # robot.go_to_known_location('zone1')
    # rospy.sleep(3)
    # target_z1 = camera.generate_trajectories()
    # contours_z1 = deepcopy(camera.contours)
    
    # rospy.sleep(2)
    
    # robot.go_to_known_location('zone2')
    # rospy.sleep(3)
    # target_z2 = camera.generate_trajectories()
    # contours_z2 = deepcopy(camera.contours)
    
    # matched_contour = []
    
    # # cnt1 = contours_z1[0]
    # for cnt1 in contours_z1:
    #     for cnt2 in contours_z2:
    #         approx1 = cv2.approxPolyDP(cnt1, 0.01 * cv2.arcLength(cnt1, True), True)
    #         approx2 = cv2.approxPolyDP(cnt2, 0.01 * cv2.arcLength(cnt2, True), True)
    #         if len(approx1) == len(approx2):
    #             print('FOUND SOMETHING')
    #             matched_contour.append(cnt2)
                
    #         # difference = cv2.matchShapes(cnt1, cnt2, 2, 0.0)
    #         # if difference < 0.01:
    #         #     matched_contour.append(cnt2)
    #         #     print("FOUND SOMETHING")
    
    # frame = deepcopy(camera.color_frame)
    # for contour in matched_contour:
    #     cv2.drawContours(frame, contour, -1, (255, 255, 0), 3)

    # cv2.imwrite('detected.jpg', frame)
        
    # # prevent exiting
    rospy.spin()