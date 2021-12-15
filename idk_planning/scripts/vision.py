#!/usr/bin/env python

import cv2
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
from copy import deepcopy
from math import atan2, cos, sin, sqrt, pi

# ros stuff
import rospy
from sensor_msgs.msg import Image
        
class RSCamera:
    def __init__(self, debug=False):
        # declaring variables
        self.color_topic = '/camera/color/image_raw'
        self.info_topic  = '/camera/color/camera_info'
        self.depth_topic = '/camera/aligned_depth_to_color/image_raw'
        self.bridge = CvBridge()
        self.color_frame = None
        self.depth_frame = None
        self.width = 1280
        self.height = 720
        
        # wait for topics to dump images out
        rospy.wait_for_message(self.color_topic, Image)
        rospy.wait_for_message(self.depth_topic, Image)
        self.color_sub = rospy.Subscriber(self.color_topic, Image, self.handle_color_image)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.handle_depth_image)
        
        # defining ROI(s)
        self.x_min = 125
        self.y_min = 65
        self.x_max = 1155
        self.y_max = 640
        self.roi_tl = (self.x_min, self.y_min)
        self.roi_br = (self.x_max, self.y_max)
        
        self.narrow_zone = 125
        
        self.narrow_x_min = int(self.width  / 2 - self.narrow_zone / 2)
        self.narrow_y_min = int(self.height / 2 - self.narrow_zone / 2)
        self.narrow_x_max = int(self.width / 2 + self.narrow_zone / 2)
        self.narrow_y_max = int(self.height / 2 + self.narrow_zone / 2)
        self.narrow_roi_tl = (self.narrow_x_min, self.narrow_y_min)
        self.narrow_roi_br = (self.narrow_x_max, self.narrow_y_max)
        
        # reference dimensions (mm)
        self.xref = 125.0
        self.yref = 75.0
        
        # full height measurements (mm -> pixel)
        self.fxpix = 291.0
        self.fypix = 173.0
        self.fhxratio = self.xref / self.fxpix
        self.fhyratio = self.yref / self.fypix
        self.full_ratio = (self.fhxratio + self.fhyratio) / 2.0
        
        # narrow height measurements (mm -> pixel)
        self.nxpix = 460.0
        self.nypix = 272.0
        self.nhxratio = self.xref / self.nxpix
        self.nhyratio = self.yref / self.nypix
        self.narrow_ratio = (self.nhxratio + self.nhxratio) / 2.0
        
        # height level
        self.observable = 'full'
        
        # center and crosshair
        self.camera_center = (int(self.width / 2.0), int(self.height / 2.0))
        
        # output image
        self.output_sub = rospy.Subscriber(self.color_topic, Image, self.display)
        
    def handle_color_image(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
        self.color_frame = cv2.cvtColor(self.color_frame, cv2.COLOR_BGR2RGB)
    
    def handle_depth_image(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
        
    def display(self, _):
        original = deepcopy(self.color_frame)
        
        cont_image = deepcopy(self.color_frame)
        contours = self.find_contours()
        cv2.drawContours(cont_image, contours, -1, (0, 255, 0), 1)
        
        xhair_image = deepcopy(self.color_frame)
        centers = self.find_contour_centers(contours, self.observable)
        for c in centers: 
            self.place_crosshair(xhair_image, c)
            self.place_text(xhair_image, c, str(self.distance_to_camera(c)))
        self.place_crosshair(xhair_image, self.camera_center)
        
        roi_image = deepcopy(self.color_frame)
        cv2.rectangle(roi_image, self.roi_tl, self.roi_br, (0, 0, 255), 1)
        cv2.rectangle(roi_image, self.narrow_roi_tl, self.narrow_roi_br, (0, 0, 255), 1)
        
        pca = deepcopy(self.color_frame)
        for c in contours:
            if self.point_in_roi(self.get_contour_center(c)):
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(pca ,[box],0,(0,0,255),2)
                # self.getOrientation(c, pca)
        
        # cv2.imshow('original', original)
        cv2.imshow('ROIs', roi_image)
        # cv2.imshow('contours', cont_image)
        cv2.imshow('crosshairs', xhair_image)
        # cv2.imshow('PCA', pca)
        
        cv2.waitKey(1)
        
    def find_contours(self, min_area = 100):
        original = deepcopy(self.color_frame)
        bright = self.brighten(original)
        grey = self.greyscale(bright)
        blur = self.mblur(self.gblur(self.gblur(self.gblur(grey))))
        thresh = self.threshold(blur)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        large_contours = [c for c in contours if cv2.contourArea(c) >= min_area]
        return large_contours

    def find_contour_centers(self, contours, filter_type):
        centers = []
        in_roi = self.point_in_roi if filter_type == 'full' else self.point_in_narrow_roi
        for contour in contours:
            center = self.get_contour_center(contour)
            if in_roi(center): centers.append(center)
        return centers
    
    def get_contour_center(self, contour):
        moment = cv2.moments(contour)
        cx = int(moment['m10']/moment['m00'])
        cy = int(moment['m01']/moment['m00'])
        return (cx, cy)

    def greyscale(self, image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    
    def gblur(self, image, kernel = 5):
        k = (kernel, kernel)
        return cv2.GaussianBlur(image, k, 0)
    
    def mblur(self, image, kernel = 5):
        return cv2.medianBlur(image, kernel)
    
    def threshold(self, image):
        _, result = cv2.threshold(image, 240, 255, cv2.THRESH_BINARY_INV)
        return result
    
    def brighten(self, image, alpha = 2.0, beta = 50):
        alpha = np.clip(alpha, 1.0, 3.0)
        beta = np.clip(beta, 0.0, 100.0)
        return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    
    def point_in_roi(self, point):
        px, py = point
        good_x = (px >= self.x_min and px <= self.x_max)
        good_y = (py >= self.y_min and py <= self.y_max)
        return (good_x and good_y)
    
    def point_in_narrow_roi(self, point):
        px, py = point
        good_x = (px >= self.narrow_x_min and px <= self.narrow_x_max)
        good_y = (py >= self.narrow_y_min and py <= self.narrow_y_max)
        return (good_x and good_y)
    
    def place_crosshair(self, image, position, color = (255, 255, 0)):
        return cv2.drawMarker(image, position, color, markerSize=10)
    
    def place_text(self, image, position, text, color = (255, 255, 0), scale = 0.4):
        x, y = position
        x -= int(len(text) * 3.5)
        y += 20
        return cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, lineType=cv2.LINE_AA)
    
    def distance_to_camera(self, point):
        ratio = self.full_ratio if self.observable == 'full' else self.narrow_ratio
        px, py = point
        cx, cy = self.camera_center
        dx = round((px - cx) * ratio / 1000.0, 3) 
        dy = round((py - cy) * ratio / 1000.0, 3)
        return (dx, -dy)
    
    def drawAxis(self, img, p_, q_, colour, scale):
        p = list(p_)
        q = list(q_)
        
        angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
        hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
        # Here we lengthen the arrow by a factor of scale
        q[0] = p[0] - scale * hypotenuse * cos(angle)
        q[1] = p[1] - scale * hypotenuse * sin(angle)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
        # create the arrow hooks
        p[0] = q[0] + 9 * cos(angle + pi / 4)
        p[1] = q[1] + 9 * sin(angle + pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
        p[0] = q[0] + 9 * cos(angle - pi / 4)
        p[1] = q[1] + 9 * sin(angle - pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
        
    def getOrientation(self, pts, img):
        sz = len(pts)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i,0] = pts[i,0,0]
            data_pts[i,1] = pts[i,0,1]
        # Perform PCA analysis
        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
        # Store the center of the object
        cntr = (int(mean[0,0]), int(mean[0,1]))
        
        
        cv2.circle(img, cntr, 3, (255, 0, 255), 2)
        p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
        p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
        self.drawAxis(img, cntr, p1, (0, 255, 0), 1)
        self.drawAxis(img, cntr, p2, (255, 255, 0), 5)
        angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
        
        return angle