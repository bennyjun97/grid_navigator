# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np


class PathwayDetector(Node):
    def __init__(self):
        super().__init__('path_finder')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.pathway_publisher_ = self.create_publisher(
            Int16MultiArray,
            'pathway_info',
            10)
        self.image_publisher_ = self.create_publisher(
            Image,
            'image_visual',
            10)
        self.bridge = CvBridge()
        self.height = 480
        self.width = 640
        self.green = [33, 38, 41, 90, 255, 255]
        self.min_area = 100

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        pathway_msg = Int16MultiArray()
        image_msg = Image()
        msg1, msg2 = self.boundary_image(cv_image)
        pathway_msg.data = msg1
        image_msg = msg2
        self.pathway_publisher_.publish(pathway_msg)
        self.image_publisher_.publish(image_msg)

    def empty(a):
        pass

    # create canny of the image
    def canny(self, img, t1, t2):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, t1, t2)
        return canny

    # trim the canny image with given region of interest (polygonal)
    def roi(self, img):
        my_shape = np.array([[
                (0, self.height), (0, int(self.height*0.5)),
                (self.width, int(self.height*0.5)), (self.width, self.height)]])
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, my_shape, 255)
        img_copy = img.copy()
        masked_image = cv2.bitwise_and(img_copy, mask)
        return masked_image

    # find hough lines of the image
    def find_lines(self, img, pix, ang, votes, len, gap):
        return cv2.HoughLinesP(img, pix, ang*np.pi/180, votes, minLineLength=len, maxLineGap=gap)

    # find left and right boundary of the path
    def find_boundary(self, lines, angle_upp, angle_low):
        left_l_sq = 0
        right_l_sq = 0
        m1, b1, m2, b2 = 0, 0, 0, 0
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                if x1 != x2:
                    m = (y2-y1)/(x2-x1)
                    b = y1-m*x1
                    slope_deg = np.arctan(-1*m)*180/np.pi
                    slope_deg_abs = np.abs(slope_deg)
                    if slope_deg_abs > angle_upp or slope_deg_abs < angle_low:
                        continue
                    squared = (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)
                    if slope_deg > 0 and squared > left_l_sq:
                        left_l_sq = squared
                        m1, b1 = m, b
                    elif slope_deg < 0 and squared > right_l_sq:
                        right_l_sq = squared
                        m2, b2 = m, b
        return m1, b1, m2, b2

    # given two boundary lines, find the centerline's top and bottom points, along with error value
    def find_center(self, m1, b1, m2, b2):
        if m1*b1*m2*b2 != 0:
            # x_peak = (b2-b1)/(m1-m2)
            x_top = int((0-b1/m1-b2/m2)/2)
            x_bottom = int(((self.height-b1)/m1+(self.height-b2)/m2)/2)
            # x_center = x_top+(x_bottom-x_top)*2
            # color = self.get_color(x_peak, x_center)
            err = int((x_top+x_bottom)/2-self.width/2)
            color = self.get_color(err, 100)
            return x_top, x_bottom, err, color
        else:
            return 0, 0, 1000, (0, 0, 0)

    # create a visual color, greener the better, redder the worse
    def get_color(self, err, thres):
        errabs = np.abs(err)
        if errabs > thres:
            errabs = thres
        diff = int(errabs*510/thres)
        green = 510-diff
        if green > 255:
            green = 255
        red = diff
        if red > 255:
            red = 255
        color = (0, green, red)
        return color

    # show candidate lines in a new window
    def show_lines(self, img, lines):
        img_copy = img.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)
                cv2.line(img_copy, (x1, y1), (x2, y2), (255, 0, 0), 10)
        return img_copy

    # show the center of the path
    def show_center(self, img, x_top, x_bottom, color):
        img_copy = img.copy()
        if x_top != 0:
            cv2.line(img_copy, (x_top, 0), (x_bottom, self.height), color, 10)
        return img_copy

    # find the marker and return the height of the marker in the frame
    def find_marker(self, img, color):
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array(color[0:3])
        upper = np.array(color[3:6])
        mask = cv2.inRange(imgHSV, lower, upper)
        cont = self.getContours(mask)
        marker_x, marker_y = self.marker_height(cont)
        if marker_y == 0:
            marker_err = 1000
        else:
            marker_err = int((marker_y-self.height*0.5)/self.height*100)
        marker_color = self.get_color(marker_err, 20)
        return marker_x, marker_y, marker_err, marker_color

    # get biggest contour with area larger than the minimum area
    def getContours(self, img):
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        maxArea = 0
        biggest = [[[0, 0]], [[0, 0]], [[0, 0]], [[0, 0]]]
        for cnt in contours:
            area = cv2.contourArea(cnt)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02*peri, True)
            if area > self.min_area and area > maxArea and len(approx) == 4:
                biggest = approx
                maxArea = area
        return biggest

    # given 4 points, return average height in the frame
    # If no marker is found, return different value
    def marker_height(self, cont):
        x = 0
        y = 0
        for point in cont:
            x += point[0][0]
            y += point[0][1]
        return int(x/4), int(y/4)

    # return path error and marker height error
    def boundary_image(self, img):
        # set parameters
        threshold1, threshold2, pix, ang, minimum_votes, minimum_length = 20, 20, 2, 1, 100, 40
        gap, angle_upp, angle_low = 5, 89, 45
        # get canny of image
        img_canny = self.canny(img, threshold1, threshold2)
        # get canny with region of interest
        img_roi = self.roi(img_canny)
        # get candidate lines from canny image
        lines = self.find_lines(img_roi, pix, ang, minimum_votes, minimum_length, gap)
        # find left and right boundary
        m1, b1, m2, b2 = self.find_boundary(lines, angle_upp, angle_low)
        # find and draw center line
        x_top, x_bottom, err, color = self.find_center(m1, b1, m2, b2)
        img_center = self.show_center(img, x_top, x_bottom, color)
        # find the marker
        marker_x, marker_y, marker_err, marker_color = self.find_marker(img, self.green)
        if marker_x != 0:
            img_center = cv2.circle(img_center, (marker_x, marker_y), 10, marker_color, -1)
        img_out = img_center
        error_string = str(err)
        marker_string = str(marker_err) + '%'
        if err == 1000:
            error_string = 'N/A'
        if marker_err == 1000:
            marker_string = 'N/A'
        cv2.putText(img_out, 'pathway offset from center: ' + error_string, (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(img_out, 'marker offset from center : ' + marker_string, (5, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        img_final = self.bridge.cv2_to_imgmsg(img_out, encoding='bgr8')
        return [err, marker_err], img_final


def main(args=None):
    rclpy.init(args=args)

    pathway_detector = PathwayDetector()

    rclpy.spin(pathway_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pathway_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
