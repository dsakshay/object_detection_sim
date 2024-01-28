#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Point

import tf2_ros
# from std_msgs.msg import Float64MultiArray

DEBUG = False

def debug_log(node, msg=""):
    if DEBUG:
        node.get_logger().warn(msg)


class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection_node")

        self.bridge = CvBridge()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.pub_point = self.create_publisher(Point, "/obj_center_point", 10)
        # self.pub_goal_pose = self.create_publisher(PoseStamped, "/detected_obj_loc_pose", 10)
        # self.pub_state = self.create_publisher(String, "detection_state", 10)

        self.sub_image = self.create_subscription(Image, "/front_camera/rgb", self.imageCb, 10)
        # self.sub_camera_info = self.create_subscription(CameraInfo, "/front_camera/rgb/camera_info", self.cameraInfoCb, 10)
        
        # self.client_change_detection_status = self.create_client(SetBool, "obj_detection_status")

        # self.camera_model = PinholeCameraModel()
        # self.cam_int_mat = np.zeros((3,3))

        self.get_logger().info("Initialised Detection Node")


    # def sendTransform(self):
    #     pass

    def normalisePoint(self, cv_image, point):
        if point is not None:
            rows = float(cv_image.shape[0])
            cols = float(cv_image.shape[1])
            # print(rows, cols)
            center_x    = 0.5*cols
            center_y    = 0.5*rows
            # print(center_x)
            x = (point.x - center_x)/(center_x)
            y = (point.y - center_y)/(center_y)

            point_out = Point()
            point_out.x = x
            point_out.y = y
            point_out.z = point.z/cv_image.shape[1]

            return point_out
        else:
            raise Exception("Haven't found the object yet, searching..")


    def imageCb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")        
        self.detectObject(cv_image)


    def detectObject(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 100, 100])
        upper_red = np.array([5, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        result = cv2.bitwise_and(frame, frame, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        debug_log(self, f"len of contours in detect objects: {len(contours)}, type of contours: {type(contours)}")

        
        try:
            center_pt = self.findRectangleCenter(mask, contours)
            
            point_out = self.normalisePoint(mask, center_pt)

            self.pub_point.publish(point_out)
        
        except Exception as e:
            self.get_logger().warn(f"{e}")

    
    def findRectangleCenter(self, mask, contours):
        center_point = Point()
        if contours:
            # Find contours
            # contours, _ = cv2.findContours(working_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Filter rectangles based on the number of vertices
            rectangles = []
            for contour in contours:
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) == 4:
                    rectangles.append(contour)
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(mask, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calculate center point of the rectangle
                center_x = x + w // 2
                center_y = y + h // 2

                # Get the center point
                center_point.x = float(center_x)
                center_point.y = float(center_y)
                center_point.z = float(cv2.contourArea(contour))

            out_image = mask.copy()
            line_color = (255, 255, 255)
            cv2.drawContours(out_image, rectangles, -1, line_color, 2)

            self.get_logger().info("Object Detected", throttle_duration_sec=5.0)
            cv2.imshow("Object Detection", out_image)
            cv2.waitKey(1)
            
            return center_point

        else:
            self.get_logger().warn("Not finding contours, still searching..")
            raise Exception("Haven't found the object yet, searching..")



def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()