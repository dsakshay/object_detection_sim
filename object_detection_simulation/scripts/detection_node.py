#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from tf2_geometry_msgs import PointStamped

from std_msgs.msg import String

import tf2_ros
# from std_msgs.msg import Float64MultiArray


class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection_node")

        self.sub_image = self.create_subscription(Image, "/front_camera/rgb", self.imageCb, 10)

        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)

        self.pub_loc = self.create_publisher(PoseStamped, "/detected_obj_loc", 10)

        self.pub_state = self.create_publisher(String, "/state", 10)

        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state = "search"


    def imageCb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        twist_msg, loc_msg = self.detectObject(cv_image)
        self.pub_twist.publish(twist_msg)
        self.pub_loc.publish(loc_msg)

        if self.state == "search":
            # Continue searching behavior
            self.pub_twist.publish(self.search_behavior())
        elif self.state == "detect":
            # If the object is detected, switch to move towards behavior
            self.pub_twist.publish(self.move_towards_behavior(loc_msg))
        elif self.state == "move_towards":
            # Continue moving towards the object
            self.pub_twist.publish(self.move_towards_behavior(loc_msg))
        elif self.state == "reached":
            self.pub_twist.publish(self.stopRobotMotion())

        state = String()
        state.data = self.state
        self.pub_state.publish(state)

    def search_behavior(self):
        # Implement a simple search behavior (e.g., spin in place)
        twist_msg = Twist()
        twist_msg.angular.z = 0.1  # Adjust the angular velocity for search behavior
        return twist_msg

    def move_towards_behavior(self, localization_msg):
        # Implement behavior to move towards the object based on localization information
        twist_msg = Twist()
        target_x = 0.5  # Adjust the desired x-coordinate to move towards
        target_y = localization_msg.pose.position.y
        twist_msg.linear.x = 0.1  # Linear velocity
        twist_msg.angular.z = 0.01 * (target_y - localization_msg.pose.position.y)  # Adjust the proportional control
        return twist_msg
    
    def stopRobotMotion(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        return twist_msg

    def detectObject(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 100, 100])
        upper_red = np.array([5, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        

        result = cv2.bitwise_and(frame, frame, mask=mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist_msg = Twist()

        loc_msg = PoseStamped()
        try:
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                moments = cv2.moments(largest_contour)

                if moments["m00"] != 0 and self.state != "reached":
                    cx = float(moments["m10"] / moments["m00"])
                    cy = float(moments["m01"] / moments["m00"])

                    # Move towards the center of the red object (adjust the values as needed)
                    twist_msg.linear.x = 0.5
                    # twist_msg.angular.z = 0.01 * (cx - frame.shape[1] / 2)

                    # Localization data (publishing center coordinates)
                    # loc_msg.data = [cx, cy]
                    loc_msg.header.stamp = self.get_clock().now().to_msg()
                    loc_msg.header.frame_id = "map"
                    
                    # !!! I would then transform the camera frame to map frame to send the pose of the detected object !!!
                    # map_point = self.convertPixelToMap(cx, cy)

                    # loc_msg.pose.position.x = map_point.point.x
                    # loc_msg.pose.position.y = map_point.point.y
                    # loc_msg.pose.position.z = map_point.point.z 

                    # using image coordinates for now - 
                    loc_msg.pose.position.x = cx / 100.0 # because cx is about 643 or so
                    loc_msg.pose.position.y = cy / 100.0 
                    loc_msg.pose.position.z = 0.0 # considering the object to be on the ground


                    # Draw bounding box around the detected object
                    x_img, y_img, w_img, h_img = cv2.boundingRect(largest_contour)
                    cv2.rectangle(frame, (x_img, y_img), (x_img + w_img, y_img + h_img), (0, 255, 0), 2)
                    self.state = "detect"
                else:
                    self.state = "search"
            else:
                self.state = "search"
                self.get_logger().warn("not finding contours, searching..")

            cv2.imshow("Object Detection", frame)
            cv2.waitKey(1)

            return twist_msg, loc_msg
        
        except Exception as e:
            self.get_logger().error(f"Exception as {e}")

    def convertPixelToMap(self, cx, cy):
        # This solution was what I was trying to execute, but couldn't
        # Getting issue with extrapolation of the tf from camera frame to map frame for lookup transform
        point_msg = PointStamped()
        point_msg.header.frame_id = 'robot/camera_front'
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = cx
        point_msg.point.y = cy
        point_msg.point.z = 1.0

        try:
            when = self.get_clock().now() - rclpy.time.Duration(seconds=5.0)

            t = self.tf_buffer.lookup_transform_full(
                target_frame="map",
                target_time=rclpy.time.Time(),
                source_frame="camera_front",
                source_time=when,
                fixed_frame='map',
                timeout=rclpy.duration.Duration(seconds=0.05))
                    
            map_point = self.tf_buffer.transform(point_msg, 'map')
            self.get_logger().warn("here 8")
            return map_point
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Error transforming pixel coordinates to map frame: {e}")
            return None
        

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()