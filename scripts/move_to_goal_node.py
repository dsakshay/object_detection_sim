#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point, PoseStamped, Quaternion

import math


class MoveToGoalNode(Node):
    def __init__(self):
        super().__init__("move_to_goal_node")

        self.pub_twist = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "obj_pose", 10)

        self.sub_loc_det_obj = self.create_subscription(Point, "/obj_center_point", self.objCentrePtCb, 10)

        self.timeout = 1.0

        self.max_area_thresh = 75.0

        self.timer_period = 0.1

        self.timer = self.create_timer(self.timer_period, self.timerCb)

        self.target_val = 0.0

        self.target_dist = 0.0

        # From camera plugin in robot.sdf
        self.im_width = 1280
        self.im_height = 720
        self.h_fov = 1.22
        self.aspect_ratio = self.im_width / self.im_height

        # calculate vfov assuming camera is a rectangle and using pinhole camera model
        self.v_fov = self.h_fov / self.aspect_ratio

        self.last_rec_time = self.get_clock().now().nanoseconds*1e-9 - 1000


    def timerCb(self):
        msg = Twist()
        now = self.getTimeSeconds()
        if (now - self.last_rec_time < self.timeout):
            self.get_logger().warn(f"Target: {self.target_val}", throttle_duration_sec=2.0)
            self.get_logger().warn(f"Target distance: {self.target_dist}", throttle_duration_sec=2.0)
            if (self.target_dist < self.max_area_thresh):
                msg.linear.x = 0.5
            msg.angular.z = -0.7*self.target_val
        else:
            self.get_logger().info('Target lost')
            msg.angular.z = 0.5
        self.pub_twist.publish(msg)

    def objCentrePtCb(self, msg):
        filter = 0.9
        if msg is not None:
            self.target_val = self.target_val * filter + msg.x * (1 - filter)
            self.target_dist = self.target_dist * filter + msg.z * (1-filter)
            self.last_rec_time = self.get_clock().now().nanoseconds*1e-9

            ang_size_x = msg.x * self.h_fov
            ang_size_y = msg.y * self.v_fov

            area = abs(ang_size_x) * abs(ang_size_y)
            
            # Calculate angular and distance deviations in X and Y
            x_ang = ang_size_x / 2
            x = math.sqrt(area) * math.tan(x_ang)

            y_ang = ang_size_y / 2
            y = math.sqrt(area) * math.tan(y_ang)

            # Set the z-coordinate as the area of the rectangle
            z = area

            # Publish the pose of the rectangle in 3D
            pose = PoseStamped()

            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "robot/camera_front"

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation = Quaternion()  # Assuming no rotation for simplicity

            # Not working
            self.pub_pose.publish(pose)
        
        else:
            self.get_logger().warn("Haven't found the object yet", throttle_duration_sec=2.0)



    def getTimeSeconds(self):
        return self.get_clock().now().nanoseconds*1e-9


def main(args=None):
    rclpy.init(args=args)

    mtg_node = MoveToGoalNode()

    rclpy.spin(mtg_node)

    mtg_node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

