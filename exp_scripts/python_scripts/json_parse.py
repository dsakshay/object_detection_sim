#!/usr/bin/env python3
import ast
import rclpy

from rclpy.node import Node
from std_msgs.msg import String


def getDict(event):

    dict_obj = ast.literal_eval(event)
    return dict_obj


class MinimalSub(Node):
    def __init__(self):
        super().__init__("minimal_sub_node")

        self.subscriber = self.create_subscription(String, "/controller/eventstr", self.strCallback, 1)

    def strCallback(self, msg):
        self.get_logger().info("Received an event")

        topic_dict = getDict(msg.data)

        topic = topic_dict['topic']

        self.get_logger().info(f"topic received is: {topic}")


def main(args=None):
    rclpy.init(args=args)

    min_sub = MinimalSub()

    rclpy.spin(min_sub)

    min_sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

