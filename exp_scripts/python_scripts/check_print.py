#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import os

import yaml

from ament_index_python import get_package_share_directory

from colorama import Fore

class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.node_name = self.get_name()

    def __del__(self):
        print(Fore.RED, f"{self.node_name} crashed.")

        self.get_logger().info(f"{self.node_name} crashed")

    def load_yaml_file(self, yaml_file_path):
        try:
            with open(yaml_file_path, 'r') as file:
                return yaml.safe_load(file)
        except EnvironmentError as err: # parent of IOError, OSError *and* WindowsError where available
            print("Could not load YAML file. Error: ", err)
            return None 
        



def main(args=None):
    rclpy.init(args=args)

    tn = TestNode()
    yaml_params = tn.load_yaml_file(os.path.join(get_package_share_directory("terra_launch_configs"), "configs", "timmy", "general_params.yaml"))
    print(yaml_params)
    # rclpy.spin(tn)

    rclpy.shutdown()


if __name__ == "__main__":
    main()