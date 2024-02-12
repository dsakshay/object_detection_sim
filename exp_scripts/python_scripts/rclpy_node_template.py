#!/usr/bin/env python3
# include the shebang at the start of every script


# Import only necessary functions
import rclpy
from rclpy.node import Node
import logging

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

from std_msgs.msg import String, Int8
from fpn_msgs.srv import BoolSrv 

'''
This is a template for a rclpy node!

This template could be used as a guide while developing rclpy nodes.

Logging - involves both dev & prod logs - use extensive loggers wherever needed. 

try-except-finally block to be used in all important functions.

init and del functions to be mandatory.
'''

# Define the node class using the following
class GeneralNodeName(Node):
    # define the init function that is required as the node starts
    def __init__(self):
        # super function to initialise the name of the node
        super().__init__("generic_node_name")
        # define class variables that may required for the node
        self.node_name = self.get_name()

        # define a qos profile based on the publishers and subscribers being used in the node
        self.std_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # initialise all the methods required for the functioning of the node
        self.initParams()

        self.initPublishers()

        self.initSubscribers()
        
        self.initServices()

    # define the del function which acts as a destructor function
    def __del__(self):
        # maintain the logger statements to be somewhat similar to the ones below
        self.get_logger().error(f"{self.node_name} got killed!")
        self.myprodlog(f"{self.node_name} got killed!")
    
    # define a function to log the errors into a file during production
    def myprodlog(self, level, msg=''):
        if self.prod_logs:
            match level:
                case logging.DEBUG:
                    self.prod_logger.debug(msg)
                case logging.INFO:
                    self.prod_logger.info(msg)
                case logging.ERROR:
                    self.prod_logger.error(msg)
                case logging.WARNING:
                    self.prod_logger.warning(msg)

    # define a function to initialise the production logging file, format and level
    def initProdLogger(self):
        logging.basicConfig(filename=self.log_file, format='%(name)s:%(levelname)s:%(message)s')
        logger = logging.getLogger(self.node_name)
        match self.log_level:
            case "DEBUG":
                self.prod_logger.setLevel(logging.DEBUG)
            
            case "INFO":
                self.prod_logger.setLevel(logging.INFO)
            
            case "WARNING":
                self.prod_logger.setLevel(logging.WARNING)

            case "ERROR":
                self.prod_logger.setLevel(logging.ERROR)

    # def the init params method which declares and gets the parameter values to class variables
    def initParams(self):
        self.declare_parameter("generic_parameter", "default_value")

        self.log_level = self.get_parameter("generic_parameter").get_parameter_value().string_value

    # define a initPublishers function to define all publishers for the node
    def initPublishers(self):
        # defining a generic publisher for example purposes
        self.pub_datalog_switch = self.create_publisher(String, 'datalog_switch', self.std_qos)

    # define a initSubscribers function to define all subscribers for the node
    def initSubscribers(self):
        self.sub_datalog_switch = self.create_subscription(Int8,'datalog_switch_request', self.datalogSwitchCb, self.std_qos)

    # if needed define a function to define all the service clients, servers
    def initServices(self):
        self.toggle_record = self.create_service(BoolSrv, "trigger_service", self.triggerCb)
        self.generic_client = self.create_client(BoolSrv, "stop_srv_name")

    # define a run function that is supposed to run in a loop when the node is alive
    def run(self):
        # add code that needs to be run in a loop
        pass

    def datalogSwitchCb(self, msg):
        pass

    def triggerCb(self, request, response):
        pass

# define a main function that handles the initialisation of the node and its running
def main(args=None):
    # initialise rclpy
    rclpy.init(args=args)

    # initialise the node
    gen_node = GeneralNodeName()

    # use a callback to run a timer so that run function can be targeted to the timer
    gen_node.run_cb = gen_node.create_timer(1/20, gen_node.run)
    # 1/20 is the time period = 20Hz (frequency of the node)
    
    try:
        rclpy.spin(gen_node)
    except KeyboardInterrupt:
        gen_node.get_logger().info('Keyboard interrupt called!')
    except Exception as e:
        gen_node.get_logger().error(f'Exception: {e}')
    finally:
        gen_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()