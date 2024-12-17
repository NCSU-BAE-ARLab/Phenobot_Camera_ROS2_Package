#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SinglePublisher(Node):
    def __init__(self):
        super().__init__('single_publisher')
        self.connect_publisher_ = self.create_publisher(String, '/robot/connect', 10)
        self.exposureTime_publisher_ = self.create_publisher(String, '/robot/exposureTime', 10)
        self.capture_publisher_ = self.create_publisher(String, '/robot/capture', 10)
        self.contTrigger_publisher_ = self.create_publisher(String, '/robot/contTrigger', 10) 
        self.stopContTrigger_publisher_ = self.create_publisher(String, '/robot/stopContTrigger', 10)
        # self.startPreview_publisher_ = self.create_publisher(String, '/robot/startPreview', 10)  
        self.publish_message()

    def publish_message(self):
        msg = String()
        msg.data = 'connect'
        self.connect_publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # input('Press 1 and enter to publish exposure time.')

        # msg.data = '40'
        # self.exposureTime_publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: "{msg.data}"')

        # input('Press 1 and enter to start capturing.')

        msg.data = 'capture'
        self.capture_publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        # input('Press 1 and enter to trigger the camera.')

        msg.data = 'contTrigger'
        self.contTrigger_publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # msg.data = 'stopContTrigger'
        # self.stopContTrigger_publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: "{msg.data}"')

        # msg.data = 'startPreview'
        # self.startPreview_publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: "{msg.data}"')

        # rclpy.shutdown()  # Shut down after publishing

       
def main(args=None):
    rclpy.init(args=args)
    node = SinglePublisher()
    # node.publish_message()
    rclpy.spin(node)
    # time.sleep(2)
    node.get_logger().info('Publishing ends')
    node.destroy_node()
    rclpy.shutdown() 
    # executor = MultiThreadedExecutor()
    # executor.add_node(node)
    # executor.spin()
    
    node.destroy_node()

if __name__ == '__main__':
    main()