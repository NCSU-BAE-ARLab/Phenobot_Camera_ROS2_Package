#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SinglePublisher(Node):
    def __init__(self):
        super().__init__('single_publisher')
       
        self.contTrigger_publisher_ = self.create_publisher(String, '/robot/contTrigger', 10)   
        self.publish_message()

    def publish_message(self):
        msg = String()

        msg.data = 'contTrigger'
        self.contTrigger_publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


       
def main(args=None):
    rclpy.init(args=args)
    node = SinglePublisher()
    # node.publish_message()
    # rclpy.spin(node)
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