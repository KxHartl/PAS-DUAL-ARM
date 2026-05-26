#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

class MainTaskNode(Node):
    def __init__(self):
        super().__init__('main_task_node')
        self.get_logger().info('Main Task Node initialized.')
        
        # Here we will define ActionClients for Nav2 (NavigateToPose) 
        # and MoveIt2 (MoveGroup for both arms).
        
    def execute_task(self):
        self.get_logger().info('Step 1: Navigating to the pre-grasp area...')
        # Call Nav2 Action
        time.sleep(2.0)
        
        self.get_logger().info('Step 2: Looking for Aruco Marker ID 0...')
        # Wait for tf transform from base_link to aruco_marker_frame
        time.sleep(2.0)
        
        self.get_logger().info('Step 3: Calculating Dual-Arm IK and planning grasp...')
        # Call MoveIt to plan left and right arms
        time.sleep(2.0)
        
        self.get_logger().info('Step 4: Executing grasp and lifting the box...')
        # Execute MoveIt trajectory
        time.sleep(2.0)
        
        self.get_logger().info('Step 5: Navigating through the 0.8m doorway to the table...')
        # Call Nav2 to X=3.5, Y=0
        time.sleep(2.0)
        
        self.get_logger().info('Step 6: Placing the box on the table...')
        # MoveIt un-grasp
        time.sleep(2.0)
        
        self.get_logger().info('Task Completed Successfully!')

def main(args=None):
    rclpy.init(args=args)
    node = MainTaskNode()
    
    # Run the task in a separate thread or just block
    node.execute_task()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
