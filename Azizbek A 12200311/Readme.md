# TurtleBot3 Run Motor

Install ROS 2
Install TurtleBot3 Packages: which I found these packages on the official TurtleBot GitHub repository: https://github.com/ROBOTIS-GIT/turtlebot3
Workspace: colcon build –symlink-install
Source our ROS 2 workspace: source install/setup.bash
ROS 2 code to Control Motors:

import rclpy
from geometry_msgs.msg import Twist

def control_turtlebot3():
    rclpy.init()
    node = rclpy.create_node('turtlebot3_controller')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

   Easy way to move forward
    twist = Twist()
    twist.linear.x = 0.2  
    twist.angular.z = 0.0  

    while rclpy.ok():
        pub.publish(twist)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    control_turtlebot3()


# I created a new ROS2 package to Control Motor:
Ros2 pkg create my_turtlebot3_control
Build and Run Your Code:
Workspace again: colcon build --symlink-install
Source our workspace colcon build --symlink-install
Run our code: ros2 run my_turtlebot3_control my_turtlebot3_control_node












