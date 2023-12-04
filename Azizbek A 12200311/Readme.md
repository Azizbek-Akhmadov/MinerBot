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


# First, I installed the following documentation:
pip install dynamixel-sdk

<img width="590" alt="Screenshot 2023-12-04 at 10 42 27 PM" src="https://github.com/Azizbek-Akhmadov/MinerBot/assets/81019633/0cb897ff-7773-4285-a9b8-47862d03c658">

Codes: 


<img width="554" alt="Screenshot 2023-12-04 at 10 43 19 PM" src="https://github.com/Azizbek-Akhmadov/MinerBot/assets/81019633/edda6366-5f64-494a-a6bf-cb2a69e2eea5">


<img width="552" alt="Screenshot 2023-12-04 at 10 43 39 PM" src="https://github.com/Azizbek-Akhmadov/MinerBot/assets/81019633/6727331b-b9aa-49ba-965e-3c2eef41a93a">


<img width="842" alt="Screenshot 2023-12-04 at 10 43 58 PM" src="https://github.com/Azizbek-Akhmadov/MinerBot/assets/81019633/9ce34b8b-5a20-4a17-82b5-26b6e1b1451c">















