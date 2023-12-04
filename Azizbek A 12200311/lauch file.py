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

