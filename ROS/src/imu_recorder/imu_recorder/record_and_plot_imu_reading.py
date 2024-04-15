import rclpy
from sensor_msgs.msg import Imu
import time
import os
from ament_index_python.packages import get_package_share_path

class CmdVelSubscriberNode:

    def __init__(self):
        self.node = rclpy.create_node('imu_subscriber')
        self.subscription = self.node.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        self.file_path = './src/imu_recorder/data/imu_values.txt' # Path to the file where values will be stored
        self.file_handle = open(self.file_path, 'w')

    def imu_callback(self, msg:Imu):
        # Extract linear and angular velocities from the Twist message
        x_velocity = msg.angular_velocity.x
        y_velocity = msg.angular_velocity.y
        z_velocity = msg.angular_velocity.z
        x_acceleration = msg.linear_acceleration.x
        y_acceleration = msg.linear_acceleration.y
        z_acceleration = msg.linear_acceleration.z
        

        # Write the velocities to the file
        self.file_handle.write(f'{x_velocity},{y_velocity},{z_velocity},{x_acceleration},{y_acceleration},{z_acceleration}\n')

    def spin(self):
        rclpy.spin(self.node)

    def destroy_node(self):
        self.file_handle.close()
        self.node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = CmdVelSubscriberNode()
    try:
        cmd_vel_subscriber.spin()
    finally:
        cmd_vel_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()