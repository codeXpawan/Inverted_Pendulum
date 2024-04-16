import rclpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_path
import numpy as np

class CmdVelSubscriberNode:

    def __init__(self):
        self.node = rclpy.create_node('robot_control')
        self.subscription = self.node.create_subscription(
            Imu,
            '/imu',
            self.control_callback,
            10)
        self.velocity_pub = self.node.create_publisher(Twist,'/cmd_vel',10)
        self.K_p = 10
        self.K_i = 1
        self.K_d = 0.1
        self.previous_error = 0
        self.previous_sum_error = 0
        
        
        
    def quaternion_to_euler(self,q_w,q_x,q_y,q_z):
        # Convert quaternion to Euler angles
        roll = np.arctan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))
        pitch = np.arcsin(2 * (q_w * q_y - q_z * q_x))
        yaw = np.arctan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
        
        return (roll,pitch,yaw)
        
    def control_callback(self, msg:Imu):
        roll,pitch,yaw = self.quaternion_to_euler(msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z)
        #one big issue is faced that is due to the I_gain, the output is high due to this. Fixing this issue by making the previous_sum_error = 0 whenever the angle is 90.i.e parallel to ground
        if self.previous_sum_error > 1 or self.previous_sum_error < -1:
            self.previous_sum_error = 0
        #roll is the inclined angle of the robot
        linear_velocity = self.compute_pid(roll,0.0)
        print(linear_velocity)
        velocity = Twist()
        velocity.linear.x = linear_velocity
        self.velocity_pub.publish(velocity)
        
    def compute_pid(self,observed_value,setpoint):
        error = observed_value - setpoint
        p_gain = self.K_p * error
        self.previous_sum_error += error
        i_gain = self.K_i * self.previous_sum_error
        diff = self.previous_error - error
        self.previous_error = error
        d_gain = self.K_d * diff
        
        return p_gain+i_gain+d_gain
        

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